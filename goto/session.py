"""Thread-safe controller around the goto stack for remote (BLE) operation.

The CLI in ``goto.py`` is a one-shot: parse args, init hardware, slew,
track, exit. A remote controller needs the inverse:

  * hardware stays initialized across many goto requests,
  * a worker thread runs the current operation (slew/track/calibrate),
  * commands are async — return immediately, advertise progress via
    status snapshots and a log stream.

``TrackerSession`` is that controller. It owns the IMU, servo bus,
strategy, and capture pipeline; everything that talks to hardware goes
through it, serialised by a single worker thread. The BLE server polls
``snapshot()`` for telemetry and pushes ``Command`` objects via
``submit()``.
"""

from __future__ import annotations

import os
import time
import json
import queue
import threading
import traceback
from dataclasses import dataclass, field
from typing import Any, Callable, Optional

from astropy.coordinates import SkyCoord

from . import tracker as _tracker  # for module-level `running` (SIGINT)
from .config import (
    OBSERVER_LAT, OBSERVER_LON, SERVO_PORT,
    MODE_IMU, MODE_NDOF, MODE_SERVO,
    SHOULDER_ROT, SHOULDER_PITCH, ELBOW, WRIST_PITCH, WRIST_ROLL,
    WHEEL_IDS, ADDR_LOCK, ADDR_MODE, CAL_POSES, POSES_DIR,
    STAR_CATALOG, SOLAR_SYSTEM,
    save_pose, load_pose, angle_diff,
)
from .imu import init_imu, read_imu, calib_str, calibrate_imu, gravity_pitch
from .servos import init_servos, move_to_pose, stop_wheels
from .celestial import resolve_target, compute_altaz
from .strategy import WristOnlyStrategy
from .tracker import (
    slew_to_target, track_target,
    init_camera, init_speckle_pipeline,
)


# ── Command queue payload ──

@dataclass
class Command:
    cmd: str
    req: Optional[int] = None
    args: dict = field(default_factory=dict)


@dataclass
class Config:
    mode: str = "ndof"           # "ndof" | "imu"
    exposure: int = 10000
    burst_count: int = 1
    capture: bool = False        # default off — BLE users usually just point
    capture_dir: str = "speckle_captures"


# ── Session ──

class TrackerSession:
    def __init__(self, log_sink: Optional[Callable[[str], None]] = None):
        self._log_sink = log_sink or (lambda line: None)
        self._lock = threading.Lock()
        self._cmd_q: "queue.Queue[Command]" = queue.Queue()
        self._stop_evt = threading.Event()       # interrupts current op
        self._shutdown_evt = threading.Event()   # ends the worker thread
        self._t0 = time.monotonic()

        self.config = Config()
        self.observer_lat = OBSERVER_LAT
        self.observer_lon = OBSERVER_LON

        # Hardware handles (None until initialize_hw runs)
        self.imu_bus = None
        self.ph = None
        self.pkt = None
        self.strategy: Optional[WristOnlyStrategy] = None
        self.cam = None
        self.pipeline = None
        self.locked_pose_name: Optional[str] = None

        # Live status (read by snapshot())
        self._status = {
            "state": "init",
            "target": None,
            "target_alt": None, "target_az": None,
            "imu_heading": None, "imu_pitch": None,
            "az_err": None, "alt_err": None,
            "calib": "",
            "phase": None,
            "corrections": 0, "captures": 0,
            "error": None,
        }

        self._worker = threading.Thread(target=self._run, daemon=True, name="tracker-worker")
        self._worker.start()

    # ── Public API ──

    def submit(self, cmd: Command) -> None:
        """Queue a command. Returns immediately."""
        self._cmd_q.put(cmd)

    def request_stop_current(self) -> None:
        """Interrupt the current operation (slew/track). Worker stays alive."""
        self._stop_evt.set()

    def shutdown(self) -> None:
        self._shutdown_evt.set()
        self._stop_evt.set()
        self._cmd_q.put(Command("__shutdown"))
        self._worker.join(timeout=10)
        self._close_hw()

    def snapshot(self) -> dict:
        """Latest status as a plain dict (safe to JSON-encode)."""
        with self._lock:
            s = dict(self._status)
        s["v"] = 1
        s["observer"] = {"lat": self.observer_lat, "lon": self.observer_lon}
        s["mode"] = self.config.mode
        s["hw"] = {
            "imu": self.imu_bus is not None,
            "servos": self.ph is not None,
            "camera": self.cam is not None,
        }
        s["capture"] = {
            "enabled": self.config.capture and self.pipeline is not None,
            "burst_count": self.config.burst_count,
        }
        s["locked_pose"] = self.locked_pose_name
        s["uptime"] = round(time.monotonic() - self._t0, 1)
        return s

    def list_poses(self) -> list:
        if not os.path.isdir(POSES_DIR):
            return []
        out = []
        for fn in sorted(os.listdir(POSES_DIR)):
            if not fn.endswith(".json"):
                continue
            name = fn[:-5]
            if name == "pitch_cal":
                continue
            try:
                with open(os.path.join(POSES_DIR, fn)) as f:
                    data = json.load(f)
                out.append({
                    "name": name,
                    "timestamp": data.get("timestamp"),
                    "heading": data.get("heading"),
                    "pitch": data.get("pitch"),
                })
            except Exception:
                continue
        return out

    def info(self) -> dict:
        return {
            "v": 1,
            "name": "StarTracker",
            "version": "1.0",
            "stars": sorted(STAR_CATALOG.keys()),
            "planets": list(SOLAR_SYSTEM),
        }

    # ── Worker loop ──

    def _run(self) -> None:
        # The worker owns all hardware I/O. Other threads only enqueue.
        try:
            self._initialize_hw()
            self._set_state("idle")
        except Exception as e:
            self._log(f"E init failed: {e}")
            self._set_state("error", error=str(e))

        while not self._shutdown_evt.is_set():
            try:
                cmd = self._cmd_q.get(timeout=0.5)
            except queue.Empty:
                # idle tick — refresh IMU into status so the UI sees live readout
                self._refresh_idle_status()
                continue

            if cmd.cmd == "__shutdown":
                break

            self._stop_evt.clear()
            self._dispatch(cmd)

    def _dispatch(self, cmd: Command) -> None:
        c = cmd.cmd
        a = cmd.args
        try:
            if c == "goto":
                self._do_goto(cmd)
            elif c == "stop":
                # Already interrupted via request_stop_current; just confirm.
                self._log(f"D {cmd.req or ''} stop:ok")
            elif c == "park":
                self._do_park(cmd)
            elif c == "set_observer":
                self._do_set_observer(cmd)
            elif c == "set_pose":
                self._do_set_pose(cmd)
            elif c == "record_pose":
                self._do_record_pose(cmd)
            elif c == "delete_pose":
                self._do_delete_pose(cmd)
            elif c == "calibrate_imu":
                self._do_calibrate_imu(cmd)
            elif c == "calibrate_pitch":
                self._log(f"E {cmd.req or ''} calibrate_pitch: not yet remoteable (use CLI)")
            elif c == "set_config":
                self._do_set_config(cmd)
            elif c == "reinit_hw":
                self._close_hw()
                self._initialize_hw()
                self._set_state("idle")
                self._log(f"D {cmd.req or ''} reinit_hw:ok")
            elif c == "refresh_poses":
                self._log(f"D {cmd.req or ''} refresh_poses:ok")  # server handles notify
            else:
                self._log(f"E {cmd.req or ''} unknown cmd: {c}")
        except Exception as e:
            traceback.print_exc()
            self._log(f"E {cmd.req or ''} {c}: {e}")
            self._set_state("error", error=f"{c}: {e}")

    # ── Command handlers ──

    def _do_goto(self, cmd: Command) -> None:
        if self.imu_bus is None or self.ph is None:
            raise RuntimeError("hardware not initialized")

        a = cmd.args
        if "target" in a:
            name = str(a["target"])
            info = resolve_target(name)
            if info is None:
                raise ValueError(f"unknown target '{name}'")
        elif "ra" in a and "dec" in a:
            name = f"RA={a['ra']} Dec={a['dec']}"
            info = ("star", SkyCoord(ra=a["ra"], dec=a["dec"], frame="icrs"))
        elif "alt" in a and "az" in a:
            name = f"Alt={a['alt']}° Az={a['az']}°"
            info = ("fixed", (float(a["alt"]), float(a["az"])))
        else:
            raise ValueError("goto needs target | (ra,dec) | (alt,az)")

        alt, az = compute_altaz(info)
        if alt < 0:
            raise ValueError(f"target below horizon (alt={alt:.1f}°)")

        with self._lock:
            self._status["target"] = name

        self._log(f"I {cmd.req or ''} goto {name}: alt={alt:.1f}° az={az:.1f}°")
        active_mode = MODE_NDOF if self.config.mode == "ndof" else MODE_IMU

        def should_stop():
            return self._stop_evt.is_set() or not _tracker.running

        def status_cb(d):
            with self._lock:
                self._status.update(d)

        self._set_state("slewing")
        ok = slew_to_target(
            info, self.imu_bus, self.ph, self.pkt, self.strategy,
            target_name=name, pipeline=self.pipeline, active_mode=active_mode,
            should_stop=should_stop, status_cb=status_cb,
        )
        if not ok or should_stop():
            self._set_state("idle")
            self._log(f"D {cmd.req or ''} goto:stopped")
            return

        self._set_state("tracking")
        track_target(
            info, name, self.imu_bus, self.ph, self.pkt, self.strategy,
            pipeline=self.pipeline, active_mode=active_mode,
            should_stop=should_stop, status_cb=status_cb,
        )
        self._set_state("idle")
        self._log(f"D {cmd.req or ''} goto:done")

    def _do_park(self, cmd: Command) -> None:
        if self.ph is None:
            raise RuntimeError("servos not initialized")
        self._set_state("parking")
        move_to_pose(self.ph, self.pkt, CAL_POSES["home"])
        self._set_state("idle")
        self._log(f"D {cmd.req or ''} park:ok")

    def _do_set_observer(self, cmd: Command) -> None:
        lat = float(cmd.args["lat"])
        lon = float(cmd.args["lon"])
        self.observer_lat = lat
        self.observer_lon = lon
        # also patch the module-level constants used by celestial.py
        from . import config as cfg
        cfg.OBSERVER_LAT = lat
        cfg.OBSERVER_LON = lon
        self._log(f"D {cmd.req or ''} set_observer:{lat:.4f},{lon:.4f}")

    def _do_set_pose(self, cmd: Command) -> None:
        name = str(cmd.args["name"])
        all_positions = load_pose(name)
        locked = {k: v for k, v in all_positions.items()
                  if k in [SHOULDER_PITCH, ELBOW]}
        if not locked:
            raise ValueError(f"pose '{name}' has no M2/M3 positions")
        # re-initialize servos with locked pose
        self._close_servos()
        self.ph, self.pkt = init_servos(locked_pose=locked)
        self.strategy = WristOnlyStrategy()
        if self.imu_bus is not None:
            self.strategy.calibrate_pitch_dir(self.ph, self.pkt, self.imu_bus)
        self.locked_pose_name = name
        self._log(f"D {cmd.req or ''} set_pose:{name}")

    def _do_record_pose(self, cmd: Command) -> None:
        if self.ph is None or self.imu_bus is None:
            raise RuntimeError("hardware not initialized")
        import scservo_sdk as sdk
        from .config import ADDR_PRESENT_POSITION
        sids = [SHOULDER_ROT, SHOULDER_PITCH, ELBOW, WRIST_PITCH, WRIST_ROLL]
        positions = {}
        for sid in sids:
            pos, result, _ = self.pkt.read2ByteTxRx(self.ph, sid, ADDR_PRESENT_POSITION)
            if result == sdk.COMM_SUCCESS:
                positions[sid] = pos
        imu = read_imu(self.imu_bus)
        save_pose(cmd.args["name"], positions, metadata={
            "heading": imu["heading"], "pitch": imu["pitch"],
        })
        self._log(f"D {cmd.req or ''} record_pose:{cmd.args['name']}")

    def _do_delete_pose(self, cmd: Command) -> None:
        name = str(cmd.args["name"])
        path = os.path.join(POSES_DIR, f"{name}.json")
        if os.path.exists(path):
            os.remove(path)
            self._log(f"D {cmd.req or ''} delete_pose:{name}")
        else:
            raise FileNotFoundError(f"pose '{name}' not found")

    def _do_calibrate_imu(self, cmd: Command) -> None:
        if self.imu_bus is None or self.ph is None:
            raise RuntimeError("hardware not initialized")
        self._set_state("calibrating")
        active_mode = MODE_NDOF if self.config.mode == "ndof" else MODE_IMU

        def move_fn(pose_name):
            move_to_pose(self.ph, self.pkt, CAL_POSES[pose_name])

        calibrate_imu(self.imu_bus, move_fn, active_mode)
        self._set_state("idle")
        self._log(f"D {cmd.req or ''} calibrate_imu:ok")

    def _do_set_config(self, cmd: Command) -> None:
        a = cmd.args
        changed = []
        if "mode" in a and a["mode"] in ("ndof", "imu"):
            self.config.mode = a["mode"]; changed.append(f"mode={a['mode']}")
        if "exposure" in a:
            self.config.exposure = int(a["exposure"]); changed.append(f"exp={a['exposure']}")
        if "burst_count" in a:
            self.config.burst_count = int(a["burst_count"])
            changed.append(f"burst={a['burst_count']}")
        if "capture" in a:
            self.config.capture = bool(a["capture"]); changed.append(f"cap={a['capture']}")
        self._log(f"D {cmd.req or ''} set_config:{','.join(changed) or 'noop'}")

    # ── Hardware helpers ──

    def _initialize_hw(self) -> None:
        active_mode = MODE_NDOF if self.config.mode == "ndof" else MODE_IMU
        self._log("I init: opening IMU")
        self.imu_bus = init_imu(mode=active_mode)
        self._log("I init: opening servos")
        self.ph, self.pkt = init_servos()
        self._log("I init: pitch-dir calibration")
        self.strategy = WristOnlyStrategy()
        self.strategy.calibrate_pitch_dir(self.ph, self.pkt, self.imu_bus)
        if self.config.capture:
            self.cam = init_camera(self.config.exposure)
            self.pipeline = init_speckle_pipeline(
                self.cam, self.config.exposure,
                self.config.burst_count, self.config.capture_dir,
            )
        self._log("I init: ok")

    def _close_servos(self) -> None:
        if self.ph is None:
            return
        try:
            stop_wheels(self.ph, self.pkt)
            for sid in WHEEL_IDS:
                self.pkt.write1ByteTxRx(self.ph, sid, ADDR_LOCK, 0)
                self.pkt.write1ByteTxRx(self.ph, sid, ADDR_MODE, MODE_SERVO)
                self.pkt.write1ByteTxRx(self.ph, sid, ADDR_LOCK, 1)
        finally:
            try:
                self.ph.closePort()
            except Exception:
                pass
            self.ph = None
            self.pkt = None

    def _close_hw(self) -> None:
        self._close_servos()
        if self.pipeline is not None:
            try:
                self.pipeline.wait_for_processing(timeout=10)
            except Exception:
                pass
            self.pipeline = None
        if self.cam is not None:
            try:
                self.cam.close()
            except Exception:
                pass
            self.cam = None
        if self.imu_bus is not None:
            try:
                self.imu_bus.close()
            except Exception:
                pass
            self.imu_bus = None

    def _refresh_idle_status(self) -> None:
        if self.imu_bus is None:
            return
        try:
            imu = read_imu(self.imu_bus, samples=1)
            gp = gravity_pitch(self.imu_bus)
            active_mode = MODE_NDOF if self.config.mode == "ndof" else MODE_IMU
            with self._lock:
                self._status["imu_heading"] = imu["heading"]
                self._status["imu_pitch"] = gp
                self._status["calib"] = calib_str(imu, active_mode)
        except Exception:
            pass

    # ── State / logging ──

    def _set_state(self, state: str, **extra) -> None:
        with self._lock:
            self._status["state"] = state
            self._status["error"] = extra.get("error")
            for k, v in extra.items():
                if k != "error":
                    self._status[k] = v

    def _log(self, line: str) -> None:
        try:
            self._log_sink(line)
        except Exception:
            pass
        print(f"[session] {line}")
