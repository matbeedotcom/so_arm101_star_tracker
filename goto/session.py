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

import asyncio
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
from . import network as _network
from .live_camera import LiveCamera
from .media import MediaBroadcaster
from .image_server import ImageServer
from .config import (
    OBSERVER_LAT, OBSERVER_LON, SERVO_PORT,
    MODE_IMU, MODE_NDOF, MODE_SERVO,
    SHOULDER_ROT, SHOULDER_PITCH, ELBOW, WRIST_PITCH, WRIST_ROLL,
    WHEEL_IDS, ADDR_LOCK, ADDR_MODE, CAL_POSES, POSES_DIR,
    STAR_CATALOG, SOLAR_SYSTEM,
    CAMERA_HFOV_DEG, CAMERA_VFOV_DEG,
    save_pose, load_pose, angle_diff,
)
from .imu import init_imu, read_imu, calib_str, calibrate_imu, gravity_pitch
from .servos import init_servos, move_to_pose, stop_wheels
from .celestial import resolve_target, compute_altaz, next_rise_time
from .scheduler import Scheduler, ScheduledJob
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

        # asyncio loop reference — set by the BLE server once it's
        # running, so the worker thread can schedule media coroutines.
        self._loop: "asyncio.AbstractEventLoop | None" = None
        self.media: Optional[MediaBroadcaster] = None
        self.image_server: Optional[ImageServer] = None
        self.live_camera: Optional[LiveCamera] = None
        self._media_port = 8765
        self._media_token: Optional[str] = None
        self.scheduler: Optional[Scheduler] = None
        self._suggestion: Optional[dict] = None
        self._workspace = os.path.dirname(os.path.dirname(__file__))

        # Set by goto_ble.py CLI flags before the loop starts:
        #   --auto-media → start image_server on boot
        #   --open-media → don't require a token (LAN trust mode)
        self.auto_media = bool(int(os.environ.get("STAR_TRACKER_AUTO_MEDIA", "0")))
        self.media_open = bool(int(os.environ.get("STAR_TRACKER_OPEN_MEDIA", "0")))
        self._net: list[dict] = []
        self._ap: dict = {"active": False, "ssid": None, "passphrase": None,
                          "iface": None, "client_count": 0}

        self.config = Config()
        self.observer_lat = OBSERVER_LAT
        self.observer_lon = OBSERVER_LON
        self.camera_hfov = float(CAMERA_HFOV_DEG)
        self.camera_vfov = float(CAMERA_VFOV_DEG)

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

    def attach_loop(self, loop) -> None:
        """Bind the BLE asyncio loop. Required before media commands work."""
        self._loop = loop
        # Scheduler runs on the BLE loop. Fire callbacks queue normal goto
        # commands into the same worker thread that handles BLE-driven gotos.
        if self.scheduler is None:
            self.scheduler = Scheduler(
                store_path=os.path.join(self._workspace, "schedule.json"),
                on_fire=self._on_scheduled_fire,
            )
            asyncio.run_coroutine_threadsafe(self.scheduler.start(), loop)

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
        # Stop async services on the BLE loop if still attached.
        if self.image_server is not None or self.media is not None:
            try:
                self._async(self._shutdown_media())
            except Exception:
                pass
        self._close_hw()

    async def _shutdown_media(self) -> None:
        if self.scheduler is not None:
            try: await self.scheduler.stop()
            except Exception: pass
        if self.live_camera is not None:
            try: await self.live_camera.stop()
            except Exception: pass
            self.live_camera = None
        if self.image_server is not None and self.image_server.running:
            try: await self.image_server.stop()
            except Exception: pass
        if self.media is not None:
            try: await self.media.stop()
            except Exception: pass

    def snapshot(self) -> dict:
        """High-frequency tick state — must stay under the BLE ATT MTU.

        Slow-changing fields (hw, observer, media, live_preview, …) are
        emitted via ``config_snapshot`` on its own characteristic;
        ``net``/``ap`` and ``schedule``/``suggestion`` likewise. The web
        client merges them into a single Status object for the UI.
        """
        with self._lock:
            s = dict(self._status)
        s["v"] = 1
        s["uptime"] = round(time.monotonic() - self._t0, 1)
        return s

    def config_snapshot(self) -> dict:
        """Slow-changing hw + config state. Notified only on change."""
        lp = (
            self.live_camera.status() if self.live_camera is not None
            else {"active": False, "available": False, "fps_target": 0,
                  "fps_actual": 0.0, "w": 0, "h": 0, "exposure_us": 0,
                  "frames": 0, "import_error": None}
        )
        # Round fps_actual so live preview running at 9.4/9.6/9.8 doesn't
        # cause this characteristic to fire every tick. Drop the running
        # frame counter for the same reason — it's just internal stats.
        lp = dict(lp)
        if "fps_actual" in lp:
            lp["fps_actual"] = round(float(lp["fps_actual"]))
        lp.pop("frames", None)
        return {
            "v": 1,
            "observer": {"lat": self.observer_lat, "lon": self.observer_lon},
            "mode": self.config.mode,
            "hw": {
                "imu": self.imu_bus is not None,
                "servos": self.ph is not None,
                "camera": self.cam is not None,
            },
            "capture": {
                "enabled": self.config.capture and self.pipeline is not None,
                "burst_count": self.config.burst_count,
            },
            "camera": {
                "hfov_deg": self.camera_hfov,
                "vfov_deg": self.camera_vfov,
            },
            "locked_pose": self.locked_pose_name,
            "media": {
                "enabled": self.image_server.running if self.image_server else False,
                "port": self._media_port,
                "token": self._media_token,
                "path": "/live",
            },
            "live_preview": lp,
        }

    def network_snapshot(self) -> dict:
        return {"v": 1, "net": list(self._net), "ap": dict(self._ap)}

    def schedule_snapshot(self) -> dict:
        with self._lock:
            sugg = dict(self._suggestion) if self._suggestion else None
        return {
            "v": 1,
            "schedule": self.scheduler.list_jobs() if self.scheduler else [],
            "suggestion": sugg,
        }

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
            elif c == "enable_media":
                self._async(self._do_enable_media(cmd))
            elif c == "disable_media":
                self._async(self._do_disable_media(cmd))
            elif c == "start_ap":
                self._async(self._do_start_ap(cmd))
            elif c == "stop_ap":
                self._async(self._do_stop_ap(cmd))
            elif c == "live_start":
                self._async(self._do_live_start(cmd))
            elif c == "live_stop":
                self._async(self._do_live_stop(cmd))
            elif c == "schedule":
                self._do_schedule(cmd)
            elif c == "cancel_schedule":
                self._do_cancel_schedule(cmd)
            elif c == "dismiss_suggestion":
                with self._lock:
                    self._suggestion = None
                self._log(f"D {cmd.req or ''} dismiss_suggestion:ok")
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
            # Don't just error — work out when this becomes visible so the
            # client can offer to schedule the goto for then.
            try:
                rise = next_rise_time(info, min_alt=10.0)
            except Exception as e:
                rise = None
                self._log(f"W rise-time calc failed: {e}")
            spec = dict(a) if (a := cmd.args) else {}
            if not spec:
                spec = {"target": name}
            with self._lock:
                self._suggestion = {
                    "action": "schedule" if rise else "out_of_range",
                    "spec": spec,
                    "reason": f"below horizon (alt={alt:.1f}°)",
                    "current_alt": round(alt, 1),
                    "current_az": round(az, 1),
                    "next_visible": (rise or {}).get("iso"),
                    "alt_at_time": (rise or {}).get("alt"),
                    "minutes_from_now": (rise or {}).get("minutes_from_now"),
                }
            raise ValueError(f"target below horizon (alt={alt:.1f}°)")

        with self._lock:
            self._status["target"] = name
            # Successful goto clears any pending suggestion.
            self._suggestion = None

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
        prev_capture = self.config.capture
        if "mode" in a and a["mode"] in ("ndof", "imu"):
            self.config.mode = a["mode"]; changed.append(f"mode={a['mode']}")
        if "exposure" in a:
            self.config.exposure = int(a["exposure"]); changed.append(f"exp={a['exposure']}")
        if "burst_count" in a:
            self.config.burst_count = int(a["burst_count"])
            changed.append(f"burst={a['burst_count']}")
        if "capture" in a:
            self.config.capture = bool(a["capture"]); changed.append(f"cap={a['capture']}")
        if "hfov_deg" in a:
            try:
                v = float(a["hfov_deg"])
                if 1.0 <= v <= 180.0:
                    self.camera_hfov = v; changed.append(f"hfov={v:.1f}")
            except (TypeError, ValueError):
                pass
        if "vfov_deg" in a:
            try:
                v = float(a["vfov_deg"])
                if 1.0 <= v <= 180.0:
                    self.camera_vfov = v; changed.append(f"vfov={v:.1f}")
            except (TypeError, ValueError):
                pass
        # If capture toggled, hand the camera back and forth.
        if "capture" in a and self.config.capture != prev_capture:
            if self.config.capture:
                # Going into speckle mode — release the live stream.
                self._async(self._stop_live_camera())
            else:
                # Speckle just released the camera — start preview if we can.
                if self.cam is None:
                    self._async(self._ensure_live_camera())
        # Tell LiveCamera about new exposure so the preview matches.
        if "exposure" in a and self.live_camera is not None:
            self.live_camera.update_controls(exposure_us=self.config.exposure)
        self._log(f"D {cmd.req or ''} set_config:{','.join(changed) or 'noop'}")

    # ── Async (loop-bound) command handlers ──

    def _async(self, coro) -> None:
        """Run an asyncio coroutine on the BLE loop from this thread.

        If the loop isn't attached yet (running headless without BLE),
        we fall back to a one-shot loop just for this call.
        """
        import asyncio as _aio
        if self._loop is not None and self._loop.is_running():
            fut = _aio.run_coroutine_threadsafe(coro, self._loop)
            try:
                fut.result(timeout=20.0)
            except Exception as e:
                self._log(f"E media: {e}")
            return
        try:
            _aio.run(coro)
        except Exception as e:
            self._log(f"E media: {e}")

    async def _do_enable_media(self, cmd: Command) -> None:
        if self.media is None:
            self.media = MediaBroadcaster(capture_dir=self.config.capture_dir)
            await self.media.start()
        if self.image_server is None:
            self.image_server = ImageServer(
                self.media, port=self._media_port,
                bursts_dir=self.config.capture_dir,
                require_token=not self.media_open,
            )
        if not self.image_server.running:
            token = await self.image_server.start()
            self._media_token = token or None
        self._net = await _network.list_addresses_dict()
        self._log(f"D {cmd.req or ''} enable_media:ok port={self._media_port}"
                  f" auth={'token' if self.image_server.require_token else 'open'}")

    async def _do_disable_media(self, cmd: Command) -> None:
        if self.image_server is not None and self.image_server.running:
            await self.image_server.stop()
        self._media_token = None
        self._log(f"D {cmd.req or ''} disable_media:ok")

    # ── LiveCamera lifecycle ──

    async def _ensure_live_camera(self) -> None:
        """Start LiveCamera if speckle isn't holding the camera."""
        if self.cam is not None:
            return  # speckle has the sensor; live preview cannot run
        if self.media is None:
            return  # no broadcaster to hand frames to
        if self.live_camera is not None and self.live_camera.running:
            return
        if self.live_camera is None:
            self.live_camera = LiveCamera(sink=self.media.publish_array)
        ok = await self.live_camera.start()
        if ok:
            self._log("I live_preview: streaming")
        elif self.live_camera is not None and not self.live_camera.available:
            # picamera2 unavailable / sensor busy — drop the instance so
            # we don't keep retrying.
            self.live_camera = None

    async def _stop_live_camera(self) -> None:
        if self.live_camera is None:
            return
        await self.live_camera.stop()
        self._log("I live_preview: stopped")

    async def _do_live_start(self, cmd: Command) -> None:
        if self.cam is not None:
            raise RuntimeError("speckle owns the camera — disable capture first")
        await self._ensure_live_camera()
        self._log(f"D {cmd.req or ''} live_start:ok")

    async def _do_live_stop(self, cmd: Command) -> None:
        await self._stop_live_camera()
        self.live_camera = None
        self._log(f"D {cmd.req or ''} live_stop:ok")

    # ── Scheduling ──

    def _do_schedule(self, cmd: Command) -> None:
        if self.scheduler is None:
            raise RuntimeError("scheduler not running")
        args = dict(cmd.args)
        at = args.pop("at", None)
        if not at:
            raise ValueError("schedule needs 'at' (ISO 8601 timestamp)")
        note = args.pop("note", "")
        # Anything left is the goto spec (target / ra+dec / alt+az).
        if not args:
            raise ValueError("schedule needs a target spec")
        job = self.scheduler.add(spec=args, at=at, note=note)
        with self._lock:
            self._suggestion = None  # user took action on the suggestion
        self._log(f"D {cmd.req or ''} schedule:{job.id} {at} {args}")

    def _do_cancel_schedule(self, cmd: Command) -> None:
        if self.scheduler is None:
            raise RuntimeError("scheduler not running")
        jid = int(cmd.args.get("id", 0))
        if self.scheduler.cancel(jid):
            self._log(f"D {cmd.req or ''} cancel_schedule:{jid}")
        else:
            raise ValueError(f"job {jid} not found or not pending")

    async def _on_scheduled_fire(self, job: ScheduledJob) -> None:
        # Submit a normal goto through the worker so it runs serialised
        # with anything else the user is doing.
        self._log(f"I scheduler firing job {job.id}: {job.spec}")
        self.submit(Command(cmd="goto", req=None, args=dict(job.spec)))

    async def _do_start_ap(self, cmd: Command) -> None:
        ssid = str(cmd.args.get("ssid") or "StarTracker")
        passphrase = str(cmd.args.get("passphrase") or "")
        iface = str(cmd.args.get("iface") or "wlan0")
        ap = await _network.start_ap(ssid, passphrase, iface=iface)
        self._ap = {
            "active": ap.active, "ssid": ap.ssid, "passphrase": ap.passphrase,
            "iface": ap.iface, "client_count": ap.client_count,
        }
        # Refresh addresses — the new AP interface will have its own IP.
        self._net = await _network.list_addresses_dict()
        self._log(f"D {cmd.req or ''} start_ap:{ssid}")

    async def _do_stop_ap(self, cmd: Command) -> None:
        ap = await _network.stop_ap()
        self._ap = {
            "active": ap.active, "ssid": None, "passphrase": None,
            "iface": None, "client_count": 0,
        }
        self._net = await _network.list_addresses_dict()
        self._log(f"D {cmd.req or ''} stop_ap:ok")

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
            # Speckle pipeline owns the camera — make sure LiveCamera lets go.
            self._async(self._stop_live_camera())
            self.cam = init_camera(self.config.exposure)
            self.pipeline = init_speckle_pipeline(
                self.cam, self.config.exposure,
                self.config.burst_count, self.config.capture_dir,
            )
        else:
            # No speckle — opportunistic live preview from picamera2.
            self._async(self._ensure_live_camera())
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
        # Release LiveCamera before speckle camera/pipeline teardown so
        # both don't fight over picamera2.
        self._async(self._stop_live_camera())
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

    _last_net_refresh: float = 0.0

    def _refresh_idle_status(self) -> None:
        # IMU pulse
        if self.imu_bus is not None:
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
        # Network refresh on a slow cadence — shells out, don't spam.
        now = time.monotonic()
        if now - self._last_net_refresh > 10.0:
            self._last_net_refresh = now
            self._async(self._refresh_network_state())

    async def _refresh_network_state(self) -> None:
        try:
            self._net = await _network.list_addresses_dict()
            ap = await _network.ap_state()
            self._ap = {
                "active": ap.active, "ssid": ap.ssid, "passphrase": ap.passphrase,
                "iface": ap.iface, "client_count": ap.client_count,
            }
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
