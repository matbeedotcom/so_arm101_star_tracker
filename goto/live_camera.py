"""LiveCamera — picamera2-backed low-res video preview.

Runs when the camera is *not* being used by the speckle capture pipeline.
Produces a steady ~10 fps RGB stream, hands frames to MediaBroadcaster,
which fans them out to BLE (downsampled thumbnail, rate-limited) and the
WebSocket image server (full preview, every frame).

Picamera2 only allows one process to own the sensor. To avoid colliding
with ArducamQuadCapture (which the speckle pipeline uses), the session
acquires/releases this object around capture mode — never both at once.

Graceful degradation: if picamera2 isn't importable (dev machines), or
opening fails (no sensor, busy), ``start()`` becomes a no-op and the
class reports ``available = False``. The rest of the system keeps
working off the disk-watching path in MediaBroadcaster.
"""

from __future__ import annotations

import asyncio
import logging
import time
from typing import Awaitable, Callable, Optional

import numpy as np

log = logging.getLogger("live_camera")

try:
    from picamera2 import Picamera2  # type: ignore
    _HAVE_PICAMERA2 = True
    _PICAMERA2_IMPORT_ERROR: Optional[str] = None
except Exception as _e:
    Picamera2 = None  # type: ignore
    _HAVE_PICAMERA2 = False
    # Capture the exception type + message so we don't lose the root
    # cause to a silent "not available" log line. Common causes on the
    # Pi: conda env shadowing system picamera2 with no matching
    # libcamera C extension; ABI mismatch between Python versions.
    _PICAMERA2_IMPORT_ERROR = f"{type(_e).__name__}: {_e}"


FrameSink = Callable[[np.ndarray, dict], Awaitable[None] | None]


class LiveCamera:
    def __init__(
        self,
        sink: FrameSink,
        resolution: tuple[int, int] = (640, 480),
        fps: int = 10,
        exposure_us: int = 20000,
    ):
        self.sink = sink
        self.resolution = resolution
        self.fps = fps
        self.exposure_us = exposure_us

        self._picam: Optional[object] = None  # Picamera2 instance
        self._task: Optional[asyncio.Task] = None
        self._stop = asyncio.Event()
        self._frame_n = 0
        self._fps_actual = 0.0
        self._last_frame_ts = 0.0
        self.available = _HAVE_PICAMERA2

    # ── lifecycle ──

    @property
    def running(self) -> bool:
        return self._task is not None and not self._task.done()

    async def start(self) -> bool:
        """Open the camera and start the pull loop. Returns False if no-op."""
        if not _HAVE_PICAMERA2:
            log.warning(
                "picamera2 unavailable — LiveCamera disabled. import error: %s. "
                "On the Pi this is usually conda shadowing the apt-installed "
                "picamera2 with a Python version that has no matching libcamera "
                "C extension. Run with /usr/bin/python3 (system Python 3.11) or "
                "create a 3.11 conda env per README.",
                _PICAMERA2_IMPORT_ERROR or "(no exception captured)",
            )
            self.available = False
            return False
        if self.running:
            return True

        try:
            self._picam = Picamera2()
            cfg = self._picam.create_preview_configuration(  # type: ignore[attr-defined]
                main={"size": self.resolution, "format": "RGB888"},
                buffer_count=3,
            )
            self._picam.configure(cfg)  # type: ignore[attr-defined]
            # FrameDurationLimits in µs caps the fps from the sensor side.
            frame_us = max(1000, int(1_000_000 / max(1, self.fps)))
            self._picam.set_controls({  # type: ignore[attr-defined]
                "ExposureTime": int(self.exposure_us),
                "FrameDurationLimits": (frame_us, frame_us),
            })
            self._picam.start()  # type: ignore[attr-defined]
        except Exception as e:
            log.warning("LiveCamera open failed: %s", e)
            self._cleanup_picam()
            self.available = False
            return False

        self._stop.clear()
        self._task = asyncio.create_task(self._loop(), name="live-camera")
        log.info("LiveCamera streaming %dx%d @ %d fps",
                 self.resolution[0], self.resolution[1], self.fps)
        return True

    async def stop(self) -> None:
        self._stop.set()
        if self._task is not None:
            try:
                await asyncio.wait_for(self._task, timeout=2.0)
            except asyncio.TimeoutError:
                self._task.cancel()
            self._task = None
        self._cleanup_picam()

    def _cleanup_picam(self) -> None:
        if self._picam is None:
            return
        try:
            self._picam.stop()  # type: ignore[attr-defined]
        except Exception:
            pass
        try:
            self._picam.close()  # type: ignore[attr-defined]
        except Exception:
            pass
        self._picam = None

    # ── state ──

    def status(self) -> dict:
        return {
            "active": self.running,
            "available": self.available,
            "fps_target": self.fps,
            "fps_actual": round(self._fps_actual, 1),
            "w": self.resolution[0],
            "h": self.resolution[1],
            "exposure_us": self.exposure_us,
            "frames": self._frame_n,
            "import_error": _PICAMERA2_IMPORT_ERROR,
        }

    def update_controls(self, *, exposure_us: Optional[int] = None,
                         fps: Optional[int] = None) -> None:
        """Adjust exposure / fps at runtime without tearing the stream down."""
        if exposure_us is not None:
            self.exposure_us = int(exposure_us)
        if fps is not None:
            self.fps = int(fps)
        if self._picam is None:
            return
        try:
            frame_us = max(1000, int(1_000_000 / max(1, self.fps)))
            self._picam.set_controls({  # type: ignore[attr-defined]
                "ExposureTime": int(self.exposure_us),
                "FrameDurationLimits": (frame_us, frame_us),
            })
        except Exception as e:
            log.warning("update_controls failed: %s", e)

    # ── pull loop ──

    async def _loop(self) -> None:
        loop = asyncio.get_running_loop()
        # capture_array is blocking — push to the default executor.
        while not self._stop.is_set():
            try:
                arr = await loop.run_in_executor(None, self._grab)
            except Exception as e:
                log.warning("capture_array failed: %s", e)
                # Brief backoff before retrying — sensor may have stalled.
                await asyncio.sleep(0.25)
                continue
            if arr is None:
                continue

            now = time.time()
            if self._last_frame_ts > 0:
                dt = now - self._last_frame_ts
                if dt > 0:
                    # EMA on fps for a stable readout.
                    inst = 1.0 / dt
                    self._fps_actual = (
                        inst if self._fps_actual == 0 else 0.8 * self._fps_actual + 0.2 * inst
                    )
            self._last_frame_ts = now

            meta = {
                "ts": now,
                "frame_n": self._frame_n,
                "exposure_us": self.exposure_us,
                "preview_w": self.resolution[0],
                "preview_h": self.resolution[1],
                "source": "live",
            }
            self._frame_n += 1

            try:
                r = self.sink(arr, meta)
                if asyncio.iscoroutine(r):
                    await r
            except Exception:
                log.exception("sink raised")

    def _grab(self) -> Optional[np.ndarray]:
        if self._picam is None:
            return None
        try:
            return self._picam.capture_array("main")  # type: ignore[attr-defined]
        except Exception:
            return None
