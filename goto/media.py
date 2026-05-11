"""MediaBroadcaster — watches the capture directory and fans out frames.

The speckle pipeline writes bursts to ``speckle_captures/<burst-id>/``
without any awareness of the BLE server or web client. To keep speckle
decoupled, this module polls the directory, picks the newest frame, and
publishes two derived artefacts:

  * **thumbnail** — 96×96 JPEG @ Q40, ≤2 KB. Goes out over BLE.
  * **preview**   — 640×480 JPEG @ Q70. Goes out over WebSocket.

Subscribers register callbacks via ``on_thumbnail`` / ``on_frame``.
Callbacks are invoked on the asyncio loop the broadcaster was started
on, so handlers don't need their own thread bridge.
"""

from __future__ import annotations

import asyncio
import io
import logging
import os
import time
from typing import Awaitable, Callable, Optional

log = logging.getLogger("media")

# PIL is already pulled in via requirements (pillow) for the speckle stack.
try:
    from PIL import Image  # type: ignore
    _HAVE_PIL = True
except Exception:
    _HAVE_PIL = False


FrameMeta = dict
ThumbnailCb = Callable[[bytes, FrameMeta], Awaitable[None] | None]
FrameCb = Callable[[bytes, FrameMeta], Awaitable[None] | None]


# File extensions we know how to thumbnail. DNG/RAW would need rawpy —
# skip for now and rely on the JPEG sidecars that speckle writes.
_IMG_EXTS = (".jpg", ".jpeg", ".png")


class MediaBroadcaster:
    def __init__(
        self,
        capture_dir: str = "speckle_captures",
        thumb_size: int = 96,
        thumb_quality: int = 40,
        preview_size: tuple[int, int] = (640, 480),
        preview_quality: int = 70,
        poll_interval: float = 1.0,
    ):
        self.capture_dir = capture_dir
        self.thumb_size = thumb_size
        self.thumb_quality = thumb_quality
        self.preview_size = preview_size
        self.preview_quality = preview_quality
        self.poll_interval = poll_interval

        self._last_mtime: float = 0.0
        self._last_path: Optional[str] = None
        self._thumb_listeners: list[ThumbnailCb] = []
        self._frame_listeners: list[FrameCb] = []
        self._task: Optional[asyncio.Task] = None
        self._stop = asyncio.Event()

        # Cached so reads (e.g. fresh WS connections) don't have to
        # re-scan disk.
        self.latest_thumbnail: Optional[bytes] = None
        self.latest_preview: Optional[bytes] = None
        self.latest_meta: FrameMeta = {}

        # Thumbnail derivation is the most expensive op (resize + reencode).
        # Cap it to ~2 Hz; BLE notifications can't go faster anyway.
        self._thumb_min_interval = 0.5
        self._last_thumb_ts = 0.0

    # ── lifecycle ──

    async def start(self) -> None:
        if not _HAVE_PIL:
            log.warning("PIL not available — MediaBroadcaster disabled")
            return
        if self._task is not None:
            return
        self._stop.clear()
        self._task = asyncio.create_task(self._run(), name="media-broadcaster")
        log.info("MediaBroadcaster watching %s", self.capture_dir)

    async def stop(self) -> None:
        self._stop.set()
        if self._task is not None:
            try:
                await asyncio.wait_for(self._task, timeout=2.0)
            except asyncio.TimeoutError:
                self._task.cancel()
            self._task = None

    # ── subscriptions ──

    def on_thumbnail(self, cb: ThumbnailCb) -> Callable[[], None]:
        self._thumb_listeners.append(cb)
        return lambda: self._thumb_listeners.remove(cb)

    def on_frame(self, cb: FrameCb) -> Callable[[], None]:
        self._frame_listeners.append(cb)
        return lambda: self._frame_listeners.remove(cb)

    # ── manual injection (for non-disk sources, e.g. live picamera) ──

    async def publish_raw(self, image_bytes: bytes, meta: FrameMeta) -> None:
        """Generate derivatives from an in-memory encoded image."""
        if not _HAVE_PIL:
            return
        try:
            thumb, preview = self._derive(image_bytes)
        except Exception as e:
            log.warning("derive failed: %s", e)
            return
        await self._fan_out(thumb, preview, meta)

    async def publish_array(self, rgb, meta: FrameMeta) -> None:
        """Publish a live picamera2 frame (RGB ndarray, shape H×W×3).

        The preview JPEG is regenerated every call; the thumbnail is
        rate-limited because the BLE side can't fire faster than ~2 Hz
        regardless, and resizing + reencoding is the hottest op.
        """
        if not _HAVE_PIL:
            return
        loop = asyncio.get_running_loop()
        now = time.time()
        want_thumb = (now - self._last_thumb_ts) >= self._thumb_min_interval

        try:
            thumb, preview = await loop.run_in_executor(
                None, self._derive_array, rgb, want_thumb,
            )
        except Exception as e:
            log.warning("derive_array failed: %s", e)
            return

        if want_thumb and thumb is not None:
            self._last_thumb_ts = now

        await self._fan_out(thumb, preview, meta)

    # ── internals ──

    async def _run(self) -> None:
        while not self._stop.is_set():
            try:
                await self._scan_once()
            except Exception as e:
                log.exception("scan error: %s", e)
            try:
                await asyncio.wait_for(self._stop.wait(), timeout=self.poll_interval)
            except asyncio.TimeoutError:
                pass

    async def _scan_once(self) -> None:
        path = self._latest_image_in_dir(self.capture_dir)
        if path is None:
            return
        try:
            mtime = os.path.getmtime(path)
        except OSError:
            return
        if path == self._last_path and mtime <= self._last_mtime:
            return

        loop = asyncio.get_running_loop()
        try:
            with open(path, "rb") as f:
                data = f.read()
            # Image decode + reencode is CPU work — push to a thread.
            thumb, preview = await loop.run_in_executor(None, self._derive, data)
        except Exception as e:
            log.warning("read/derive failed for %s: %s", path, e)
            return

        self._last_path = path
        self._last_mtime = mtime
        meta = {
            "path": path,
            "ts": mtime,
            "size": len(data),
            "preview_w": self.preview_size[0],
            "preview_h": self.preview_size[1],
        }
        await self._fan_out(thumb, preview, meta)

    def _latest_image_in_dir(self, root: str) -> Optional[str]:
        if not os.path.isdir(root):
            return None
        best: Optional[tuple[float, str]] = None
        # Limit walk depth — bursts are at root/<burst-id>/<frame>.jpg
        for entry in os.scandir(root):
            if entry.is_dir():
                for f in os.scandir(entry.path):
                    if not f.is_file():
                        continue
                    name = f.name.lower()
                    if not name.endswith(_IMG_EXTS):
                        continue
                    try:
                        m = f.stat().st_mtime
                    except OSError:
                        continue
                    if best is None or m > best[0]:
                        best = (m, f.path)
            elif entry.is_file() and entry.name.lower().endswith(_IMG_EXTS):
                try:
                    m = entry.stat().st_mtime
                except OSError:
                    continue
                if best is None or m > best[0]:
                    best = (m, entry.path)
        return best[1] if best else None

    def _derive_array(self, rgb, want_thumb: bool) -> tuple[Optional[bytes], bytes]:
        """Encode a live RGB array to (optional thumb, preview JPEG)."""
        # PIL handles uint8 RGB arrays directly via fromarray.
        img = Image.fromarray(rgb)

        w, h = img.size
        # Same wide-aspect handling as the disk path.
        if w > 2 * h:
            left = (w - h) // 2
            img = img.crop((left, 0, left + h, h))

        # Preview (always)
        p = img.copy()
        p.thumbnail(self.preview_size)
        if p.mode not in ("L", "RGB"):
            p = p.convert("RGB")
        preview_buf = io.BytesIO()
        p.save(preview_buf, format="JPEG", quality=self.preview_quality, optimize=False)
        preview = preview_buf.getvalue()

        thumb = None
        if want_thumb:
            t = img.copy()
            t.thumbnail((self.thumb_size, self.thumb_size))
            if t.mode != "L":
                t = t.convert("L")
            thumb_buf = io.BytesIO()
            t.save(thumb_buf, format="JPEG", quality=self.thumb_quality, optimize=True)
            thumb = thumb_buf.getvalue()

        return thumb, preview

    def _derive(self, image_bytes: bytes) -> tuple[bytes, bytes]:
        """Return (thumb_jpeg, preview_jpeg). Pure CPU; runs in executor."""
        img = Image.open(io.BytesIO(image_bytes))
        img.load()

        # Convert wide aspect (quad camera) to a sane preview frame.
        # If width > 2× height, crop to centre square first to avoid a
        # tiny postage-stamp thumbnail.
        w, h = img.size
        if w > 2 * h:
            left = (w - h) // 2
            img = img.crop((left, 0, left + h, h))

        # Thumbnail (BLE) — grayscale to save bytes.
        t = img.copy()
        t.thumbnail((self.thumb_size, self.thumb_size))
        if t.mode != "L":
            t = t.convert("L")
        thumb_buf = io.BytesIO()
        t.save(thumb_buf, format="JPEG", quality=self.thumb_quality, optimize=True)
        thumb = thumb_buf.getvalue()

        # Preview (WebSocket) — color, larger.
        p = img.copy()
        p.thumbnail(self.preview_size)
        if p.mode == "I;16":
            # 16-bit grayscale — scale to 8-bit for JPEG.
            import numpy as np
            arr = np.asarray(p)
            arr = (arr / max(1, arr.max()) * 255).astype("uint8")
            p = Image.fromarray(arr, mode="L")
        elif p.mode not in ("L", "RGB"):
            p = p.convert("RGB")
        preview_buf = io.BytesIO()
        p.save(preview_buf, format="JPEG", quality=self.preview_quality, optimize=True)
        preview = preview_buf.getvalue()

        return thumb, preview

    async def _fan_out(self, thumb: Optional[bytes], preview: bytes, meta: FrameMeta) -> None:
        if thumb is not None:
            self.latest_thumbnail = thumb
        self.latest_preview = preview
        self.latest_meta = meta

        # Best-effort: a slow listener shouldn't block the others.
        async def _call(fn, *args):
            try:
                r = fn(*args)
                if asyncio.iscoroutine(r):
                    await r
            except Exception:
                log.exception("listener error")

        tasks = [_call(fn, preview, meta) for fn in list(self._frame_listeners)]
        if thumb is not None:
            tasks.extend(_call(fn, thumb, meta) for fn in list(self._thumb_listeners))
        if tasks:
            await asyncio.gather(*tasks)
