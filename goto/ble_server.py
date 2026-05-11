"""BLE GATT peripheral that exposes a TrackerSession over Bluetooth LE.

On the Pi this runs as the *server*: the React web client connects via
Web Bluetooth, writes JSON commands, and subscribes to notifications for
telemetry/log/poses.

Library: bless (cross-platform Python BLE peripheral wrapping BlueZ on
Linux via D-Bus). Install: ``pip install bless``. On Raspberry Pi OS,
make sure BlueZ >= 5.55 and bluetoothd has the experimental flag if
needed (typically not on recent images).
"""

from __future__ import annotations

import asyncio
import json
import logging
import os
import signal
import sys
from collections import deque
from typing import Any

from bless import (  # type: ignore
    BlessServer,
    BlessGATTCharacteristic,
    GATTCharacteristicProperties,
    GATTAttributePermissions,
)

from .ble_protocol import (
    SERVICE_UUID, CHAR_COMMAND, CHAR_STATUS, CHAR_INFO, CHAR_POSES, CHAR_LOG,
    CHAR_PREVIEW, DEVICE_NAME, PROTOCOL_VERSION,
)
from .media import MediaBroadcaster
from .session import TrackerSession, Command

log = logging.getLogger("ble_server")


# Characteristic property flags (combined as needed)
_WRITE = (
    GATTCharacteristicProperties.write
    | GATTCharacteristicProperties.write_without_response
)
_READ_NOTIFY = (
    GATTCharacteristicProperties.read | GATTCharacteristicProperties.notify
)
_NOTIFY_ONLY = GATTCharacteristicProperties.notify
_READ_ONLY = GATTCharacteristicProperties.read

_PERM_RW = GATTAttributePermissions.readable | GATTAttributePermissions.writeable
_PERM_R = GATTAttributePermissions.readable
_PERM_W = GATTAttributePermissions.writeable


class BLEServer:
    """Owns the bless server and the session it exposes."""

    def __init__(self, session: TrackerSession, name: str = DEVICE_NAME):
        self.session = session
        self.server = BlessServer(name=name)
        self._log_buf: "deque[str]" = deque(maxlen=200)
        self._last_status_bytes: bytes = b""
        self._last_poses_bytes: bytes = b""
        self._loop: asyncio.AbstractEventLoop | None = None
        self._stop = asyncio.Event()
        # BLE preview char — keep a tiny JPEG ready even before media is
        # enabled, so the client always gets *some* alignment feedback.
        self._preview_throttle = 0.0
        self._preview_min_interval = 0.5    # ≤2 Hz
        # Hook session log lines so they fan out over BLE
        session._log_sink = self._on_session_log

    # ── lifecycle ──

    async def start(self) -> None:
        self._loop = asyncio.get_running_loop()
        self.session.attach_loop(self._loop)
        self.server.read_request_func = self._on_read
        self.server.write_request_func = self._on_write

        # Start the broadcaster *now* so thumbnails flow even before
        # media-over-WS is enabled — that's the whole point of the BLE
        # fallback path.
        if self.session.media is None:
            self.session.media = MediaBroadcaster(
                capture_dir=self.session.config.capture_dir
            )
            await self.session.media.start()
            self.session.media.on_thumbnail(self._on_thumbnail)

        # Session's _initialize_hw may have already run on the worker
        # thread before media existed; opportunistically start the
        # picamera2 preview now that the broadcaster is wired.
        if not self.session.config.capture and self.session.cam is None:
            try:
                await self.session._ensure_live_camera()
            except Exception:
                log.exception("ensure_live_camera failed")

        # --auto-media: bring the image server up at boot so clients can
        # connect by IP without needing a BLE round-trip first.
        if self.session.auto_media:
            try:
                from .session import Command as _Cmd
                await self.session._do_enable_media(_Cmd(cmd="enable_media"))
            except Exception:
                log.exception("auto-media start failed")

        await self.server.add_new_service(SERVICE_UUID)

        await self.server.add_new_characteristic(
            SERVICE_UUID, CHAR_COMMAND, _WRITE, None, _PERM_W,
        )
        await self.server.add_new_characteristic(
            SERVICE_UUID, CHAR_STATUS, _READ_NOTIFY, b"{}", _PERM_R,
        )
        await self.server.add_new_characteristic(
            SERVICE_UUID, CHAR_INFO, _READ_ONLY,
            json.dumps(self.session.info()).encode(), _PERM_R,
        )
        await self.server.add_new_characteristic(
            SERVICE_UUID, CHAR_POSES, _READ_NOTIFY,
            json.dumps({"v": PROTOCOL_VERSION, "poses": self.session.list_poses()}).encode(),
            _PERM_R,
        )
        await self.server.add_new_characteristic(
            SERVICE_UUID, CHAR_LOG, _NOTIFY_ONLY, b"", _PERM_R,
        )
        await self.server.add_new_characteristic(
            SERVICE_UUID, CHAR_PREVIEW, _NOTIFY_ONLY, b"", _PERM_R,
        )

        await self.server.start()
        log.info("BLE advertising as %s", self.server.name)

        # Run the telemetry / pose loops concurrently
        await asyncio.gather(
            self._telemetry_loop(),
            self._pose_loop(),
            self._wait_stop(),
        )

    async def _wait_stop(self) -> None:
        await self._stop.wait()
        log.info("Stopping BLE server")
        await self.server.stop()

    def stop(self) -> None:
        if self._loop is not None:
            self._loop.call_soon_threadsafe(self._stop.set)

    # ── read / write callbacks (bless invokes from its own loop) ──

    def _on_read(self, characteristic: BlessGATTCharacteristic, **kw) -> bytes:
        # Reads serve whatever value is currently latched on the characteristic.
        # We keep them up to date in the telemetry/pose loops.
        return characteristic.value or b""

    def _on_write(self, characteristic: BlessGATTCharacteristic, value: bytes, **kw) -> Any:
        if characteristic.uuid.lower() != CHAR_COMMAND.lower():
            log.warning("write to unexpected char %s", characteristic.uuid)
            return
        try:
            doc = json.loads(value.decode("utf-8"))
        except Exception as e:
            self._enqueue_log(f"E parse error: {e}")
            return
        if not isinstance(doc, dict) or "cmd" not in doc:
            self._enqueue_log("E malformed command")
            return
        cmd = Command(
            cmd=str(doc.pop("cmd")),
            req=doc.pop("req", None),
            args=doc,
        )
        # "stop" is special: signal the worker immediately instead of queueing.
        if cmd.cmd == "stop":
            self.session.request_stop_current()
        self.session.submit(cmd)

        # Some commands change the pose list — schedule a refresh.
        if cmd.cmd in ("record_pose", "delete_pose", "refresh_poses"):
            if self._loop is not None:
                self._loop.call_soon_threadsafe(asyncio.create_task, self._push_poses())

    # ── telemetry / log fan-out ──

    async def _telemetry_loop(self) -> None:
        while not self._stop.is_set():
            try:
                snap = self.session.snapshot()
                payload = json.dumps(snap, separators=(",", ":")).encode()
                if payload != self._last_status_bytes:
                    self._set_value(CHAR_STATUS, payload)
                    await self._notify(CHAR_STATUS)
                    self._last_status_bytes = payload
                # Drain log buffer
                while self._log_buf:
                    line = self._log_buf.popleft()
                    self._set_value(CHAR_LOG, line.encode()[:240])
                    await self._notify(CHAR_LOG)
            except Exception as e:
                log.exception("telemetry loop error: %s", e)
            await asyncio.sleep(0.5)

    async def _pose_loop(self) -> None:
        while not self._stop.is_set():
            await asyncio.sleep(5.0)
            await self._push_poses()

    async def _push_poses(self) -> None:
        try:
            payload = json.dumps(
                {"v": PROTOCOL_VERSION, "poses": self.session.list_poses()},
                separators=(",", ":"),
            ).encode()
            if payload != self._last_poses_bytes:
                self._set_value(CHAR_POSES, payload)
                await self._notify(CHAR_POSES)
                self._last_poses_bytes = payload
        except Exception as e:
            log.exception("pose push error: %s", e)

    # ── helpers ──

    def _on_session_log(self, line: str) -> None:
        # Called from the worker thread; just enqueue.
        self._enqueue_log(line if line.startswith(("I ", "W ", "E ", "D ")) else f"I {line}")

    def _enqueue_log(self, line: str) -> None:
        self._log_buf.append(line[:240])

    async def _on_thumbnail(self, jpeg: bytes, _meta: dict) -> None:
        # Throttle, and drop oversize thumbnails to stay inside one MTU.
        loop = asyncio.get_event_loop()
        now = loop.time()
        if now - self._preview_throttle < self._preview_min_interval:
            return
        if len(jpeg) > 220:
            return  # MediaBroadcaster should keep us under, but guard anyway
        self._preview_throttle = now
        self._set_value(CHAR_PREVIEW, jpeg)
        await self._notify(CHAR_PREVIEW)

    def _set_value(self, uuid: str, value: bytes) -> None:
        char = self.server.get_characteristic(uuid)
        if char is None:
            return
        char.value = value

    async def _notify(self, uuid: str) -> None:
        try:
            await self.server.update_value(SERVICE_UUID, uuid)
        except Exception as e:
            # Updating when no client is subscribed is harmless on most stacks
            log.debug("notify %s skipped: %s", uuid, e)


# ── module entry ──

async def serve(name: str = DEVICE_NAME, log_level: str = "INFO") -> None:
    logging.basicConfig(
        level=getattr(logging, log_level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)-5s %(name)s: %(message)s",
    )
    session = TrackerSession()
    server = BLEServer(session, name=name)

    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, server.stop)
        except NotImplementedError:
            # Windows doesn't support add_signal_handler — ignore.
            pass

    try:
        await server.start()
    finally:
        session.shutdown()
