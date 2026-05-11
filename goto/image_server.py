"""Lightweight aiohttp server: WebSocket live preview + burst browsing.

Wire format on ``/live``:

  Each frame is a JSON text frame followed by one binary frame.

      → text  {"t":1715380000.123, "n":42, "exp":10000, "w":640, "h":480,
               "mime":"image/jpeg"}
      → bin   <JPEG bytes>

  Heartbeats (text):

      → text  {"hb":1715380005.0}

Auth: clients pass ``?token=<token>`` on the WebSocket URL. The session
hands out a fresh token whenever it enables media; sharing the token is
implicit consent from whoever has the BLE connection.
"""

from __future__ import annotations

import asyncio
import json
import logging
import os
import secrets
from typing import Optional

from aiohttp import WSMsgType, web

from .media import MediaBroadcaster

log = logging.getLogger("image_server")


class ImageServer:
    def __init__(
        self,
        broadcaster: MediaBroadcaster,
        port: int = 8765,
        bursts_dir: str = "speckle_captures",
    ):
        self.broadcaster = broadcaster
        self.port = port
        self.bursts_dir = bursts_dir
        self.token: Optional[str] = None

        self._app: Optional[web.Application] = None
        self._runner: Optional[web.AppRunner] = None
        self._site: Optional[web.TCPSite] = None
        self._clients: set[web.WebSocketResponse] = set()
        self._unsubscribe = None
        self._hb_task: Optional[asyncio.Task] = None

    @property
    def running(self) -> bool:
        return self._site is not None

    # ── lifecycle ──

    async def start(self) -> str:
        if self.running:
            return self.token or ""

        self.token = secrets.token_urlsafe(12)

        app = web.Application()
        app.router.add_get("/health", self._health)
        app.router.add_get("/live", self._ws)
        app.router.add_get("/bursts", self._list_bursts)
        app.router.add_get("/bursts/{burst}/{name}", self._burst_file)

        self._app = app
        self._runner = web.AppRunner(app)
        await self._runner.setup()
        self._site = web.TCPSite(self._runner, host="0.0.0.0", port=self.port)
        await self._site.start()

        # Tap the broadcaster.
        self._unsubscribe = self.broadcaster.on_frame(self._on_frame)
        self._hb_task = asyncio.create_task(self._heartbeat(), name="ws-heartbeat")

        log.info("image server listening on 0.0.0.0:%d (token=%s…)",
                 self.port, self.token[:4])
        return self.token

    async def stop(self) -> None:
        if not self.running:
            return
        if self._unsubscribe is not None:
            try: self._unsubscribe()
            except Exception: pass
            self._unsubscribe = None
        if self._hb_task is not None:
            self._hb_task.cancel()
            try: await self._hb_task
            except (asyncio.CancelledError, Exception): pass
            self._hb_task = None
        # Close every live socket so clients see a clean shutdown.
        for ws in list(self._clients):
            try: await ws.close(code=1001, message=b"server stopping")
            except Exception: pass
        self._clients.clear()
        if self._site is not None:
            await self._site.stop(); self._site = None
        if self._runner is not None:
            await self._runner.cleanup(); self._runner = None
        self._app = None
        self.token = None

    # ── handlers ──

    async def _health(self, _req: web.Request) -> web.Response:
        return web.json_response({"ok": True, "clients": len(self._clients)})

    async def _ws(self, req: web.Request) -> web.WebSocketResponse:
        if self.token and req.query.get("token") != self.token:
            return web.Response(status=401, text="bad token")

        ws = web.WebSocketResponse(heartbeat=20)
        await ws.prepare(req)
        self._clients.add(ws)
        log.info("ws client connected (%d total)", len(self._clients))

        # Replay the latest frame immediately so the UI doesn't sit blank.
        if self.broadcaster.latest_preview is not None:
            try:
                await self._send_frame(ws, self.broadcaster.latest_preview,
                                       self.broadcaster.latest_meta)
            except Exception:
                pass

        try:
            async for msg in ws:
                # Clients can send small JSON messages (e.g. {"req":"resync"}).
                if msg.type == WSMsgType.TEXT:
                    try:
                        doc = json.loads(msg.data)
                    except Exception:
                        continue
                    if doc.get("req") == "resync" and self.broadcaster.latest_preview:
                        await self._send_frame(ws, self.broadcaster.latest_preview,
                                               self.broadcaster.latest_meta)
                elif msg.type == WSMsgType.ERROR:
                    break
        finally:
            self._clients.discard(ws)
            log.info("ws client disconnected (%d remaining)", len(self._clients))
        return ws

    async def _list_bursts(self, _req: web.Request) -> web.Response:
        if not os.path.isdir(self.bursts_dir):
            return web.json_response({"bursts": []})
        items = []
        for entry in os.scandir(self.bursts_dir):
            if not entry.is_dir():
                continue
            try:
                files = [f.name for f in os.scandir(entry.path) if f.is_file()]
            except OSError:
                continue
            items.append({
                "id": entry.name,
                "mtime": entry.stat().st_mtime,
                "files": files,
            })
        items.sort(key=lambda x: x["mtime"], reverse=True)
        return web.json_response({"bursts": items})

    async def _burst_file(self, req: web.Request) -> web.StreamResponse:
        burst = req.match_info["burst"]
        name = req.match_info["name"]
        # Path containment check — refuse anything with traversal tokens.
        if any(part in (burst, name) for part in ("..", "/", "\\")):
            return web.Response(status=400, text="bad path")
        path = os.path.join(self.bursts_dir, burst, name)
        if not os.path.isfile(path):
            return web.Response(status=404)
        return web.FileResponse(path)

    # ── frame fan-out ──

    async def _on_frame(self, jpeg: bytes, meta: dict) -> None:
        if not self._clients:
            return
        dead = []
        for ws in list(self._clients):
            try:
                await self._send_frame(ws, jpeg, meta)
            except Exception:
                dead.append(ws)
        for ws in dead:
            self._clients.discard(ws)

    async def _send_frame(self, ws: web.WebSocketResponse, jpeg: bytes,
                           meta: dict) -> None:
        header = {
            "t": meta.get("ts", 0.0),
            "n": meta.get("frame_n", 0),
            "exp": meta.get("exposure_us"),
            "w": meta.get("preview_w"),
            "h": meta.get("preview_h"),
            "mime": "image/jpeg",
            "size": len(jpeg),
        }
        await ws.send_str(json.dumps(header, separators=(",", ":")))
        await ws.send_bytes(jpeg)

    async def _heartbeat(self) -> None:
        try:
            import time
            while True:
                await asyncio.sleep(20)
                if not self._clients:
                    continue
                msg = json.dumps({"hb": time.time()})
                for ws in list(self._clients):
                    try: await ws.send_str(msg)
                    except Exception: self._clients.discard(ws)
        except asyncio.CancelledError:
            return
