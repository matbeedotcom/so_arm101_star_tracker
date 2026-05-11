# Star Tracker Remote (Web Bluetooth client)

React + Vite + TypeScript front-end for the Pi-side BLE peripheral
(`goto_ble.py`). Talks to the tracker over Web Bluetooth — no server,
no Wi-Fi required.

## Browser support

Web Bluetooth is supported by **Chrome, Edge, and Opera** on desktop and
Android. It is *not* supported by Safari or iOS browsers. A secure
context is required: `http://localhost` works for development; for LAN
use, serve over HTTPS (Caddy, Tailscale Funnel, ngrok…).

## Develop

```bash
cd webapp
npm install
npm run dev
```

Open <http://localhost:5173>, click **Connect**, pick `StarTracker`.

## Build

```bash
npm run build
# static bundle in webapp/dist/ — host anywhere over HTTPS
```

## UUIDs / protocol

See [`src/protocol.ts`](src/protocol.ts) — must stay in sync with
[`../goto/ble_protocol.py`](../goto/ble_protocol.py).
