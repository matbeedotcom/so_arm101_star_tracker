#!/usr/bin/env python3
"""goto_ble.py — Run the star tracker as a BLE GATT peripheral.

Headless / Wi-Fi-less Pi entry point. Bring up the radio, initialise
hardware, and accept commands from a Web Bluetooth client (Chrome/Edge).

    sudo python3 goto_ble.py
    sudo python3 goto_ble.py --name StarTracker-A --log DEBUG
    sudo python3 goto_ble.py --auto-media --open-media    # LAN trust mode

Run as root (or with the right capabilities) — BlueZ requires elevated
privileges to advertise. See README for systemd unit setup.
"""

import argparse
import asyncio
import os

from goto.ble_server import serve
from goto.ble_protocol import DEVICE_NAME


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--name", default=DEVICE_NAME, help="BLE advertised name")
    p.add_argument("--log", default="INFO",
                   choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    p.add_argument(
        "--auto-media", action="store_true",
        help="Start the WebSocket/HTTP image server at boot — without "
             "waiting for a BLE client to send enable_media. Useful when "
             "you want to connect to the live stream directly by IP.",
    )
    p.add_argument(
        "--open-media", action="store_true",
        help="Run the image server with no token (LAN trust mode). The "
             "WebSocket accepts any client. Only use this on a network "
             "you control.",
    )
    args = p.parse_args()

    # Session reads these out of the environment at construction time
    # (BLE server creates the session inside the asyncio coroutine).
    os.environ["STAR_TRACKER_AUTO_MEDIA"] = "1" if args.auto_media else "0"
    os.environ["STAR_TRACKER_OPEN_MEDIA"] = "1" if args.open_media else "0"

    try:
        asyncio.run(serve(name=args.name, log_level=args.log))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
