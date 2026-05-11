#!/usr/bin/env python3
"""goto_ble.py — Run the star tracker as a BLE GATT peripheral.

Headless / Wi-Fi-less Pi entry point. Bring up the radio, initialise
hardware, and accept commands from a Web Bluetooth client (Chrome/Edge).

    sudo python3 goto_ble.py
    sudo python3 goto_ble.py --name StarTracker-A --log DEBUG

Run as root (or with the right capabilities) — BlueZ requires elevated
privileges to advertise. See README for systemd unit setup.
"""

import argparse
import asyncio
import sys

from goto.ble_server import serve
from goto.ble_protocol import DEVICE_NAME


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--name", default=DEVICE_NAME, help="BLE advertised name")
    p.add_argument("--log", default="INFO",
                   choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    args = p.parse_args()

    try:
        asyncio.run(serve(name=args.name, log_level=args.log))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
