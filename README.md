# SO-100 Star Tracker

Point a SO-100 robot arm at any celestial object and track it in real
time. No ROS, no planner — direct I²C to a BNO055 IMU and serial to
Feetech servos. Targets are resolved through Astropy, so anything from
the built-in catalog (planets + bright stars) to arbitrary RA/Dec works.

Includes a Bluetooth LE peripheral (`goto_ble.py`) and a React Web
Bluetooth client (`webapp/`) for headless / Wi-Fi-less operation, plus
an optional speckle-interferometry burst pipeline for the Arducam quad
camera.

## How it works

```
       celestial target            BNO055 (heading, pitch)
              │                            │
              ▼                            ▼
         astropy ─────► alt/az ──► error ──► motion strategy
                                                  │
                                                  ▼
                                       Feetech servos (Sh-rot, wrist
                                       pitch) + base wheels (azimuth)
```

The wrist-pitch motor handles altitude, the shoulder rotation (with
base-wheel fallback when near its limits) handles azimuth, and motors 2
& 3 stay locked at a recorded pose so the arm geometry is consistent
across sessions.

## Hardware

| Component        | Default                                        |
| ---------------- | ---------------------------------------------- |
| Robot arm        | SO-100 / SO-101 (5 servos + 3 omni base wheels) |
| Servo bus        | Feetech STS / SCS, `/dev/ttyACM0` @ 1 Mbps      |
| IMU              | BNO055 on I²C bus 1                            |
| Camera (optional) | Arducam Quad (picamera2)                       |
| Compute          | Raspberry Pi 4/5 (also runs fine on a laptop with a USB-serial servo bus and BNO055 USB breakout) |

Observer location, port assignments, and servo IDs live in
[`goto/config.py`](goto/config.py).

## Install (Raspberry Pi OS Bookworm)

`picamera2` and `libcamera` ship as apt packages on Pi OS and can't be
pip-installed (their C extensions are built against the system Python).
Bookworm also blocks system-wide pip per PEP 668. The clean solution is
a **venv that inherits system site-packages**: pip works inside the
venv, and the apt-installed camera stack stays visible.

```bash
# 1. apt deps — picamera2/libcamera plus things we'd rather get from
#    Debian than pip (faster wheels, signed updates):
sudo apt install -y python3-full python3-venv \
    python3-picamera2 python3-libcamera \
    python3-astropy python3-aiohttp python3-pil python3-numpy \
    python3-smbus2

# 2. venv with inheritance — required so `import picamera2` works:
cd ~/dev/star_tracker_ws        # or wherever you cloned
python3 -m venv --system-site-packages .venv
source .venv/bin/activate

# 3. pip-only deps (bless + the Feetech servo SDK):
pip install -r requirements.txt

# 4. sanity check
python3 -c "from picamera2 import Picamera2; print('camera ok')"
python3 -c "import scservo_sdk, bless, astropy; print('deps ok')"
```

Add yourself to `dialout` (servo USB) and `i2c` (IMU):

```bash
sudo usermod -aG dialout,i2c $USER
# log out / back in for it to take effect
```

> **macOS / non-Pi dev machines**: skip the apt step and the camera
> packages — `python3 -m venv .venv && pip install -r requirements.txt`
> is enough to run `goto.py` against any servo bus you wire up. The
> camera + speckle bits gracefully no-op when picamera2 isn't present.

## CLI — `goto.py`

```bash
python3 goto.py polaris                    # named star or planet
python3 goto.py moon
python3 goto.py --ra 5h55m10s --dec -7d24m25s
python3 goto.py --alt 45 --az 180          # fixed alt/az

python3 goto.py --record-pose tracking     # save current arm config
python3 goto.py --pose tracking polaris    # lock motors 2,3 to that pose
python3 goto.py --calibrate                # 3-point pitch calibration
```

Useful flags: `--mode {ndof,imu}` (toggle magnetometer), `--no-capture`,
`--exposure`, `--burst-count`, `--skip-cal`, `--lat`, `--lon`.

On startup the script sweeps through a fixed set of poses to give the
BNO055 enough motion data to converge — pass `--skip-cal` to bypass.
Ctrl-C cleanly stops the wheels and restores servo mode.

## BLE remote control

For a headless Pi (no display, no Wi-Fi needed). Phone or laptop
connects over Bluetooth LE; everything else stays the same.

### Pi side

```bash
# from the workspace directory
sudo .venv/bin/python3 goto_ble.py         # advertises as "StarTracker"
```

Or run it as a service (assumes the venv lives at `.venv/` per the
Install section — edit the unit if you cloned somewhere other than
`~/dev/star_tracker_ws`):

```bash
sudo cp systemd/star-tracker-ble.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now star-tracker-ble
journalctl -u star-tracker-ble -f
```

Requires BlueZ ≥ 5.55. Root by default so it can advertise and so
`nmcli` works for the hotspot command — drop privileges with
`CAP_NET_ADMIN`/`CAP_NET_RAW` if you want.

### Web client

Chrome or Edge on desktop / Android (Safari does not support Web
Bluetooth). Use `localhost` for dev or HTTPS over a tunnel for LAN:

```bash
cd webapp
npm install
npm run dev                                # http://localhost:5173
```

UI exposes target selection (catalog, RA/Dec, alt/az), live IMU + error
telemetry, IMU calibration, observer GPS, pose record/lock/delete, and a
streamed log. The GATT schema is in
[`goto/ble_protocol.py`](goto/ble_protocol.py); the TypeScript mirror is
[`webapp/src/protocol.ts`](webapp/src/protocol.ts).

### Image streaming

Camera frames are way too fat for BLE, so there's a three-tier path:

| When you have…           | What you get                                      |
| ------------------------ | ------------------------------------------------- |
| BLE only                 | 96×96 grayscale JPEG thumbnail @ ≤2 Hz over the `Preview` characteristic |
| Pi on shared Wi-Fi/Ethernet | Click **Enable stream** → WebSocket live preview at full resolution, auto-connect using the IP advertised in `Status.net` |
| No infrastructure         | Click **Start hotspot** in the Network panel — Pi runs `nmcli device wifi hotspot`, BLE returns SSID + passphrase, join from the phone, stream connects automatically |

The image server is part of `goto_ble.py` — same process, separate
`aiohttp` runner on port 8765. Hotspot mode requires NetworkManager
(Raspberry Pi OS Bookworm default) and root; if you're on Bullseye with
`dhcpcd`, hotspot won't work but everything else (BLE thumbnail +
WebSocket on existing LAN) does.

#### Bypassing BLE discovery

If you already know the Pi's IP — common on a fixed LAN, or when
debugging — you can skip the BLE round-trip entirely for the video
stream:

```bash
# Pi: start the image server at boot, no token required.
sudo .venv/bin/python3 goto_ble.py --auto-media --open-media
```

Then in the web client, click **Direct ▸** in the Live preview header
and enter `host:port` (token blank in open mode). Or bookmark a URL:

```
http://your-webapp/?host=192.168.18.2&port=8765
```

You still need BLE for commands (goto, calibration, scheduling). The
direct path is just for the video pipe.

- `--auto-media` — start the WebSocket server at boot instead of
  waiting for the BLE `enable_media` command
- `--open-media` — don't issue or require a token. Use only on a
  network you trust; anyone reachable on the port can view the stream

## Calibration

Two separate things, both run from the CLI:

1. **IMU calibration** — sweeps the arm through fixed poses so the
   BNO055 can settle. Happens automatically on `goto.py` startup, or
   on-demand via the BLE `calibrate_imu` command.
2. **Pitch calibration** — `python3 goto.py --calibrate` walks you
   through positioning the wrist by hand at three different pitch
   angles. Records ticks ↔ degrees, fits a line, saves it to
   `poses/pitch_cal.json`. Re-run after any mechanical change.

For polar alignment / IMU axis sanity-check, see
[`aim_polaris.py`](aim_polaris.py) and
[`calibrate_imu_axes.py`](calibrate_imu_axes.py).

## Speckle capture (optional)

If an Arducam quad camera is attached, `goto.py` and `goto_ble.py` will
init a speckle pipeline. Use `--burst-count 100` for true speckle mode,
`1` for single-shot. Captures are written under `speckle_captures/` and
processed in the background; the
[`speckle/`](speckle/) package owns capture, stability detection,
storage, and reconstruction.

## Project layout

```
goto.py                  CLI entry — slew + track + optional capture
goto_ble.py              BLE peripheral entry
goto/
  config.py              Pins, baud rates, joint limits, star catalog
  celestial.py           Target resolution + alt/az via Astropy
  imu.py                 BNO055 driver + calibration helpers
  servos.py              Feetech SDK wrapper, base-wheel mode
  strategy.py            WristOnlyStrategy: maps error → motor moves
  tracker.py             slew_to_target / track_target loops
  session.py             Thread-safe controller for remote use
  ble_protocol.py        UUIDs + JSON schemas
  ble_server.py          GATT server (bless)
  media.py               Capture-dir watcher → thumbnail + preview
  image_server.py        aiohttp WebSocket + HTTP for live streaming
  network.py             nmcli wrappers: address probe + hotspot control
webapp/                  React + Vite + TS Web Bluetooth client
speckle/                 Speckle interferometry pipeline
poses/                   Saved arm configurations (.json)
systemd/                 star-tracker-ble.service unit
```

## Credits

- [SO-ARM100/101](https://github.com/TheRobotStudio/SO-ARM100) by TheRobotStudio
- [Astropy](https://www.astropy.org/) for coordinate transforms
- [Feetech SCServo SDK](https://github.com/scservo) for the servo protocol
- [bless](https://github.com/kevincar/bless) for cross-platform BLE peripheral support

## License

See upstream SO-ARM project for license terms.
