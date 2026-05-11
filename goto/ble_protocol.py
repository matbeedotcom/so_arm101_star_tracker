"""BLE GATT protocol for the star tracker remote.

Single source of truth for service/characteristic UUIDs and the JSON
shapes exchanged with the web client. Keep in sync with
webapp/src/protocol.ts.

Characteristic roles
====================
  Command  (Write, JSON)         — client → device, one operation per write
  Status   (Read + Notify, JSON) — live telemetry, pushed ~2 Hz
  Info     (Read, JSON)          — static catalogs (stars, planets, version)
  Poses    (Read + Notify, JSON) — list of saved arm poses; re-notified on change
  Log      (Notify, UTF-8 text)  — line-oriented event log

Each value is a single JSON document (or text line for Log) — all stay
well under a 240-byte negotiated ATT MTU in practice. If a payload grows
beyond that, prefer adding a paged characteristic over manual chunking.
"""

# Service base — random v4 UUID, last byte distinguishes characteristics.
SERVICE_UUID = "b87a5e8c-c2a1-4d8a-9f3a-c7e8b8c0d101"
CHAR_COMMAND = "b87a5e8c-c2a1-4d8a-9f3a-c7e8b8c0d102"
CHAR_STATUS  = "b87a5e8c-c2a1-4d8a-9f3a-c7e8b8c0d103"
CHAR_INFO    = "b87a5e8c-c2a1-4d8a-9f3a-c7e8b8c0d104"
CHAR_POSES   = "b87a5e8c-c2a1-4d8a-9f3a-c7e8b8c0d105"
CHAR_LOG     = "b87a5e8c-c2a1-4d8a-9f3a-c7e8b8c0d106"

DEVICE_NAME = "StarTracker"
PROTOCOL_VERSION = 1

# ── Commands (client → device, written to CHAR_COMMAND as JSON) ──
#
# Every command has a "cmd" field plus a "req" (request-id) echoed back
# on Log so the client can correlate completions. Unknown fields are
# ignored; unknown commands return an error log line.
#
#   {"cmd":"goto", "req":<int>, "target":"polaris"}
#   {"cmd":"goto", "req":<int>, "ra":"5h55m10s", "dec":"-7d24m25s"}
#   {"cmd":"goto", "req":<int>, "alt":45.0, "az":180.0}
#   {"cmd":"stop", "req":<int>}
#   {"cmd":"park", "req":<int>}                     # home pose, idle state
#   {"cmd":"set_observer", "req":<int>, "lat":45.4, "lon":-75.7}
#   {"cmd":"set_pose", "req":<int>, "name":"tracking"}  # lock M2/M3
#   {"cmd":"record_pose", "req":<int>, "name":"tracking"}
#   {"cmd":"delete_pose", "req":<int>, "name":"tracking"}
#   {"cmd":"calibrate_imu", "req":<int>}
#   {"cmd":"calibrate_pitch", "req":<int>}
#   {"cmd":"set_config", "req":<int>,
#        "mode":"ndof"|"imu",
#        "exposure":10000, "burst_count":1,
#        "capture":true|false}
#   {"cmd":"reinit_hw", "req":<int>}                # re-open IMU + servos
#   {"cmd":"refresh_poses", "req":<int>}            # force Poses notify
#
# ── Status (device → client, CHAR_STATUS, notify ~2 Hz) ──
#
#   {
#     "v": 1,                                       # PROTOCOL_VERSION
#     "state": "idle"|"slewing"|"tracking"|"calibrating"|"parking"|"error",
#     "target": "polaris"|null,
#     "target_alt": 41.2, "target_az": 0.5,
#     "imu_heading": 0.3, "imu_pitch": 41.1,
#     "az_err": 0.2, "alt_err": 0.1,
#     "calib": "S=3 G=3 A=3 M=3",                   # BNO055 calib bits
#     "observer": {"lat":45.4, "lon":-75.7},
#     "mode": "ndof"|"imu",
#     "hw": {"imu":true, "servos":true, "camera":false},
#     "capture": {"enabled":true, "burst_count":1, "frames_captured":12},
#     "uptime": 123.4,
#     "error": null
#   }
#
# ── Info (device → client, CHAR_INFO, read-only) ──
#
#   {
#     "v": 1,
#     "name": "StarTracker",
#     "version": "1.0",
#     "stars": ["polaris","sirius",...],
#     "planets": ["sun","moon","mercury",...]
#   }
#
# ── Poses (device → client, CHAR_POSES, read + notify on change) ──
#
#   {"v":1, "poses":[
#       {"name":"tracking", "timestamp":"2026-05-10T18:00:00",
#        "heading":12.3, "pitch":40.5},
#       ...
#   ]}
#
# ── Log (device → client, CHAR_LOG, notify) ──
#   UTF-8 text. Lines prefixed with severity letter:
#     "I "  info
#     "W "  warning
#     "E "  error
#     "D "  done (correlated to a req-id, e.g. "D 7 goto:ok")
