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
CHAR_PREVIEW = "b87a5e8c-c2a1-4d8a-9f3a-c7e8b8c0d107"  # raw JPEG bytes, ≤220 B

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
#   # ── Media / network (image streaming) ──
#   {"cmd":"enable_media",  "req":<int>}            # start aiohttp WS+HTTP on :8765
#   {"cmd":"disable_media", "req":<int>}
#   {"cmd":"start_ap",      "req":<int>,
#        "ssid":"StarTracker", "passphrase":"tracker-XXXX"}
#   {"cmd":"stop_ap",       "req":<int>}
#   {"cmd":"live_start",    "req":<int>}            # picamera2 low-res stream
#   {"cmd":"live_stop",     "req":<int>}            # release camera
#
#   # ── Scheduling ──
#   # When a goto target is below the horizon the session populates
#   # ``status.suggestion`` with the next visible time. The client uses
#   # ``schedule`` to defer the goto until then.
#   {"cmd":"schedule",      "req":<int>,
#        "at":"2026-05-11T22:30:00Z",
#        # plus exactly one of the goto target forms:
#        "target":"polaris" }                        # or "ra"+"dec", or "alt"+"az"
#   {"cmd":"cancel_schedule","req":<int>, "id":<int>}
#   {"cmd":"dismiss_suggestion","req":<int>}
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
#     "error": null,
#
#     # Streaming endpoints — empty until enable_media:
#     "media": {"enabled":false, "port":8765, "token":null, "path":"/live"},
#     "net":   [{"name":"wlan0","ip":"192.168.1.42","type":"wifi"}],
#     "ap":    {"active":false, "ssid":null, "passphrase":null,
#               "iface":"wlan0", "client_count":0},
#
#     # picamera2 live preview status (set when speckle isn't using the cam):
#     "live_preview": {"active":true, "available":true,
#                      "fps_target":10, "fps_actual":9.6,
#                      "w":640, "h":480, "exposure_us":20000, "frames":1234},
#
#     # Scheduling
#     "schedule": [
#       {"id":7, "spec":{"target":"polaris"}, "at":"2026-05-11T22:30:00Z",
#        "state":"pending", "note":"", "created":"...", "fired_at":null,
#        "error":null}, ...
#     ],
#     # Set when a goto fails because the target is below the horizon —
#     # cleared on dismiss_suggestion or a successful goto/schedule.
#     "suggestion": {
#       "action":"schedule",                          # or "out_of_range"
#       "spec":{"target":"polaris"},
#       "reason":"below horizon (alt=-48.1°)",
#       "current_alt": -48.1, "current_az": 5.2,
#       "next_visible":"2026-05-11T22:30:00Z",
#       "alt_at_time": 11.0, "minutes_from_now": 743
#     }
#   }
#
# ── Preview (device → client, CHAR_PREVIEW, notify-only) ──
#   Raw JPEG bytes, ≤220 B (fits in one ATT MTU). Throttled to ≤2 Hz.
#   The session keeps this alive even with no Wi-Fi — it's the fallback
#   visual feedback when bulk media streaming isn't an option.
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
