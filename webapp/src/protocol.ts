// Star tracker BLE GATT protocol — must match goto/ble_protocol.py.

export const SERVICE_UUID = "b87a5e8c-c2a1-4d8a-9f3a-c7e8b8c0d101";
export const CHAR_COMMAND = "b87a5e8c-c2a1-4d8a-9f3a-c7e8b8c0d102";
export const CHAR_STATUS  = "b87a5e8c-c2a1-4d8a-9f3a-c7e8b8c0d103";
export const CHAR_INFO    = "b87a5e8c-c2a1-4d8a-9f3a-c7e8b8c0d104";
export const CHAR_POSES   = "b87a5e8c-c2a1-4d8a-9f3a-c7e8b8c0d105";
export const CHAR_LOG     = "b87a5e8c-c2a1-4d8a-9f3a-c7e8b8c0d106";
export const CHAR_PREVIEW  = "b87a5e8c-c2a1-4d8a-9f3a-c7e8b8c0d107";
export const CHAR_NETWORK  = "b87a5e8c-c2a1-4d8a-9f3a-c7e8b8c0d108";
export const CHAR_SCHEDULE = "b87a5e8c-c2a1-4d8a-9f3a-c7e8b8c0d109";

export const DEVICE_NAME = "StarTracker";

export type TrackerState =
  | "init"
  | "idle"
  | "slewing"
  | "tracking"
  | "calibrating"
  | "parking"
  | "error";

export interface Status {
  v: number;
  state: TrackerState;
  target: string | null;
  target_alt: number | null;
  target_az: number | null;
  imu_heading: number | null;
  imu_pitch: number | null;
  az_err: number | null;
  alt_err: number | null;
  calib: string;
  phase: "slew" | "track" | null;
  corrections: number;
  captures: number;
  observer: { lat: number; lon: number };
  mode: "ndof" | "imu";
  hw: { imu: boolean; servos: boolean; camera: boolean };
  capture: { enabled: boolean; burst_count: number };
  locked_pose: string | null;
  uptime: number;
  error: string | null;
  media: { enabled: boolean; port: number; token: string | null; path: string };
  net: NetInterface[];
  ap: APState;
  live_preview: LivePreviewState;
  schedule: ScheduledJob[];
  suggestion: Suggestion | null;
}

/** Raw payload of CHAR_NETWORK. */
export interface NetworkPayload {
  v: number;
  net: NetInterface[];
  ap: APState;
}

/** Raw payload of CHAR_SCHEDULE. */
export interface SchedulePayload {
  v: number;
  schedule: ScheduledJob[];
  suggestion: Suggestion | null;
}

export interface ScheduledJob {
  id: number;
  spec: Record<string, unknown>;     // goto args (target | ra+dec | alt+az)
  at: string;                        // ISO 8601 UTC
  state: "pending" | "running" | "done" | "cancelled" | "failed";
  note: string;
  created: string;
  fired_at?: string | null;
  error?: string | null;
}

export interface Suggestion {
  action: "schedule" | "out_of_range";
  spec: Record<string, unknown>;
  reason: string;
  current_alt: number;
  current_az: number;
  next_visible: string | null;       // ISO 8601 UTC
  alt_at_time: number | null;
  minutes_from_now: number | null;
}

export interface LivePreviewState {
  active: boolean;        // pull loop running
  available: boolean;     // picamera2 importable
  fps_target: number;
  fps_actual: number;
  w: number;
  h: number;
  exposure_us: number;
  frames: number;
  import_error?: string | null;   // exception text when available=false
}

export interface NetInterface {
  name: string;
  ip: string;
  type: "wifi" | "eth" | "ap" | "lo" | "vpn" | "other";
}

export interface APState {
  active: boolean;
  ssid: string | null;
  passphrase: string | null;
  iface: string | null;
  client_count: number;
}

export interface LiveFrameHeader {
  t: number;
  n: number;
  exp?: number | null;
  w?: number;
  h?: number;
  mime: string;
  size: number;
}

export interface Info {
  v: number;
  name: string;
  version: string;
  stars: string[];
  planets: string[];
}

export interface PoseEntry {
  name: string;
  timestamp?: string | null;
  heading?: number | null;
  pitch?: number | null;
}

export interface PosesPayload {
  v: number;
  poses: PoseEntry[];
}

export type Command =
  | { cmd: "goto"; req?: number; target: string }
  | { cmd: "goto"; req?: number; ra: string; dec: string }
  | { cmd: "goto"; req?: number; alt: number; az: number }
  | { cmd: "stop"; req?: number }
  | { cmd: "park"; req?: number }
  | { cmd: "set_observer"; req?: number; lat: number; lon: number }
  | { cmd: "set_pose"; req?: number; name: string }
  | { cmd: "record_pose"; req?: number; name: string }
  | { cmd: "delete_pose"; req?: number; name: string }
  | { cmd: "calibrate_imu"; req?: number }
  | { cmd: "calibrate_pitch"; req?: number }
  | { cmd: "set_config"; req?: number; mode?: "ndof" | "imu"; exposure?: number; burst_count?: number; capture?: boolean }
  | { cmd: "reinit_hw"; req?: number }
  | { cmd: "refresh_poses"; req?: number }
  | { cmd: "enable_media"; req?: number }
  | { cmd: "disable_media"; req?: number }
  | { cmd: "start_ap"; req?: number; ssid: string; passphrase: string; iface?: string }
  | { cmd: "stop_ap"; req?: number }
  | { cmd: "live_start"; req?: number }
  | { cmd: "live_stop"; req?: number }
  | { cmd: "schedule"; req?: number; at: string; target?: string; ra?: string; dec?: string; alt?: number; az?: number; note?: string }
  | { cmd: "cancel_schedule"; req?: number; id: number }
  | { cmd: "dismiss_suggestion"; req?: number };
