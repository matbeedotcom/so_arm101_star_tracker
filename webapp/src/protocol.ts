// Star tracker BLE GATT protocol — must match goto/ble_protocol.py.

export const SERVICE_UUID = "b87a5e8c-c2a1-4d8a-9f3a-c7e8b8c0d101";
export const CHAR_COMMAND = "b87a5e8c-c2a1-4d8a-9f3a-c7e8b8c0d102";
export const CHAR_STATUS  = "b87a5e8c-c2a1-4d8a-9f3a-c7e8b8c0d103";
export const CHAR_INFO    = "b87a5e8c-c2a1-4d8a-9f3a-c7e8b8c0d104";
export const CHAR_POSES   = "b87a5e8c-c2a1-4d8a-9f3a-c7e8b8c0d105";
export const CHAR_LOG     = "b87a5e8c-c2a1-4d8a-9f3a-c7e8b8c0d106";

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
  | { cmd: "refresh_poses"; req?: number };
