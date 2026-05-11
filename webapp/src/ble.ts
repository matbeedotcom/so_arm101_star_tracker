// Web Bluetooth client wrapper for the star tracker.
//
// All BLE plumbing (connect, characteristic lookup, notify subscription,
// encode/decode, reconnect) lives here so React components can stay
// declarative.

import {
  SERVICE_UUID,
  CHAR_COMMAND,
  CHAR_STATUS,
  CHAR_INFO,
  CHAR_POSES,
  CHAR_LOG,
  CHAR_PREVIEW,
  DEVICE_NAME,
  type Command,
  type Info,
  type PosesPayload,
  type Status,
} from "./protocol";

type Listener<T> = (value: T) => void;

const td = new TextDecoder();
const te = new TextEncoder();

export class StarTrackerClient {
  device: BluetoothDevice | null = null;
  server: BluetoothRemoteGATTServer | null = null;
  service: BluetoothRemoteGATTService | null = null;
  chars: {
    command?: BluetoothRemoteGATTCharacteristic;
    status?: BluetoothRemoteGATTCharacteristic;
    info?: BluetoothRemoteGATTCharacteristic;
    poses?: BluetoothRemoteGATTCharacteristic;
    log?: BluetoothRemoteGATTCharacteristic;
    preview?: BluetoothRemoteGATTCharacteristic;
  } = {};

  // Request ids for command correlation
  private _nextReq = 1;
  private _connectionListeners = new Set<Listener<boolean>>();
  private _statusListeners = new Set<Listener<Status>>();
  private _posesListeners = new Set<Listener<PosesPayload>>();
  private _logListeners = new Set<Listener<string>>();
  private _previewListeners = new Set<Listener<Blob>>();

  get connected(): boolean {
    return !!this.server?.connected;
  }

  // ── Connection ──

  async connect(): Promise<void> {
    if (!("bluetooth" in navigator)) {
      throw new Error("Web Bluetooth not available (use Chrome/Edge over HTTPS or localhost)");
    }
    this.device = await navigator.bluetooth.requestDevice({
      filters: [{ services: [SERVICE_UUID] }, { namePrefix: DEVICE_NAME }],
      optionalServices: [SERVICE_UUID],
    });
    this.device.addEventListener("gattserverdisconnected", () => {
      this._fireConnection(false);
    });
    await this._openGatt();
  }

  async disconnect(): Promise<void> {
    if (this.device?.gatt?.connected) {
      this.device.gatt.disconnect();
    }
    this._fireConnection(false);
  }

  private async _openGatt(): Promise<void> {
    if (!this.device?.gatt) throw new Error("No GATT server on selected device");
    this.server = await this.device.gatt.connect();
    this.service = await this.server.getPrimaryService(SERVICE_UUID);

    this.chars.command = await this.service.getCharacteristic(CHAR_COMMAND);
    this.chars.status  = await this.service.getCharacteristic(CHAR_STATUS);
    this.chars.info    = await this.service.getCharacteristic(CHAR_INFO);
    this.chars.poses   = await this.service.getCharacteristic(CHAR_POSES);
    this.chars.log     = await this.service.getCharacteristic(CHAR_LOG);
    // CHAR_PREVIEW is optional — older firmware won't have it.
    try { this.chars.preview = await this.service.getCharacteristic(CHAR_PREVIEW); }
    catch { /* preview unsupported */ }

    this.chars.status.addEventListener("characteristicvaluechanged", (e) => {
      const v = (e.target as BluetoothRemoteGATTCharacteristic).value;
      if (!v) return;
      try {
        const s = JSON.parse(td.decode(v)) as Status;
        this._statusListeners.forEach((fn) => fn(s));
      } catch {/* malformed — skip */}
    });
    this.chars.poses.addEventListener("characteristicvaluechanged", (e) => {
      const v = (e.target as BluetoothRemoteGATTCharacteristic).value;
      if (!v) return;
      try {
        const p = JSON.parse(td.decode(v)) as PosesPayload;
        this._posesListeners.forEach((fn) => fn(p));
      } catch {/* skip */}
    });
    this.chars.log.addEventListener("characteristicvaluechanged", (e) => {
      const v = (e.target as BluetoothRemoteGATTCharacteristic).value;
      if (!v) return;
      const line = td.decode(v);
      this._logListeners.forEach((fn) => fn(line));
    });

    await this.chars.status.startNotifications();
    await this.chars.poses.startNotifications();
    await this.chars.log.startNotifications();

    if (this.chars.preview) {
      this.chars.preview.addEventListener("characteristicvaluechanged", (e) => {
        const v = (e.target as BluetoothRemoteGATTCharacteristic).value;
        if (!v || v.byteLength === 0) return;
        const blob = new Blob([v.buffer], { type: "image/jpeg" });
        this._previewListeners.forEach((fn) => fn(blob));
      });
      try { await this.chars.preview.startNotifications(); } catch { /* skip */ }
    }

    // Prime listeners with whatever the characteristics already hold.
    await this._readInitial();

    this._fireConnection(true);
  }

  private async _readInitial(): Promise<void> {
    try {
      const v = await this.chars.status?.readValue();
      if (v) {
        const s = JSON.parse(td.decode(v)) as Status;
        this._statusListeners.forEach((fn) => fn(s));
      }
    } catch {/* status may be empty on first connect */}
    try {
      const v = await this.chars.poses?.readValue();
      if (v) {
        const p = JSON.parse(td.decode(v)) as PosesPayload;
        this._posesListeners.forEach((fn) => fn(p));
      }
    } catch {/* skip */}
  }

  // ── Read-once ──

  async readInfo(): Promise<Info> {
    const v = await this.chars.info?.readValue();
    if (!v) throw new Error("info characteristic unavailable");
    return JSON.parse(td.decode(v)) as Info;
  }

  // ── Commands ──

  async send(cmd: Command): Promise<number> {
    if (!this.chars.command) throw new Error("Not connected");
    const req = (cmd as { req?: number }).req ?? this._nextReq++;
    const payload = JSON.stringify({ ...cmd, req });
    // writeWithoutResponse is faster but we want backpressure so use writeValue
    await this.chars.command.writeValue(te.encode(payload));
    return req;
  }

  // ── Subscriptions ──

  onConnectionChange(fn: Listener<boolean>): () => void {
    this._connectionListeners.add(fn);
    return () => this._connectionListeners.delete(fn);
  }
  onStatus(fn: Listener<Status>): () => void {
    this._statusListeners.add(fn);
    return () => this._statusListeners.delete(fn);
  }
  onPoses(fn: Listener<PosesPayload>): () => void {
    this._posesListeners.add(fn);
    return () => this._posesListeners.delete(fn);
  }
  onLog(fn: Listener<string>): () => void {
    this._logListeners.add(fn);
    return () => this._logListeners.delete(fn);
  }
  onPreview(fn: Listener<Blob>): () => void {
    this._previewListeners.add(fn);
    return () => this._previewListeners.delete(fn);
  }
  get hasPreview(): boolean {
    return !!this.chars.preview;
  }

  private _fireConnection(state: boolean): void {
    this._connectionListeners.forEach((fn) => fn(state));
  }
}
