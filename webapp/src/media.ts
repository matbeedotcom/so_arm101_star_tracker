// WebSocket client for the Pi's image_server. Pairs with goto/image_server.py.
//
// Frame protocol on /live: a JSON text frame (header) followed by one
// binary frame (JPEG bytes). Heartbeats are JSON {"hb": ts}. The class
// here is intentionally stateless beyond the connection — listeners
// get each (Blob, header) pair, and they decide what to do with it.

import type { LiveFrameHeader } from "./protocol";

export type MediaState = "idle" | "connecting" | "live" | "reconnecting" | "error";

export interface MediaListeners {
  onState?(s: MediaState, info?: { error?: string; url?: string }): void;
  onFrame?(blob: Blob, header: LiveFrameHeader): void;
  onHeartbeat?(ts: number): void;
}

export class MediaClient {
  private ws: WebSocket | null = null;
  private url: string | null = null;
  private pendingHeader: LiveFrameHeader | null = null;
  private listeners: MediaListeners = {};
  private reconnectTimer: number | null = null;
  private explicitClose = false;
  state: MediaState = "idle";

  setListeners(l: MediaListeners) { this.listeners = l; }

  /** Build a ws:// URL from a status snapshot. */
  static urlFor(ip: string, port: number, path: string, token: string | null): string {
    const proto = location.protocol === "https:" ? "wss:" : "ws:";
    const q = token ? `?token=${encodeURIComponent(token)}` : "";
    return `${proto}//${ip}:${port}${path}${q}`;
  }

  async connect(url: string): Promise<void> {
    this.disconnect();
    this.explicitClose = false;
    this.url = url;
    this._setState("connecting", { url });

    try {
      const ws = new WebSocket(url);
      ws.binaryType = "arraybuffer";
      this.ws = ws;

      ws.onopen = () => this._setState("live", { url });
      ws.onmessage = (ev) => this._onMessage(ev);
      ws.onerror = () => {
        if (this.state !== "reconnecting") {
          this._setState("error", { error: "websocket error", url });
        }
      };
      ws.onclose = () => {
        this.ws = null;
        if (this.explicitClose) {
          this._setState("idle");
        } else {
          this._scheduleReconnect();
        }
      };
    } catch (e) {
      this._setState("error", { error: (e as Error).message, url });
    }
  }

  /** Ask the server to resend its latest frame (useful after a stall). */
  resync(): void {
    if (this.ws?.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify({ req: "resync" }));
    }
  }

  disconnect(): void {
    this.explicitClose = true;
    if (this.reconnectTimer !== null) {
      clearTimeout(this.reconnectTimer);
      this.reconnectTimer = null;
    }
    if (this.ws) {
      try { this.ws.close(); } catch { /* ignore */ }
      this.ws = null;
    }
    this.pendingHeader = null;
    if (this.state !== "idle") this._setState("idle");
  }

  // ── internals ──

  private _onMessage(ev: MessageEvent): void {
    if (typeof ev.data === "string") {
      // Text frame — JSON header or heartbeat.
      let doc: any;
      try { doc = JSON.parse(ev.data); }
      catch { return; }
      if (typeof doc.hb === "number") {
        this.listeners.onHeartbeat?.(doc.hb);
        return;
      }
      this.pendingHeader = doc as LiveFrameHeader;
    } else {
      // Binary frame — paired with the most recent header.
      if (!this.pendingHeader) return;
      const buf: ArrayBuffer = ev.data instanceof ArrayBuffer
        ? ev.data
        : (ev.data as Uint8Array).buffer.slice(0);
      const blob = new Blob([buf], { type: this.pendingHeader.mime || "image/jpeg" });
      this.listeners.onFrame?.(blob, this.pendingHeader);
      this.pendingHeader = null;
    }
  }

  private _scheduleReconnect(): void {
    if (!this.url || this.explicitClose) return;
    this._setState("reconnecting", { url: this.url });
    this.reconnectTimer = window.setTimeout(() => {
      this.reconnectTimer = null;
      if (this.url) this.connect(this.url);
    }, 1500);
  }

  private _setState(s: MediaState, info?: { error?: string; url?: string }): void {
    this.state = s;
    this.listeners.onState?.(s, info);
  }
}
