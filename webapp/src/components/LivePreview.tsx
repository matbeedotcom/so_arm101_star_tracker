import { useEffect, useMemo, useRef, useState } from "react";
import type { StarTrackerClient } from "../ble";
import { MediaClient, type MediaState } from "../media";
import type { LiveFrameHeader, Status } from "../protocol";

/** Stored across reloads so the user doesn't have to re-enter on refresh. */
const DIRECT_LS_KEY = "star-tracker.direct-url";

/** Read host/port/token from the page URL once (returns ws:// URL or null). */
function readUrlParams(): string | null {
  if (typeof window === "undefined") return null;
  const params = new URLSearchParams(window.location.search);
  const host = params.get("host");
  if (!host) return null;
  const port = params.get("port") || "8765";
  const token = params.get("token");
  const path = params.get("path") || "/live";
  return buildWsUrl(host, port, path, token);
}

function buildWsUrl(host: string, port: string | number, path: string, token: string | null): string {
  const proto = typeof location !== "undefined" && location.protocol === "https:" ? "wss:" : "ws:";
  const q = token ? `?token=${encodeURIComponent(token)}` : "";
  return `${proto}//${host}:${port}${path}${q}`;
}

export function LivePreview({
  client, status,
}: { client: StarTrackerClient; status: Status | null }) {
  const mediaRef = useRef<MediaClient>();
  if (!mediaRef.current) mediaRef.current = new MediaClient();
  const media = mediaRef.current!;

  const [thumbUrl, setThumbUrl] = useState<string | null>(null);
  const [liveUrl, setLiveUrl] = useState<string | null>(null);
  const [liveHeader, setLiveHeader] = useState<LiveFrameHeader | null>(null);
  const [mediaState, setMediaState] = useState<MediaState>("idle");
  const [mediaError, setMediaError] = useState<string | null>(null);

  // Direct connect — overrides BLE-driven auto-pick when set.
  const [directUrl, setDirectUrl] = useState<string | null>(() => {
    return readUrlParams() ?? localStorage.getItem(DIRECT_LS_KEY);
  });
  const [showDirectForm, setShowDirectForm] = useState(false);
  const lastConnectUrl = useRef<string | null>(null);

  // BLE thumbnail subscription.
  useEffect(() => {
    const off = client.onPreview((blob) => {
      setThumbUrl((prev) => {
        if (prev) URL.revokeObjectURL(prev);
        return URL.createObjectURL(blob);
      });
    });
    return () => {
      off();
      if (thumbUrl) URL.revokeObjectURL(thumbUrl);
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [client]);

  // Media client wiring.
  useEffect(() => {
    media.setListeners({
      onState: (s, info) => {
        setMediaState(s);
        setMediaError(info?.error ?? null);
      },
      onFrame: (blob, header) => {
        setLiveUrl((prev) => {
          if (prev) URL.revokeObjectURL(prev);
          return URL.createObjectURL(blob);
        });
        setLiveHeader(header);
      },
    });
    return () => media.disconnect();
  }, [media]);

  // Decide which URL to connect to. Direct override wins; otherwise
  // fall back to whatever the BLE Status snapshot advertises.
  const candidate = useMemo(() => pickCandidate(status), [status]);
  const targetUrl = directUrl ?? candidate?.url ?? null;

  useEffect(() => {
    if (!targetUrl) return;
    if (lastConnectUrl.current === targetUrl) return;
    media.connect(targetUrl).catch(() => {});
    lastConnectUrl.current = targetUrl;
  }, [targetUrl, media]);

  // If BLE reports media disabled AND we don't have a direct override,
  // tear the WS down. Direct override is sticky — user explicitly
  // pointed us somewhere.
  useEffect(() => {
    if (!directUrl && status && !status.media.enabled) {
      media.disconnect();
      lastConnectUrl.current = null;
      if (liveUrl) { URL.revokeObjectURL(liveUrl); setLiveUrl(null); }
      setLiveHeader(null);
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [status?.media.enabled, directUrl]);

  function applyDirectUrl(url: string | null) {
    if (url) {
      localStorage.setItem(DIRECT_LS_KEY, url);
    } else {
      localStorage.removeItem(DIRECT_LS_KEY);
    }
    setDirectUrl(url);
    lastConnectUrl.current = null;     // force a fresh connect attempt
    setShowDirectForm(false);
  }

  // ── render ──

  const isLive = mediaState === "live" && liveUrl;
  const mediaEnabled = status?.media.enabled ?? false;
  const live = status?.live_preview;
  const sourceLabel = directUrl ? "direct" : "ble";

  return (
    <div className="card">
      <h2>
        Live preview
        <span className="preview-toolbar">
          {isLive && (
            <span className="chip chip-tracking" title={lastConnectUrl.current || ""}>
              live · {sourceLabel} · {liveHeader?.w}×{liveHeader?.h}
            </span>
          )}
          {!isLive && mediaState === "connecting" && (
            <span className="chip chip-init">connecting…</span>
          )}
          {!isLive && mediaState === "reconnecting" && (
            <span className="chip chip-init">reconnecting…</span>
          )}
          {!isLive && mediaState === "error" && (
            <span className="chip chip-error" title={mediaError ?? undefined}>ws error</span>
          )}
          {!isLive && mediaState === "idle" && live?.active && (
            <span className="chip chip-idle" title="picamera2 streaming via broadcaster">
              picam · {live.fps_actual.toFixed(1)} fps
            </span>
          )}
          {!isLive && mediaState === "idle" && !live?.active && thumbUrl && (
            <span className="chip chip-idle">BLE · thumb</span>
          )}

          <button
            type="button"
            className="btn-mini"
            onClick={() => setShowDirectForm((v) => !v)}
            title="Connect directly to a known WebSocket without going through BLE discovery"
          >
            {directUrl ? "Direct ✓" : "Direct"}
          </button>
          {live && !live.active && live.available && status?.hw.camera === false && (
            <button
              type="button"
              className="btn-mini"
              onClick={() => client.send({ cmd: "live_start" })}
            >
              Start picam
            </button>
          )}
          {live?.active && (
            <button
              type="button"
              className="btn-mini"
              onClick={() => client.send({ cmd: "live_stop" })}
            >
              Stop picam
            </button>
          )}
          {!mediaEnabled ? (
            <button
              type="button"
              className="btn-mini"
              onClick={() => client.send({ cmd: "enable_media" })}
            >
              Enable stream
            </button>
          ) : (
            <button
              type="button"
              className="btn-mini"
              onClick={() => client.send({ cmd: "disable_media" })}
            >
              Disable
            </button>
          )}
        </span>
      </h2>

      {showDirectForm && (
        <DirectConnectForm
          initial={directUrl}
          status={status}
          onApply={applyDirectUrl}
        />
      )}

      <div className="preview-stage">
        {isLive ? (
          <img className="preview-image" src={liveUrl!} alt="live frame" />
        ) : thumbUrl ? (
          <img className="preview-image" src={thumbUrl} alt="BLE thumbnail" />
        ) : (
          <div className="preview-empty">
            <div className="muted">No frames yet.</div>
            <div className="faint preview-empty-hint">
              {live?.available
                ? "Start picam preview, or capture a burst to see imagery."
                : "Capture a burst (or enable media stream) to see imagery."}
            </div>
            {live && !live.available && live.import_error && (
              <div className="faint preview-empty-hint err-inline mono">
                picamera2 import error: {live.import_error}
              </div>
            )}
          </div>
        )}
      </div>

      <div className="preview-meta mono faint">
        {isLive && liveHeader ? (
          <>
            frame #{liveHeader.n}
            {liveHeader.exp ? ` · ${liveHeader.exp}µs` : ""}
            {" · "}{(liveHeader.size / 1024).toFixed(1)} KB
            {" · "}{new Date(liveHeader.t * 1000).toLocaleTimeString()}
            {live?.active && <> · picam {live.w}×{live.h} @ {live.fps_actual.toFixed(1)}fps</>}
          </>
        ) : targetUrl ? (
          <>
            {mediaState} → <span className="mono">{targetUrl}</span>
            {mediaError && <span className="err-inline"> · {mediaError}</span>}
          </>
        ) : mediaEnabled && !candidate ? (
          <>media enabled but no reachable IP advertised — use Direct ▸ to enter one, or join the Pi's hotspot</>
        ) : live?.active ? (
          <>picam streaming · enable media stream + connect to view full frames</>
        ) : (
          <>BLE preview only · enable the stream for full-resolution frames</>
        )}
      </div>
    </div>
  );
}

/** Pick the best reachable URL from the status snapshot. */
function pickCandidate(status: Status | null): { url: string; ip: string } | null {
  if (!status?.media.enabled) return null;
  const { net, media } = status;
  if (!net?.length) return null;
  const order = ["wifi", "ap", "eth", "other"];
  const sorted = [...net].sort(
    (a, b) => order.indexOf(a.type) - order.indexOf(b.type),
  );
  const pick = sorted.find((i) => i.ip && i.ip !== "127.0.0.1");
  if (!pick) return null;
  return {
    ip: pick.ip,
    url: MediaClient.urlFor(pick.ip, media.port, media.path, media.token),
  };
}

function DirectConnectForm({
  initial, status, onApply,
}: {
  initial: string | null;
  status: Status | null;
  onApply: (url: string | null) => void;
}) {
  // Parse the initial URL so the form starts populated, if possible.
  const seeded = useMemo(() => parseWsUrl(initial), [initial]);

  const [host, setHost] = useState(seeded?.host
    ?? status?.net?.find(n => n.ip && n.ip !== "127.0.0.1")?.ip
    ?? "");
  const [port, setPort] = useState(seeded?.port ?? String(status?.media.port ?? 8765));
  const [token, setToken] = useState(seeded?.token ?? status?.media.token ?? "");

  return (
    <div className="direct-form">
      <div className="row">
        <div className="direct-field">
          <label>Host / IP</label>
          <input
            value={host}
            onChange={e => setHost(e.target.value)}
            placeholder="192.168.18.2"
          />
        </div>
        <div className="direct-field direct-field-port">
          <label>Port</label>
          <input
            value={port}
            onChange={e => setPort(e.target.value)}
            placeholder="8765"
            inputMode="numeric"
          />
        </div>
        <div className="direct-field">
          <label>Token <span className="faint">(blank for open mode)</span></label>
          <input
            value={token}
            onChange={e => setToken(e.target.value)}
            placeholder="(optional)"
          />
        </div>
      </div>
      <div className="row direct-actions">
        <button
          type="button"
          className="primary"
          disabled={!host || !port}
          onClick={() => onApply(buildWsUrl(host, port, "/live", token.trim() || null))}
        >
          Connect direct
        </button>
        {initial && (
          <button type="button" onClick={() => onApply(null)}>
            Clear (use BLE)
          </button>
        )}
        <div className="faint direct-hint">
          Bookmarkable: append <span className="mono">?host={host || "<ip>"}&amp;port={port || "8765"}{token ? `&token=${token}` : ""}</span> to the page URL.
        </div>
      </div>
    </div>
  );
}

function parseWsUrl(url: string | null): { host: string; port: string; token: string | null } | null {
  if (!url) return null;
  try {
    const u = new URL(url);
    return {
      host: u.hostname,
      port: u.port || "8765",
      token: u.searchParams.get("token"),
    };
  } catch { return null; }
}
