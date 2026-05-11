import { useEffect, useMemo, useRef, useState } from "react";
import type { StarTrackerClient } from "../ble";
import { MediaClient, type MediaState } from "../media";
import type { LiveFrameHeader, Status } from "../protocol";

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
  const [autoConnected, setAutoConnected] = useState(false);

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

  // Auto-connect when status reports media enabled + a reachable IP.
  const candidate = useMemo(() => pickCandidate(status), [status]);
  useEffect(() => {
    if (!candidate || autoConnected) return;
    if (mediaState !== "idle" && mediaState !== "error") return;
    media.connect(candidate.url).catch(() => {});
    setAutoConnected(true);
  }, [candidate, autoConnected, mediaState, media]);

  // If media gets disabled server-side, drop the WS connection.
  useEffect(() => {
    if (status && !status.media.enabled) {
      media.disconnect();
      setAutoConnected(false);
      if (liveUrl) { URL.revokeObjectURL(liveUrl); setLiveUrl(null); }
      setLiveHeader(null);
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [status?.media.enabled]);

  // ── render ──

  const isLive = mediaState === "live" && liveUrl;
  const mediaEnabled = status?.media.enabled ?? false;
  const live = status?.live_preview;

  return (
    <div className="card">
      <h2>
        Live preview
        <span className="preview-toolbar">
          {isLive && (
            <span className="chip chip-tracking" title={candidate?.url}>
              live · {liveHeader?.w}×{liveHeader?.h}
            </span>
          )}
          {!isLive && live?.active && (
            <span className="chip chip-idle" title="picamera2 streaming via broadcaster">
              picam · {live.fps_actual.toFixed(1)} fps
            </span>
          )}
          {!isLive && !live?.active && thumbUrl && (
            <span className="chip chip-idle">BLE · thumb</span>
          )}
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
        ) : mediaEnabled && candidate ? (
          <>
            connecting to {candidate.url}
            {mediaState === "reconnecting" && " (retrying…)"}
            {mediaError && <span className="err-inline"> · {mediaError}</span>}
          </>
        ) : mediaEnabled && !candidate ? (
          <>media enabled but no reachable IP — start a hotspot or join the same network</>
        ) : live?.active ? (
          <>picam streaming · enable media stream to view full frames in this browser</>
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
  // Prefer Wi-Fi/AP over loopback; fall back to whatever's there.
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
