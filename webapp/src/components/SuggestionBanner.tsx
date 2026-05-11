import { useState } from "react";
import type { StarTrackerClient } from "../ble";
import type { Status, Suggestion } from "../protocol";

function describeSpec(spec: Record<string, unknown>): string {
  if (typeof spec.target === "string") return spec.target;
  if (spec.ra && spec.dec) return `RA ${spec.ra} / Dec ${spec.dec}`;
  if (spec.alt != null && spec.az != null) return `alt ${spec.alt}° / az ${spec.az}°`;
  return JSON.stringify(spec);
}

function formatWhen(iso: string): { local: string; relative: string } {
  const d = new Date(iso);
  const local = d.toLocaleString(undefined, {
    weekday: "short", month: "short", day: "numeric",
    hour: "2-digit", minute: "2-digit",
  });
  const minutes = Math.round((d.getTime() - Date.now()) / 60000);
  let rel: string;
  if (minutes < 60) rel = `in ${minutes} min`;
  else if (minutes < 60 * 24) rel = `in ${Math.round(minutes / 60)} h`;
  else rel = `in ${Math.round(minutes / 1440)} d`;
  return { local, relative: rel };
}

export function SuggestionBanner({
  client, status,
}: { client: StarTrackerClient; status: Status | null }) {
  const s: Suggestion | null = status?.suggestion ?? null;
  const [busy, setBusy] = useState(false);
  if (!s) return null;

  const target = describeSpec(s.spec);
  const out = s.action === "out_of_range";
  const visible = !out && s.next_visible;

  async function scheduleAt(iso: string) {
    setBusy(true);
    try {
      await client.send({
        cmd: "schedule",
        at: iso,
        ...(s!.spec as any),
      });
    } finally { setBusy(false); }
  }
  async function dismiss() {
    setBusy(true);
    try { await client.send({ cmd: "dismiss_suggestion" }); }
    finally { setBusy(false); }
  }

  return (
    <div className={`suggestion ${out ? "suggestion-out" : ""}`}>
      <div className="suggestion-icon" aria-hidden>⏳</div>
      <div className="suggestion-body">
        <div className="suggestion-title">
          <strong>{target}</strong> is {s.reason}.
        </div>
        {visible ? (
          <div className="suggestion-text">
            Next clears 10° {formatWhen(s.next_visible!).relative}
            {" — "}
            <span className="mono">{formatWhen(s.next_visible!).local}</span>
            {s.alt_at_time != null && (
              <span className="faint"> · alt {s.alt_at_time.toFixed(1)}°</span>
            )}
          </div>
        ) : (
          <div className="suggestion-text">
            Not visible from this site in the next 48&nbsp;hours.
          </div>
        )}
      </div>
      <div className="suggestion-actions">
        {visible && (
          <button
            type="button"
            className="primary"
            disabled={busy}
            onClick={() => scheduleAt(s.next_visible!)}
          >
            Schedule
          </button>
        )}
        <button type="button" disabled={busy} onClick={dismiss}>Dismiss</button>
      </div>
    </div>
  );
}
