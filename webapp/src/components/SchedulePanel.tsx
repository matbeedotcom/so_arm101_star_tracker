import { useState } from "react";
import type { StarTrackerClient } from "../ble";
import type { ScheduledJob, Status } from "../protocol";

function describeSpec(spec: Record<string, unknown>): string {
  if (typeof spec.target === "string") return spec.target;
  if (spec.ra && spec.dec) return `RA ${spec.ra} · Dec ${spec.dec}`;
  if (spec.alt != null && spec.az != null) return `alt ${spec.alt}° / az ${spec.az}°`;
  return JSON.stringify(spec);
}

function whenLine(iso: string): string {
  const d = new Date(iso);
  const minutes = Math.round((d.getTime() - Date.now()) / 60000);
  const local = d.toLocaleString(undefined, {
    month: "short", day: "numeric", hour: "2-digit", minute: "2-digit",
  });
  let rel: string;
  if (minutes < 0)          rel = `${-minutes} min ago`;
  else if (minutes < 60)    rel = `in ${minutes} min`;
  else if (minutes < 1440)  rel = `in ${Math.round(minutes / 60)} h`;
  else                       rel = `in ${Math.round(minutes / 1440)} d`;
  return `${local}  ·  ${rel}`;
}

export function SchedulePanel({
  client, status,
}: { client: StarTrackerClient; status: Status | null }) {
  const jobs: ScheduledJob[] = status?.schedule ?? [];
  const [showAdd, setShowAdd] = useState(false);
  const [target, setTarget] = useState("polaris");
  const [datetime, setDatetime] = useState(defaultLocalDatetime());

  async function addJob() {
    // <input type="datetime-local"> gives a naive local-time string.
    // Convert to ISO UTC before sending.
    const local = new Date(datetime);
    if (Number.isNaN(local.getTime())) return;
    await client.send({
      cmd: "schedule",
      at: local.toISOString(),
      target,
    });
    setShowAdd(false);
  }

  return (
    <div className="card">
      <h2>
        Schedule
        <span className="card-header-right">
          <button
            type="button"
            className="btn-mini"
            onClick={() => setShowAdd(v => !v)}
          >
            {showAdd ? "Close" : "+ Add"}
          </button>
        </span>
      </h2>

      {showAdd && (
        <div className="schedule-add">
          <div className="row">
            <div className="schedule-add-field">
              <label>Target</label>
              <input
                value={target}
                onChange={e => setTarget(e.target.value)}
                placeholder="polaris"
              />
            </div>
            <div className="schedule-add-field">
              <label>When (local time)</label>
              <input
                type="datetime-local"
                value={datetime}
                onChange={e => setDatetime(e.target.value)}
              />
            </div>
            <button type="button" className="primary" onClick={addJob}>
              Schedule
            </button>
          </div>
          <hr className="hr" />
        </div>
      )}

      {jobs.length === 0 ? (
        <div className="muted faint">No scheduled jobs.</div>
      ) : (
        <ul className="schedule-list">
          {jobs.map(j => (
            <li key={j.id}>
              <div className="schedule-line">
                <span className={`chip chip-${j.state === "running" ? "tracking" : "idle"}`}>
                  {j.state}
                </span>
                <span className="mono"><strong>{describeSpec(j.spec)}</strong></span>
                <span className="faint mono">#{j.id}</span>
              </div>
              <div className="faint mono schedule-when">{whenLine(j.at)}</div>
              {j.error && <div className="err-inline">{j.error}</div>}
              {j.state === "pending" && (
                <button
                  type="button"
                  className="btn-mini danger schedule-cancel"
                  onClick={() => client.send({ cmd: "cancel_schedule", id: j.id })}
                >
                  Cancel
                </button>
              )}
            </li>
          ))}
        </ul>
      )}
    </div>
  );
}

function defaultLocalDatetime(): string {
  // <input type="datetime-local"> expects YYYY-MM-DDTHH:mm with no tz suffix
  // and interprets it as local time. Default to "now + 1 hour".
  const d = new Date(Date.now() + 3600_000);
  const pad = (n: number) => n.toString().padStart(2, "0");
  return `${d.getFullYear()}-${pad(d.getMonth() + 1)}-${pad(d.getDate())}T${pad(d.getHours())}:${pad(d.getMinutes())}`;
}
