import type { Status } from "../protocol";

const fmt = (v: number | null | undefined, units = "°", places = 1) =>
  v == null || Number.isNaN(v) ? "—" : `${v.toFixed(places)}${units}`;

function errClass(v: number | null): string {
  if (v == null) return "";
  const a = Math.abs(v);
  if (a < 0.5) return "err-ok";
  if (a < 3.0) return "err-warn";
  return "err-bad";
}

// Classify pointing error relative to the camera FOV. The thresholds
// are fractions of the smaller (vertical) dimension since framing
// "comfortably" means staying away from the frame edges in both axes.
function fovClass(err: number | null, vfov: number | null | undefined): string {
  if (err == null || !vfov || vfov <= 0) return "";
  const half = vfov / 2;
  if (err < half * 0.1)  return "err-ok";    // within 5% of frame center
  if (err < half * 0.5)  return "err-warn";  // still in frame, off-center
  return "err-bad";                            // off-frame
}

function fovLabel(err: number | null, vfov: number | null | undefined): string {
  if (err == null || !vfov || vfov <= 0) return "—";
  const half = vfov / 2;
  const pct = (err / vfov) * 100;
  let tag = "off-frame";
  if (err < half * 0.1)  tag = "centered";
  else if (err < half * 0.5) tag = "in frame";
  return `${pct.toFixed(1)}% of FOV · ${tag}`;
}

export function TelemetryPanel({ status }: { status: Status | null }) {
  if (!status) {
    return (
      <div className="card">
        <h2>Telemetry</h2>
        <div className="muted">No data yet.</div>
      </div>
    );
  }
  const totalErr = status.az_err != null && status.alt_err != null
    ? Math.sqrt(status.az_err ** 2 + status.alt_err ** 2)
    : null;

  return (
    <div className="card">
      <h2>
        Telemetry
        <span style={{ float: "right" }}>
          <span className={`chip chip-${status.state}`}>{status.state}</span>
          {status.target && <span className="mono muted" style={{ marginLeft: 8 }}>→ {status.target}</span>}
        </span>
      </h2>

      <div className="telemetry">
        <div>
          <div className="label">Target Alt / Az</div>
          <div className="value">
            {fmt(status.target_alt)} <span className="faint">/</span> {fmt(status.target_az)}
          </div>
        </div>
        <div>
          <div className="label">IMU Heading / Pitch</div>
          <div className="value">
            {fmt(status.imu_heading)} <span className="faint">/</span> {fmt(status.imu_pitch)}
          </div>
        </div>
        <div>
          <div className="label">Az Error</div>
          <div className={`value ${errClass(status.az_err)}`}>
            {status.az_err == null ? "—" : `${status.az_err >= 0 ? "+" : ""}${status.az_err.toFixed(2)}°`}
          </div>
        </div>
        <div>
          <div className="label">Alt Error</div>
          <div className={`value ${errClass(status.alt_err)}`}>
            {status.alt_err == null ? "—" : `${status.alt_err >= 0 ? "+" : ""}${status.alt_err.toFixed(2)}°`}
          </div>
        </div>
        <div>
          <div className="label">Total Error</div>
          <div className={`value ${errClass(totalErr)}`}>{fmt(totalErr, "°", 2)}</div>
        </div>
        <div>
          <div className="label">Framing</div>
          <div className={`value small ${fovClass(totalErr, status.camera?.vfov_deg)}`}>
            {fovLabel(totalErr, status.camera?.vfov_deg)}
          </div>
        </div>
        <div>
          <div className="label">Calib / Mode</div>
          <div className="value small">
            {status.calib || "—"} <span className="faint">· {status.mode}</span>
          </div>
        </div>
      </div>

      <hr className="hr" />

      <div className="row mono faint" style={{ fontSize: 12 }}>
        <span>obs {status.observer.lat.toFixed(3)}°, {status.observer.lon.toFixed(3)}°</span>
        <span style={{ marginLeft: 12 }}>
          hw: imu{status.hw.imu ? "✓" : "✗"} servos{status.hw.servos ? "✓" : "✗"} cam{status.hw.camera ? "✓" : "✗"}
        </span>
        {status.locked_pose && <span style={{ marginLeft: 12 }}>locked: {status.locked_pose}</span>}
        <span style={{ marginLeft: "auto" }}>up {status.uptime.toFixed(0)}s</span>
      </div>
      {status.error && <div className="row" style={{ color: "var(--err)", fontSize: 12 }}>error: {status.error}</div>}
    </div>
  );
}
