import { useState } from "react";
import type { StarTrackerClient } from "../ble";
import type { Status } from "../protocol";
import { SkyMap, type SkySelection } from "./SkyMap";
import { targetAngularSize } from "../sky";

type Mode = "sky" | "radec" | "altaz";

export function TargetSelector({
  client, stars, planets, status,
}: {
  client: StarTrackerClient;
  stars: string[];
  planets: string[];
  status: Status | null;
}) {
  const [mode, setMode] = useState<Mode>("sky");
  const [selection, setSelection] = useState<SkySelection | null>(null);
  const [ra, setRa] = useState("5h55m10s");
  const [dec, setDec] = useState("+7d24m25s");
  const [alt, setAlt] = useState("45");
  const [az, setAz] = useState("180");
  const [busy, setBusy] = useState(false);

  const slewing = status?.state === "slewing" || status?.state === "tracking";
  const observer = status?.observer ?? { lat: 43.65, lon: -79.38 };

  async function send(payload: Record<string, unknown>) {
    setBusy(true);
    try { await client.send({ cmd: "goto", ...payload } as any); }
    finally { setBusy(false); }
  }
  async function go() {
    if (mode === "sky") {
      if (!selection) return;
      if (selection.kind === "body") return send({ target: selection.name });
      return send({ alt: selection.alt, az: selection.az });
    }
    if (mode === "radec") return send({ ra, dec });
    return send({ alt: parseFloat(alt), az: parseFloat(az) });
  }

  const canGo = (() => {
    if (busy) return false;
    if (mode === "sky") return selection != null;
    if (mode === "radec") return ra.trim().length > 0 && dec.trim().length > 0;
    return !Number.isNaN(parseFloat(alt)) && !Number.isNaN(parseFloat(az));
  })();

  return (
    <div className="card">
      <h2>Target</h2>

      <div className="row" style={{ marginBottom: 10 }}>
        <ModeBtn current={mode} value="sky"   setMode={setMode}>Sky map</ModeBtn>
        <ModeBtn current={mode} value="radec" setMode={setMode}>RA / Dec</ModeBtn>
        <ModeBtn current={mode} value="altaz" setMode={setMode}>Alt / Az</ModeBtn>
      </div>

      {mode === "sky" && (
        <SkyMap
          observerLat={observer.lat}
          observerLon={observer.lon}
          stars={stars}
          planets={planets}
          currentAlt={status?.imu_pitch ?? null}
          currentAz={status?.imu_heading ?? null}
          targetAlt={status?.target_alt ?? null}
          targetAz={status?.target_az ?? null}
          selected={selection}
          onSelect={setSelection}
        />
      )}

      {mode === "radec" && (
        <div className="col">
          <div className="row">
            <div style={{ flex: 1 }}>
              <label>Right Ascension</label>
              <input value={ra} onChange={(e) => setRa(e.target.value)} placeholder='5h55m10s' style={{ width: "100%" }} />
            </div>
            <div style={{ flex: 1 }}>
              <label>Declination</label>
              <input value={dec} onChange={(e) => setDec(e.target.value)} placeholder='+7d24m25s' style={{ width: "100%" }} />
            </div>
          </div>
        </div>
      )}

      {mode === "altaz" && (
        <div className="col">
          <div className="row">
            <div style={{ flex: 1 }}>
              <label>Altitude (°)</label>
              <input value={alt} type="number" min={0} max={90} onChange={(e) => setAlt(e.target.value)} style={{ width: "100%" }} />
            </div>
            <div style={{ flex: 1 }}>
              <label>Azimuth (°)</label>
              <input value={az} type="number" min={0} max={360} onChange={(e) => setAz(e.target.value)} style={{ width: "100%" }} />
            </div>
          </div>
        </div>
      )}

      <PrecisionHint mode={mode} selection={selection} status={status} />

      <hr className="hr" />

      <div className="row">
        <button className="primary" onClick={go} disabled={!canGo} style={{ flex: 1 }}>
          {slewing ? "Re-target" : "Go"}
        </button>
        <button className="danger" onClick={() => client.send({ cmd: "stop" })} disabled={!slewing}>
          Stop
        </button>
        <button onClick={() => client.send({ cmd: "park" })} disabled={busy || slewing}>
          Park
        </button>
      </div>
    </div>
  );
}

function PrecisionHint({
  mode, selection, status,
}: {
  mode: Mode;
  selection: SkySelection | null;
  status: Status | null;
}) {
  // Only known named targets have angular-size data; raw RA/Dec or
  // alt/az inputs don't.
  const targetName =
    mode === "sky" && selection && selection.kind === "body" ? selection.name : null;
  if (!targetName) return null;

  const size = targetAngularSize(targetName);
  if (size == null) return null;

  const hfov = status?.camera?.hfov_deg ?? null;

  // Required precision heuristic: half the target's angular diameter
  // for extended objects, or ~1% of HFOV for point sources (something
  // the user can realistically achieve with our IMU). Floor at 0.02°
  // — finer than that is meaningless on most setups.
  const required = Math.max(
    size > 0 ? size / 2 : (hfov ? hfov / 100 : 0.5),
    0.02
  );

  const frameFrac = hfov && hfov > 0 ? (size / hfov) * 100 : null;
  const isPoint = size === 0;

  // Current pointing error (if available) compared to required precision.
  const azErr = status?.az_err;
  const altErr = status?.alt_err;
  const totalErr =
    azErr != null && altErr != null
      ? Math.sqrt(azErr * azErr + altErr * altErr)
      : null;
  let tone = "faint";
  if (totalErr != null) {
    if (totalErr <= required) tone = "err-ok";
    else if (hfov && totalErr <= hfov / 4) tone = "err-warn";
    else tone = "err-bad";
  }

  return (
    <div className="row faint" style={{ fontSize: 12, marginTop: 6 }}>
      <span>
        <strong>{targetName}</strong>{" "}
        {isPoint
          ? "(point source)"
          : `~${size.toFixed(size < 0.01 ? 4 : 3)}° wide`}
        {frameFrac != null && !isPoint && ` · ${frameFrac.toFixed(2)}% of frame`}
      </span>
      <span className={tone} style={{ marginLeft: "auto" }}>
        needs ~{required.toFixed(required < 0.1 ? 3 : 2)}° pointing
        {totalErr != null && ` · now ${totalErr.toFixed(2)}°`}
      </span>
    </div>
  );
}

function ModeBtn({
  value, current, setMode, children,
}: { value: Mode; current: Mode; setMode: (m: Mode) => void; children: React.ReactNode }) {
  const active = value === current;
  return (
    <button
      onClick={() => setMode(value)}
      style={{
        background: active ? "var(--accent)" : "var(--bg-elev-2)",
        color: active ? "#2a1c00" : "var(--text)",
        borderColor: active ? "var(--accent)" : "var(--border)",
        fontWeight: active ? 600 : 400,
      }}
    >
      {children}
    </button>
  );
}
