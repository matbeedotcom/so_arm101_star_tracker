import { useState } from "react";
import type { StarTrackerClient } from "../ble";
import type { Status } from "../protocol";

type Mode = "named" | "radec" | "altaz";

export function TargetSelector({
  client, stars, planets, status,
}: {
  client: StarTrackerClient;
  stars: string[];
  planets: string[];
  status: Status | null;
}) {
  const [mode, setMode] = useState<Mode>("named");
  const [target, setTarget] = useState<string>("polaris");
  const [ra, setRa] = useState("5h55m10s");
  const [dec, setDec] = useState("+7d24m25s");
  const [alt, setAlt] = useState("45");
  const [az, setAz] = useState("180");
  const [busy, setBusy] = useState(false);

  const slewing = status?.state === "slewing" || status?.state === "tracking";

  async function send(payload: Record<string, unknown>) {
    setBusy(true);
    try { await client.send({ cmd: "goto", ...payload } as any); }
    finally { setBusy(false); }
  }
  async function go() {
    if (mode === "named") return send({ target });
    if (mode === "radec") return send({ ra, dec });
    return send({ alt: parseFloat(alt), az: parseFloat(az) });
  }

  return (
    <div className="card">
      <h2>Target</h2>

      <div className="row" style={{ marginBottom: 10 }}>
        <ModeBtn current={mode} value="named" setMode={setMode}>Named</ModeBtn>
        <ModeBtn current={mode} value="radec" setMode={setMode}>RA / Dec</ModeBtn>
        <ModeBtn current={mode} value="altaz" setMode={setMode}>Alt / Az</ModeBtn>
      </div>

      {mode === "named" && (
        <div className="col">
          <select value={target} onChange={(e) => setTarget(e.target.value)} style={{ width: "100%" }}>
            <optgroup label="Solar system">
              {planets.map((p) => <option key={p} value={p}>{p}</option>)}
            </optgroup>
            <optgroup label="Stars">
              {stars.map((s) => <option key={s} value={s}>{s}</option>)}
            </optgroup>
          </select>
        </div>
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

      <hr className="hr" />

      <div className="row">
        <button className="primary" onClick={go} disabled={busy} style={{ flex: 1 }}>
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
