import { useState } from "react";
import type { StarTrackerClient } from "../ble";
import type { Status } from "../protocol";

function randomPassphrase(): string {
  // 10-char URL-safe — random enough for short-lived hotspots.
  const a = new Uint8Array(8);
  crypto.getRandomValues(a);
  return "st-" + Array.from(a, (b) => b.toString(36).padStart(2, "0")).join("").slice(0, 10);
}

export function NetworkPanel({
  client, status,
}: { client: StarTrackerClient; status: Status | null }) {
  const [ssid, setSsid] = useState("StarTracker");
  const [passphrase, setPassphrase] = useState(() => randomPassphrase());
  const [confirming, setConfirming] = useState(false);

  const ap = status?.ap;
  const apActive = !!ap?.active;

  async function start() {
    if (passphrase.length < 8) return;
    await client.send({ cmd: "start_ap", ssid, passphrase });
    setConfirming(true);
  }
  async function stop() {
    await client.send({ cmd: "stop_ap" });
    setConfirming(false);
  }

  return (
    <div className="card">
      <h2>Network</h2>

      <div className="muted" style={{ marginTop: 0, marginBottom: 10, fontSize: 13 }}>
        Bulk image streaming runs over Wi-Fi. Use an existing network if available;
        otherwise start a hotspot on the Pi and join it from your phone.
      </div>

      <div className="col">
        <div className="label">Reachable addresses</div>
        {status?.net?.length ? (
          <ul className="net-list">
            {status.net.map((n) => (
              <li key={`${n.name}-${n.ip}`}>
                <span className={`chip chip-${n.type === "ap" ? "tracking" : "idle"}`}>{n.type}</span>
                <span className="mono">{n.ip}</span>
                <span className="faint mono">{n.name}</span>
              </li>
            ))}
          </ul>
        ) : (
          <div className="faint">No addresses detected.</div>
        )}
      </div>

      <hr className="hr" />

      {apActive ? (
        <div className="col">
          <div className="row">
            <span className="chip chip-tracking">hotspot active</span>
            <span className="muted">on {ap?.iface ?? "?"}</span>
            <span className="muted">· {ap?.client_count ?? 0} client(s)</span>
          </div>
          <div className="mono" style={{ fontSize: 13 }}>
            SSID <strong>{ap?.ssid}</strong>
            <br />
            Pass <strong>{ap?.passphrase}</strong>
          </div>
          {confirming && (
            <div className="faint" style={{ fontSize: 12 }}>
              Join this SSID on the phone you're holding, then the live stream
              will connect automatically. Cellular data may pause while joined.
            </div>
          )}
          <div className="row">
            <button className="danger" onClick={stop}>Stop hotspot</button>
          </div>
        </div>
      ) : (
        <div className="col">
          <div className="row">
            <div style={{ flex: 1 }}>
              <label>SSID</label>
              <input value={ssid} onChange={(e) => setSsid(e.target.value)} style={{ width: "100%" }} />
            </div>
            <div style={{ flex: 1 }}>
              <label>Passphrase (≥8 chars)</label>
              <input
                value={passphrase}
                onChange={(e) => setPassphrase(e.target.value)}
                style={{ width: "100%" }}
              />
            </div>
            <div style={{ alignSelf: "flex-end" }}>
              <button onClick={() => setPassphrase(randomPassphrase())}>↺</button>
            </div>
          </div>
          <div className="row">
            <button
              className="primary"
              onClick={start}
              disabled={passphrase.length < 8 || !ssid}
            >
              Start hotspot
            </button>
          </div>
        </div>
      )}
    </div>
  );
}
