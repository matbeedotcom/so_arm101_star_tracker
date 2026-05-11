import { useEffect, useState } from "react";
import type { StarTrackerClient } from "../ble";
import type { Status } from "../protocol";

export function ConfigPanel({
  client, status,
}: { client: StarTrackerClient; status: Status | null }) {
  const [lat, setLat] = useState<string>("");
  const [lon, setLon] = useState<string>("");
  const [mode, setMode] = useState<"ndof" | "imu">("ndof");
  const [exposure, setExposure] = useState<string>("10000");
  const [burst, setBurst] = useState<string>("1");
  const [capture, setCapture] = useState(false);
  const [hfov, setHfov] = useState<string>("");
  const [vfov, setVfov] = useState<string>("");

  // Initialise from status when first available.
  useEffect(() => {
    if (!status) return;
    if (lat === "") setLat(String(status.observer.lat));
    if (lon === "") setLon(String(status.observer.lon));
    setMode(status.mode);
    setCapture(status.capture.enabled);
    setBurst(String(status.capture.burst_count));
    if (hfov === "" && status.camera?.hfov_deg != null) setHfov(String(status.camera.hfov_deg));
    if (vfov === "" && status.camera?.vfov_deg != null) setVfov(String(status.camera.vfov_deg));
  }, [status]);

  function applyObserver() {
    const la = parseFloat(lat), lo = parseFloat(lon);
    if (Number.isFinite(la) && Number.isFinite(lo)) {
      client.send({ cmd: "set_observer", lat: la, lon: lo });
    }
  }
  function applyConfig() {
    const cmd: any = {
      cmd: "set_config",
      mode,
      exposure: parseInt(exposure, 10),
      burst_count: parseInt(burst, 10),
      capture,
    };
    const h = parseFloat(hfov);
    const v = parseFloat(vfov);
    if (Number.isFinite(h) && h > 0) cmd.hfov_deg = h;
    if (Number.isFinite(v) && v > 0) cmd.vfov_deg = v;
    client.send(cmd);
  }

  function useGeo() {
    if (!("geolocation" in navigator)) return;
    navigator.geolocation.getCurrentPosition((pos) => {
      setLat(pos.coords.latitude.toFixed(5));
      setLon(pos.coords.longitude.toFixed(5));
    });
  }

  return (
    <div className="card">
      <h2>Config</h2>

      <div className="row">
        <div style={{ flex: 1 }}>
          <label>Observer latitude</label>
          <input value={lat} onChange={(e) => setLat(e.target.value)} style={{ width: "100%" }} />
        </div>
        <div style={{ flex: 1 }}>
          <label>Observer longitude</label>
          <input value={lon} onChange={(e) => setLon(e.target.value)} style={{ width: "100%" }} />
        </div>
      </div>
      <div className="row" style={{ marginTop: 8 }}>
        <button onClick={useGeo}>Use browser location</button>
        <button className="primary" onClick={applyObserver}>Apply</button>
      </div>

      <hr className="hr" />

      <div className="row">
        <div style={{ flex: 1 }}>
          <label>IMU mode</label>
          <select value={mode} onChange={(e) => setMode(e.target.value as "ndof" | "imu")} style={{ width: "100%" }}>
            <option value="ndof">NDOF (full fusion)</option>
            <option value="imu">IMU (no magnetometer)</option>
          </select>
        </div>
        <div style={{ flex: 1 }}>
          <label>Exposure (µs)</label>
          <input value={exposure} type="number" onChange={(e) => setExposure(e.target.value)} style={{ width: "100%" }} />
        </div>
        <div style={{ width: 100 }}>
          <label>Burst</label>
          <input value={burst} type="number" min={1} onChange={(e) => setBurst(e.target.value)} style={{ width: "100%" }} />
        </div>
      </div>

      <div className="row" style={{ marginTop: 10 }}>
        <label style={{ display: "flex", gap: 6, alignItems: "center", margin: 0 }}>
          <input type="checkbox" checked={capture} onChange={(e) => setCapture(e.target.checked)} />
          Capture frames while tracking
        </label>
      </div>

      <hr className="hr" />

      <div className="row">
        <div style={{ flex: 1 }}>
          <label>Camera HFOV (°)</label>
          <input
            value={hfov} type="number" min={1} max={180} step="0.1"
            onChange={(e) => setHfov(e.target.value)}
            style={{ width: "100%" }}
          />
        </div>
        <div style={{ flex: 1 }}>
          <label>Camera VFOV (°)</label>
          <input
            value={vfov} type="number" min={1} max={180} step="0.1"
            onChange={(e) => setVfov(e.target.value)}
            style={{ width: "100%" }}
          />
        </div>
      </div>
      <div className="row faint" style={{ fontSize: 12, marginTop: 4 }}>
        Used to render "% of FOV" feedback and decide if a target is comfortably framed.
      </div>

      <div className="row" style={{ marginTop: 10 }}>
        <button className="primary" style={{ marginLeft: "auto" }} onClick={applyConfig}>Apply</button>
      </div>
    </div>
  );
}
