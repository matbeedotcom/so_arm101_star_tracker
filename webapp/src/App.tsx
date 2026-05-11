import { useEffect, useMemo, useRef, useState } from "react";
import { StarTrackerClient } from "./ble";
import type { Info, PosesPayload, Status } from "./protocol";
import { ConnectionPanel } from "./components/ConnectionPanel";
import { TelemetryPanel } from "./components/TelemetryPanel";
import { TargetSelector } from "./components/TargetSelector";
import { PoseManager } from "./components/PoseManager";
import { CalibrationPanel } from "./components/CalibrationPanel";
import { ConfigPanel } from "./components/ConfigPanel";
import { LogStream } from "./components/LogStream";
import { LivePreview } from "./components/LivePreview";
import { NetworkPanel } from "./components/NetworkPanel";
import { SuggestionBanner } from "./components/SuggestionBanner";
import { SchedulePanel } from "./components/SchedulePanel";

export function App() {
  // Single client instance for the app lifetime.
  const clientRef = useRef<StarTrackerClient>();
  if (!clientRef.current) clientRef.current = new StarTrackerClient();
  const client = clientRef.current!;

  const [connected, setConnected] = useState(false);
  const [status, setStatus] = useState<Status | null>(null);
  const [info, setInfo] = useState<Info | null>(null);
  const [poses, setPoses] = useState<PosesPayload | null>(null);
  const [logLines, setLogLines] = useState<string[]>([]);

  useEffect(() => {
    const offs = [
      client.onConnectionChange(setConnected),
      client.onStatus(setStatus),
      client.onPoses(setPoses),
      client.onLog((line) => {
        setLogLines((prev) => {
          const next = [...prev, line];
          return next.length > 400 ? next.slice(-400) : next;
        });
      }),
    ];
    return () => offs.forEach((off) => off());
  }, [client]);

  // Pull Info once on connect.
  useEffect(() => {
    if (!connected) return;
    let cancelled = false;
    client.readInfo().then((i) => { if (!cancelled) setInfo(i); }).catch(() => {});
    return () => { cancelled = true; };
  }, [connected, client]);

  const targets = useMemo(() => ({
    stars: info?.stars ?? [],
    planets: info?.planets ?? [],
  }), [info]);

  return (
    <div className="app">
      <header className="app-header">
        <h1>
          <span className="accent">★</span> Star Tracker <span className="muted">Remote</span>
        </h1>
        <ConnectionPanel client={client} connected={connected} status={status} />
      </header>

      {!connected ? (
        <div className="disconnected-overlay">
          <p>Not connected.</p>
          <div className="hint">
            Click <strong>Connect</strong> to scan for a nearby StarTracker over Bluetooth LE.
            Web Bluetooth requires Chrome or Edge over HTTPS or localhost.
            Make sure <code className="mono">goto_ble.py</code> is running on the Pi.
          </div>
        </div>
      ) : (
        <>
          <TelemetryPanel status={status} />

          <SuggestionBanner client={client} status={status} />

          <div className="section">
            <LivePreview client={client} status={status} />
          </div>

          <div className="grid section">
            <TargetSelector client={client} stars={targets.stars} planets={targets.planets} status={status} />
            <SchedulePanel client={client} status={status} />
            <PoseManager client={client} poses={poses?.poses ?? []} status={status} />
            <CalibrationPanel client={client} status={status} />
            <ConfigPanel client={client} status={status} />
            <NetworkPanel client={client} status={status} />
          </div>

          <div className="section">
            <LogStream lines={logLines} onClear={() => setLogLines([])} />
          </div>
        </>
      )}
    </div>
  );
}
