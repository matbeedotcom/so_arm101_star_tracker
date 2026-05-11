import { useState } from "react";
import type { StarTrackerClient } from "../ble";
import type { Status } from "../protocol";

export function ConnectionPanel({
  client, connected, status,
}: { client: StarTrackerClient; connected: boolean; status: Status | null }) {
  const [busy, setBusy] = useState(false);
  const [err, setErr] = useState<string | null>(null);

  async function onConnect() {
    setErr(null);
    setBusy(true);
    try {
      await client.connect();
    } catch (e) {
      setErr((e as Error).message);
    } finally {
      setBusy(false);
    }
  }
  async function onDisconnect() {
    setBusy(true);
    try { await client.disconnect(); } finally { setBusy(false); }
  }

  return (
    <div style={{ display: "flex", gap: 10, alignItems: "center" }}>
      {connected && status && (
        <span className={`chip chip-${status.state}`}>{status.state}</span>
      )}
      {connected ? (
        <button onClick={onDisconnect} disabled={busy}>Disconnect</button>
      ) : (
        <button className="primary" onClick={onConnect} disabled={busy}>
          {busy ? "Connecting…" : "Connect"}
        </button>
      )}
      {err && <span className="faint" style={{ fontSize: 12 }}>{err}</span>}
    </div>
  );
}
