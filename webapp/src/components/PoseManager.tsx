import { useState } from "react";
import type { StarTrackerClient } from "../ble";
import type { PoseEntry, Status } from "../protocol";

export function PoseManager({
  client, poses, status,
}: { client: StarTrackerClient; poses: PoseEntry[]; status: Status | null }) {
  const [name, setName] = useState("tracking");

  return (
    <div className="card">
      <h2>Poses</h2>
      <p className="muted" style={{ marginTop: 0 }}>
        Save servo positions so you can lock motors 2 &amp; 3 to a known configuration
        during tracking. Wrist (motor 4) keeps doing altitude.
      </p>

      <div className="row">
        <input
          value={name}
          onChange={(e) => setName(e.target.value)}
          placeholder="pose name"
          style={{ flex: 1 }}
        />
        <button
          onClick={() => client.send({ cmd: "record_pose", name })}
          disabled={!name || status?.state === "slewing" || status?.state === "tracking"}
        >
          Record
        </button>
      </div>

      <hr className="hr" />

      {poses.length === 0 ? (
        <div className="muted faint">No saved poses.</div>
      ) : (
        <ul className="pose-list">
          {poses.map((p) => (
            <li key={p.name}>
              <div>
                <div>
                  {p.name}
                  {status?.locked_pose === p.name && (
                    <span className="chip chip-tracking" style={{ marginLeft: 8 }}>locked</span>
                  )}
                </div>
                <div className="pose-meta">
                  {p.heading != null && `H=${p.heading.toFixed(1)}° `}
                  {p.pitch != null && `P=${p.pitch.toFixed(1)}° `}
                  {p.timestamp && <span className="faint"> · {p.timestamp.slice(0, 19)}</span>}
                </div>
              </div>
              <div className="pose-actions">
                <button onClick={() => client.send({ cmd: "set_pose", name: p.name })}>Lock</button>
                <button className="danger" onClick={() => client.send({ cmd: "delete_pose", name: p.name })}>×</button>
              </div>
            </li>
          ))}
        </ul>
      )}
    </div>
  );
}
