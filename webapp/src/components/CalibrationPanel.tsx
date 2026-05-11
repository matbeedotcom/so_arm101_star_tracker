import type { StarTrackerClient } from "../ble";
import type { Status } from "../protocol";

export function CalibrationPanel({
  client, status,
}: { client: StarTrackerClient; status: Status | null }) {
  const busy = status?.state === "calibrating" || status?.state === "slewing" || status?.state === "tracking";

  return (
    <div className="card">
      <h2>Calibration</h2>
      <p className="muted" style={{ marginTop: 0 }}>
        IMU calibration sweeps the arm through a fixed set of poses to give the
        BNO055 enough motion data. Run this after any major move or magnetic
        environment change.
      </p>
      <div className="row">
        <button
          onClick={() => client.send({ cmd: "calibrate_imu" })}
          disabled={busy}
        >
          Calibrate IMU
        </button>
        <button onClick={() => client.send({ cmd: "reinit_hw" })} disabled={busy}>
          Re-init hardware
        </button>
      </div>
      <hr className="hr" />
      <div className="mono faint" style={{ fontSize: 12 }}>
        {status?.calib ? `current: ${status.calib}` : "no IMU reading"}
      </div>
    </div>
  );
}
