#!/usr/bin/env python3
"""
goto.py — Point the SO-100 arm at any celestial object and track it in real time.

Usage:
    python3 goto.py polaris
    python3 goto.py moon
    python3 goto.py --ra "5h55m10s" --dec "-7d24m25s"
    python3 goto.py --alt 45 --az 180
    python3 goto.py --record-pose tracking    # save current motor positions
    python3 goto.py --pose tracking polaris    # use recorded pose for motors 2,3

No ROS2 required — direct I2C (BNO055) + serial (Feetech servos).
"""

import sys
import argparse

from astropy.coordinates import SkyCoord

from goto.config import (
    OBSERVER_LAT, OBSERVER_LON, SERVO_PORT,
    MODE_IMU, MODE_NDOF, MODE_SERVO,
    SHOULDER_ROT, SHOULDER_PITCH, ELBOW, WRIST_PITCH, WRIST_ROLL,
    WHEEL_IDS, ADDR_LOCK, ADDR_MODE,
    STAR_CATALOG, SOLAR_SYSTEM,
    save_pose, load_pose, angle_diff,
)
from goto.imu import init_imu, read_imu, calib_str, calibrate_imu, gravity_pitch
from goto.servos import init_servos, read_servo, move_to_pose, stop_wheels
from goto.celestial import resolve_target, compute_altaz
from goto.strategy import WristOnlyStrategy
from goto.tracker import (
    running, slew_to_target, track_target,
    init_camera, init_speckle_pipeline,
)


def record_pose(args):
    """Read current servo positions and IMU, save as a named pose.

    Shows live servo + IMU readout while you position the arm.
    Press Enter when ready to save.
    """
    import scservo_sdk as sdk
    import time
    import sys
    import select
    from goto.config import SERVO_BAUD, ADDR_PRESENT_POSITION, ADDR_TORQUE_ENABLE

    ph = sdk.PortHandler(SERVO_PORT)
    pkt = sdk.PacketHandler(0)
    assert ph.openPort(), "Failed to open servo port"
    assert ph.setBaudRate(SERVO_BAUD), "Failed to set baudrate"

    # Disable torque so user can freely position the arm
    for sid in [SHOULDER_ROT, SHOULDER_PITCH, ELBOW, WRIST_PITCH, WRIST_ROLL]:
        pkt.write1ByteTxRx(ph, sid, ADDR_TORQUE_ENABLE, 0)

    imu_mode = MODE_NDOF if args.mode == 'ndof' else MODE_IMU
    imu_bus = init_imu(mode=imu_mode)

    print("Torque disabled — position the arm freely.")
    print("Live readout (press Enter to save):\n")

    sids = [SHOULDER_ROT, SHOULDER_PITCH, ELBOW, WRIST_PITCH, WRIST_ROLL]
    sid_names = ['ShRot', 'ShPit', 'Elbow', 'WPit ', 'WRoll']

    try:
        while True:
            positions = {}
            parts = []
            for sid, name in zip(sids, sid_names):
                pos, result, _ = pkt.read2ByteTxRx(ph, sid, ADDR_PRESENT_POSITION)
                if result == sdk.COMM_SUCCESS:
                    positions[sid] = pos
                    parts.append(f"{name}={pos:4d}")
                else:
                    parts.append(f"{name}=ERR ")

            imu = read_imu(imu_bus)
            line = "  " + "  ".join(parts) + f"  | H={imu['heading']:5.1f}° P={imu['pitch']:5.1f}°"
            print(f"\r{line}", end="", flush=True)

            # Check if Enter was pressed (non-blocking)
            if select.select([sys.stdin], [], [], 0.0)[0]:
                sys.stdin.readline()
                break
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nCancelled.")
        ph.closePort()
        imu_bus.close()
        return

    print()  # newline after the live readout

    # Re-enable torque
    for sid in sids:
        pkt.write1ByteTxRx(ph, sid, ADDR_TORQUE_ENABLE, 1)

    for sid, name in zip(sids, sid_names):
        print(f"  Motor {sid} ({name}): {positions.get(sid, 'ERR')}")
    print(f"  IMU: heading={imu['heading']:.1f}° pitch={imu['pitch']:.1f}°")

    save_pose(args.record_pose, positions, metadata={
        'heading': imu['heading'],
        'pitch': imu['pitch'],
    })

    imu_bus.close()
    ph.closePort()


def calibrate_pitch(args):
    """Interactive 3-point pitch calibration.

    Disables torque on wrist pitch so the user can manually position it
    at low/mid/high pitch angles. Records ticks + IMU at each point to
    compute direction and ticks-per-degree.

    If --pose is specified, locks motors 2,3 to the pose first so the
    calibration matches the tracking configuration.
    """
    import scservo_sdk as sdk
    from goto.config import (
        SERVO_BAUD, ADDR_PRESENT_POSITION, ADDR_TORQUE_ENABLE,
        POSES_DIR, SHOULDER_PITCH, ELBOW,
    )
    from goto.servos import move_servo, wait_servo
    import json, os

    imu_mode = MODE_NDOF if args.mode == 'ndof' else MODE_IMU
    imu_bus = init_imu(mode=imu_mode)
    mode_name = "NDOF (full fusion)" if imu_mode == MODE_NDOF else "IMU (accel+gyro, no mag)"
    print(f"BNO055 initialized — mode: {mode_name}")

    ph = sdk.PortHandler(SERVO_PORT)
    pkt = sdk.PacketHandler(0)
    assert ph.openPort(), "Failed to open servo port"
    assert ph.setBaudRate(SERVO_BAUD), "Failed to set baudrate"

    # Enable torque on all servos first
    for sid in range(1, 6):
        pkt.write1ByteTxRx(ph, sid, ADDR_TORQUE_ENABLE, 0)
    import time
    time.sleep(0.2)
    for sid in range(1, 6):
        pkt.write1ByteTxRx(ph, sid, ADDR_TORQUE_ENABLE, 1)
    time.sleep(0.05)

    # Lock motors 2,3 to pose if specified
    if args.pose:
        try:
            all_positions = load_pose(args.pose)
            locked = {k: v for k, v in all_positions.items()
                      if k in [SHOULDER_PITCH, ELBOW]}
            for sid, pos in locked.items():
                move_servo(ph, pkt, sid, pos, speed=200)
            time.sleep(1.0)
            for sid, pos in locked.items():
                wait_servo(ph, pkt, sid, pos, timeout=8)
            print(f"  Pose '{args.pose}': locked motors {locked}")
        except FileNotFoundError as e:
            print(f"  WARNING: {e}")

    # Disable torque on wrist pitch so user can move it freely
    pkt.write1ByteTxRx(ph, WRIST_PITCH, ADDR_TORQUE_ENABLE, 0)
    print(f"\nWrist pitch (motor {WRIST_PITCH}) torque DISABLED — move it freely.")
    print("Position the arm at 3 different pitch angles.\n")

    points = []
    labels = ["LOW pitch (pointing down/flat)", "MID pitch (middle angle)", "HIGH pitch (pointing up)"]

    for i, label in enumerate(labels):
        input(f"  [{i+1}/3] Position arm at {label}, then press Enter...")

        import time
        time.sleep(0.5)
        # Use gravity-based pitch (heading-independent)
        gpitch = gravity_pitch(imu_bus)
        imu = read_imu(imu_bus, samples=5, interval=0.03)
        pos, result, _ = pkt.read2ByteTxRx(ph, WRIST_PITCH, ADDR_PRESENT_POSITION)
        if result != sdk.COMM_SUCCESS:
            print(f"    ERROR: Can't read wrist pitch servo!")
            continue

        # Read motors 2,3 to verify they're holding position
        m2, _, _ = pkt.read2ByteTxRx(ph, SHOULDER_PITCH, ADDR_PRESENT_POSITION)
        m3, _, _ = pkt.read2ByteTxRx(ph, ELBOW, ADDR_PRESENT_POSITION)

        points.append({'ticks': pos, 'pitch': gpitch, 'heading': imu['heading']})
        print(f"    Recorded: WPt={pos} ticks, grav_pitch={gpitch:.1f}°, "
              f"quat_pitch={imu['pitch']:.1f}°, heading={imu['heading']:.1f}°  "
              f"[M2={m2} M3={m3}]")

    if len(points) < 2:
        print("Not enough points recorded. Aborting.")
        ph.closePort()
        imu_bus.close()
        return

    # Re-enable torque
    pkt.write1ByteTxRx(ph, WRIST_PITCH, ADDR_TORQUE_ENABLE, 1)
    print(f"\nWrist pitch torque re-enabled.")

    # Compute calibration from points
    # Use least-squares fit: pitch = a * ticks + b
    n = len(points)
    sum_t = sum(p['ticks'] for p in points)
    sum_p = sum(p['pitch'] for p in points)
    sum_tt = sum(p['ticks'] ** 2 for p in points)
    sum_tp = sum(p['ticks'] * p['pitch'] for p in points)

    denom = n * sum_tt - sum_t ** 2
    if abs(denom) < 1e-6:
        print("ERROR: All points have same tick value. Try moving the wrist more.")
        ph.closePort()
        imu_bus.close()
        return

    slope = (n * sum_tp - sum_t * sum_p) / denom  # degrees per tick
    intercept = (sum_p - slope * sum_t) / n

    pitch_dir = 1 if slope > 0 else -1
    ticks_per_deg = abs(1.0 / slope) if abs(slope) > 1e-6 else 11.3

    print(f"\n{'='*50}")
    print(f"  Calibration results:")
    print(f"  Direction: {'+ ticks = pitch UP' if pitch_dir > 0 else '+ ticks = pitch DOWN'}")
    print(f"  Ticks/degree: {ticks_per_deg:.1f}")
    print(f"  Slope: {slope:.4f} deg/tick")
    for p in points:
        predicted = slope * p['ticks'] + intercept
        err = p['pitch'] - predicted
        print(f"    WPt={p['ticks']} pitch={p['pitch']:.1f}° (fit: {predicted:.1f}°, err: {err:+.1f}°)")
    print(f"{'='*50}")

    # Save calibration
    cal_data = {
        'pitch_dir': pitch_dir,
        'ticks_per_deg': ticks_per_deg,
        'slope_deg_per_tick': slope,
        'intercept': intercept,
        'points': points,
    }
    os.makedirs(POSES_DIR, exist_ok=True)
    cal_path = os.path.join(POSES_DIR, 'pitch_cal.json')
    with open(cal_path, 'w') as f:
        json.dump(cal_data, f, indent=2)
    print(f"  Saved to {cal_path}")

    ph.closePort()
    imu_bus.close()


def main():
    parser = argparse.ArgumentParser(
        description="Point SO-100 arm at a celestial object and track it.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""Examples:
  python3 goto.py polaris
  python3 goto.py moon
  python3 goto.py --record-pose tracking
  python3 goto.py --pose tracking polaris
  python3 goto.py --ra "5h55m10s" --dec "-7d24m25s"
  python3 goto.py --alt 45 --az 180

Supported named targets:
  Solar system: """ + ", ".join(SOLAR_SYSTEM) + """
  Stars: """ + ", ".join(sorted(STAR_CATALOG.keys()))
    )
    parser.add_argument('target', nargs='?', help='Target name (e.g., polaris, moon, mars)')
    parser.add_argument('--ra', help='Right ascension (e.g., "5h55m10s")')
    parser.add_argument('--dec', help='Declination (e.g., "-7d24m25s")')
    parser.add_argument('--alt', type=float, help='Manual altitude in degrees')
    parser.add_argument('--az', type=float, help='Manual azimuth in degrees')
    parser.add_argument('--lat', type=float, default=OBSERVER_LAT, help='Observer latitude')
    parser.add_argument('--lon', type=float, default=OBSERVER_LON, help='Observer longitude')
    parser.add_argument('--no-capture', action='store_true', help='Disable camera capture')
    parser.add_argument('--capture-dir', default='speckle_captures', help='Directory for captures')
    parser.add_argument('--exposure', type=int, default=10000, help='Camera exposure in us')
    parser.add_argument('--burst-count', type=int, default=1,
                        help='Frames per burst (1=single-shot, 100=speckle mode)')
    parser.add_argument('--mode', choices=['imu', 'ndof'], default='ndof',
                        help='IMU fusion mode: ndof (full fusion, default) or imu (no mag)')
    parser.add_argument('--skip-cal', action='store_true', help='Skip IMU calibration routine')
    parser.add_argument('--record-pose', metavar='NAME',
                        help='Record current servo positions as a named pose and exit')
    parser.add_argument('--pose', metavar='NAME',
                        help='Load a recorded pose to lock motors 2,3 during tracking')
    parser.add_argument('--calibrate', action='store_true',
                        help='Interactive 3-point pitch calibration and exit')

    args = parser.parse_args()

    # ── Record pose mode ──
    if args.record_pose:
        record_pose(args)
        return

    # ── Calibrate pitch mode ──
    if args.calibrate:
        calibrate_pitch(args)
        return

    # ── Update observer location ──
    import goto.config as cfg
    cfg.OBSERVER_LAT = args.lat
    cfg.OBSERVER_LON = args.lon

    # ── Resolve target ──
    if args.alt is not None and args.az is not None:
        target_name = f"Alt={args.alt}° Az={args.az}°"
        target_info = ('fixed', (args.alt, args.az))
    elif args.ra and args.dec:
        target_name = f"RA={args.ra} Dec={args.dec}"
        coord = SkyCoord(ra=args.ra, dec=args.dec, frame='icrs')
        target_info = ('star', coord)
    elif args.target:
        target_name = args.target
        target_info = resolve_target(args.target)
        if target_info is None:
            print(f"Unknown target: '{args.target}'")
            print(f"Try one of: {', '.join(SOLAR_SYSTEM + sorted(STAR_CATALOG.keys()))}")
            sys.exit(1)
    else:
        parser.print_help()
        sys.exit(1)

    # Compute initial position
    target_alt, target_az = compute_altaz(target_info)

    print("=" * 60)
    print(f"  GOTO: {target_name}")
    print("=" * 60)
    print(f"  Position: Alt={target_alt:.2f}°  Az={target_az:.2f}°")
    print(f"  Observer: {args.lat:.2f}°N, {args.lon:.2f}°E")

    if target_alt < 0:
        print(f"\n  ERROR: Target is below the horizon ({target_alt:.1f}°)!")
        print(f"  Cannot point below the horizon. Try again when the target has risen.")
        sys.exit(1)

    # ── IMU mode ──
    imu_mode = MODE_NDOF if args.mode == 'ndof' else MODE_IMU
    mode_name = "NDOF (full fusion)" if imu_mode == MODE_NDOF else "IMU (accel+gyro, no mag)"

    # ── Load locked pose ──
    locked_pose = None
    if args.pose:
        try:
            all_positions = load_pose(args.pose)
            # Only lock motors 2 and 3
            locked_pose = {k: v for k, v in all_positions.items()
                          if k in [SHOULDER_PITCH, ELBOW]}
            print(f"  Pose '{args.pose}': locking motors {locked_pose}")
        except FileNotFoundError as e:
            print(f"  WARNING: {e} — using default home pose")

    # ── Initialize hardware ──
    print()
    imu_bus = init_imu(mode=imu_mode)
    print(f"BNO055 initialized — mode: {mode_name}")
    ph, pkt = init_servos(locked_pose=locked_pose)
    print(f"Servos connected on {SERVO_PORT}")

    imu = read_imu(imu_bus)
    print(f"IMU: Heading={imu['heading']:.1f}°  Pitch={imu['pitch']:.1f}°  "
          f"Calib: {calib_str(imu, imu_mode)}")

    # ── Calibration ──
    if not args.skip_cal:
        from goto.config import CAL_POSES
        def move_fn(pose_name):
            move_to_pose(ph, pkt, CAL_POSES[pose_name])
        calibrate_imu(imu_bus, move_fn, imu_mode)

    # ── Motion strategy ──
    strategy = WristOnlyStrategy()
    strategy.calibrate_pitch_dir(ph, pkt, imu_bus)
    print(f"  Strategy: WristOnly (ShRot=az, WPt=alt, wheels=fallback)")
    print(f"  Pitch dir: WPt={strategy.wrist_pitch_dir:+d}")

    # ── Camera / pipeline ──
    cam = None
    pipeline = None
    if not args.no_capture:
        cam = init_camera(args.exposure)
        pipeline = init_speckle_pipeline(
            cam, args.exposure, args.burst_count, args.capture_dir)
        if pipeline:
            cap_mode = "speckle" if args.burst_count > 1 else "single-shot"
            print(f"Capture pipeline: {cap_mode} ({args.burst_count} frames/burst)")

    # ── Run ──
    try:
        slew_to_target(target_info, imu_bus, ph, pkt, strategy,
                        target_name, pipeline, imu_mode)

        if not running:
            return

        track_target(target_info, target_name, imu_bus, ph, pkt, strategy,
                      pipeline, imu_mode)

    finally:
        # Final status
        imu = read_imu(imu_bus)
        gpitch = gravity_pitch(imu_bus)
        target_alt, target_az = compute_altaz(target_info)
        az_err = angle_diff(target_az, imu['heading'])
        alt_err = target_alt - gpitch

        print(f"\n{'='*60}")
        print(f"Final: Heading={imu['heading']:.1f}° Pitch={gpitch:.1f}°  "
              f"Error: Az={az_err:+.1f}° Alt={alt_err:+.1f}°")
        print("=" * 60)

        # Stop wheels and restore servo mode
        stop_wheels(ph, pkt)
        for sid in WHEEL_IDS:
            pkt.write1ByteTxRx(ph, sid, ADDR_LOCK, 0)
            pkt.write1ByteTxRx(ph, sid, ADDR_MODE, MODE_SERVO)
            pkt.write1ByteTxRx(ph, sid, ADDR_LOCK, 1)
        print("Base wheels stopped, restored to servo mode")

        if pipeline:
            print("Waiting for background processing to finish...")
            pipeline.wait_for_processing(timeout=300)
        if cam:
            cam.close()
            print("Camera closed")
        ph.closePort()
        imu_bus.close()


if __name__ == '__main__':
    main()
