"""
Burst storage: save/load speckle bursts with JSON sidecar metadata.

Directory layout:
    <base>/<target>/<session>/burst_NNNN/cam{0-3}_frame_NNNN.npy
    <base>/<target>/<session>/burst_NNNN/burst_meta.json
"""

import os
import json
import numpy as np
from datetime import datetime


def _burst_dir(base, target, session, burst_num):
    return os.path.join(base, target, session, f"burst_{burst_num:04d}")


def save_burst(frames_by_cam, metadata, base_dir, target, session, burst_num):
    """Save a burst of frames plus one JSON sidecar.

    Args:
        frames_by_cam: dict {cam_idx: list[np.ndarray]} — frames per camera
        metadata: dict with burst-level metadata (timestamps, IMU, servos, etc.)
        base_dir: root output directory
        target: target name (e.g. "polaris")
        session: session identifier (e.g. "20260313_220000")
        burst_num: integer burst number

    Returns:
        Path to burst directory.
    """
    bdir = _burst_dir(base_dir, target, session, burst_num)
    os.makedirs(bdir, exist_ok=True)

    for cam_idx, frames in frames_by_cam.items():
        for frame_num, frame in enumerate(frames):
            path = os.path.join(bdir, f"cam{cam_idx}_frame_{frame_num:04d}.npy")
            np.save(path, frame)

    meta_path = os.path.join(bdir, "burst_meta.json")
    with open(meta_path, 'w') as f:
        json.dump(metadata, f, indent=2, default=str)

    return bdir


def load_burst(burst_dir):
    """Load a saved burst.

    Returns:
        (frames_by_cam, metadata) where frames_by_cam is
        {cam_idx: list[np.ndarray]} sorted by frame number.
    """
    meta_path = os.path.join(burst_dir, "burst_meta.json")
    with open(meta_path) as f:
        metadata = json.load(f)

    frames_by_cam = {}
    for fname in sorted(os.listdir(burst_dir)):
        if not fname.endswith('.npy'):
            continue
        # Parse cam{idx}_frame_{num}.npy
        parts = fname.replace('.npy', '').split('_')
        cam_idx = int(parts[0].replace('cam', ''))
        frame = np.load(os.path.join(burst_dir, fname))
        frames_by_cam.setdefault(cam_idx, []).append(frame)

    return frames_by_cam, metadata


def list_sessions(base_dir, target=None):
    """List available sessions, optionally filtered by target.

    Returns:
        list of (target, session, burst_count) tuples.
    """
    results = []
    if not os.path.isdir(base_dir):
        return results

    targets = [target] if target else os.listdir(base_dir)
    for t in targets:
        tdir = os.path.join(base_dir, t)
        if not os.path.isdir(tdir):
            continue
        for session in sorted(os.listdir(tdir)):
            sdir = os.path.join(tdir, session)
            if not os.path.isdir(sdir):
                continue
            bursts = [d for d in os.listdir(sdir)
                      if d.startswith('burst_') and os.path.isdir(os.path.join(sdir, d))]
            results.append((t, session, len(bursts)))

    return results
