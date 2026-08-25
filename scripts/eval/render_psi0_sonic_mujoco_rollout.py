#!/usr/bin/env python3
"""Render a SONIC MuJoCo rollout and compose it with open-loop diagnostics."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np


def sample_qpos_at_action_times(
    *,
    physics_times: np.ndarray,
    qpos: np.ndarray,
    action_start_time: float,
    frame_count: int,
    fps: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Select the nearest logged MuJoCo state for every action-video frame."""
    physics_times = np.asarray(physics_times, dtype=np.float64).reshape(-1)
    qpos = np.asarray(qpos, dtype=np.float64)
    if physics_times.size < 2 or np.any(np.diff(physics_times) <= 0.0):
        raise ValueError("physics_times must be strictly increasing")
    if qpos.ndim != 2 or qpos.shape[0] != physics_times.size:
        raise ValueError("qpos must have one row per physics timestamp")
    if not np.isfinite(qpos).all() or not np.isfinite(physics_times).all():
        raise ValueError("physics trajectory contains non-finite values")
    if frame_count <= 0 or fps <= 0.0:
        raise ValueError("frame_count and fps must be positive")
    target_times = action_start_time + np.arange(frame_count, dtype=np.float64) / fps
    if target_times[0] < physics_times[0] or target_times[-1] > physics_times[-1]:
        raise ValueError(
            "physics log does not cover the complete action interval: "
            f"[{target_times[0]:.6f}, {target_times[-1]:.6f}] outside "
            f"[{physics_times[0]:.6f}, {physics_times[-1]:.6f}]"
        )
    upper = np.searchsorted(physics_times, target_times, side="left")
    upper = np.clip(upper, 1, physics_times.size - 1)
    lower = upper - 1
    choose_upper = np.abs(physics_times[upper] - target_times) < np.abs(
        physics_times[lower] - target_times
    )
    indices = np.where(choose_upper, upper, lower)
    return qpos[indices].copy(), physics_times[indices].copy()


def load_qpos_csv(path: Path) -> tuple[np.ndarray, np.ndarray]:
    values = np.loadtxt(path, delimiter=",", skiprows=1)
    if values.ndim != 2 or values.shape[1] < 8:
        raise ValueError(f"invalid qpos CSV shape: {values.shape}")
    times = values[:, 0]
    qpos = values[:, 1:]
    keep = np.r_[True, np.diff(times) > 0.0]
    return times[keep], qpos[keep]


def read_event_time(path: Path, event: str) -> float:
    with path.open(newline="") as file:
        matches = [
            float(row["sim_time"])
            for row in csv.DictReader(file)
            if row["event"] == event
        ]
    if len(matches) != 1:
        raise ValueError(f"event {event!r} must occur exactly once, got {len(matches)}")
    return matches[0]


def render_qpos_video(
    *,
    model_xml: Path,
    qpos: np.ndarray,
    output_video: Path,
    fps: float,
    width: int = 640,
    height: int = 480,
) -> None:
    """Render a free tracking camera over a sampled MuJoCo trajectory."""
    import cv2
    import mujoco

    qpos = np.asarray(qpos, dtype=np.float64)
    model = mujoco.MjModel.from_xml_path(str(model_xml))
    if qpos.ndim != 2 or qpos.shape[1] != model.nq:
        raise ValueError(f"qpos must have shape (N, {model.nq}), got {qpos.shape}")
    data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, height=height, width=width)
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.azimuth = 125.0
    camera.elevation = -18.0
    camera.distance = 2.3
    pelvis_id = model.body("pelvis").id
    output_video.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(
        str(output_video),
        cv2.VideoWriter_fourcc(*"mp4v"),
        fps,
        (width, height),
    )
    if not writer.isOpened():
        renderer.close()
        raise RuntimeError(f"could not open simulation video: {output_video}")
    try:
        for frame_index, state in enumerate(qpos):
            data.qpos[:] = state
            mujoco.mj_forward(model, data)
            camera.lookat[:] = data.xpos[pelvis_id] + np.array([0.0, 0.0, 0.15])
            renderer.update_scene(data, camera=camera)
            frame = cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR)
            if height >= 80:
                cv2.rectangle(frame, (0, 0), (width, 34), (10, 10, 10), -1)
                cv2.putText(
                    frame,
                    f"MUJOCO / GEAR-SONIC v1.1 | frame {frame_index:04d}",
                    (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (240, 240, 240),
                    1,
                    cv2.LINE_AA,
                )
            writer.write(frame)
    finally:
        writer.release()
        renderer.close()


def compose_three_panel_video(
    *,
    open_loop_video: Path,
    simulation_video: Path,
    output_video: Path,
    expected_frames: int,
    fps: float,
) -> None:
    """Insert the simulation between the ego and diagnostics panels."""
    import cv2

    open_capture = cv2.VideoCapture(str(open_loop_video))
    sim_capture = cv2.VideoCapture(str(simulation_video))
    if not open_capture.isOpened() or not sim_capture.isOpened():
        open_capture.release()
        sim_capture.release()
        raise RuntimeError("could not open both input videos")
    open_width = int(open_capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(open_capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    if open_width <= 0 or height <= 0 or open_width % 2:
        open_capture.release()
        sim_capture.release()
        raise ValueError("open-loop video must contain two equal-width panels")
    panel_width = open_width // 2
    output_video.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(
        str(output_video),
        cv2.VideoWriter_fourcc(*"mp4v"),
        fps,
        (open_width + panel_width, height),
    )
    if not writer.isOpened():
        open_capture.release()
        sim_capture.release()
        raise RuntimeError(f"could not open output video: {output_video}")
    try:
        for _frame_index in range(expected_frames):
            open_ok, open_frame = open_capture.read()
            sim_ok, sim_frame = sim_capture.read()
            if not open_ok or not sim_ok:
                raise RuntimeError(
                    "an input video ended before the expected frame count"
                )
            if sim_frame.shape[:2] != (height, panel_width):
                sim_frame = cv2.resize(
                    sim_frame,
                    (panel_width, height),
                    interpolation=cv2.INTER_AREA,
                )
            writer.write(
                np.concatenate(
                    (
                        open_frame[:, :panel_width],
                        sim_frame,
                        open_frame[:, panel_width:],
                    ),
                    axis=1,
                )
            )
        open_extra, _ = open_capture.read()
        sim_extra, _ = sim_capture.read()
        if open_extra or sim_extra:
            raise RuntimeError("an input video contains more than the expected frames")
    finally:
        open_capture.release()
        sim_capture.release()
        writer.release()


def summarize_physics(qpos: np.ndarray, sample_times: np.ndarray) -> dict:
    qpos = np.asarray(qpos, dtype=np.float64)
    sample_times = np.asarray(sample_times, dtype=np.float64)
    root_xyz = qpos[:, :3]
    quaternions = qpos[:, 3:7]
    norms = np.linalg.norm(quaternions, axis=1, keepdims=True)
    quaternions = quaternions / np.maximum(norms, 1e-12)
    up_z = 1.0 - 2.0 * (np.square(quaternions[:, 1]) + np.square(quaternions[:, 2]))
    tilt_deg = np.degrees(np.arccos(np.clip(up_z, -1.0, 1.0)))
    horizontal = np.linalg.norm(root_xyz[:, :2] - root_xyz[0, :2], axis=1)
    return {
        "frames": int(qpos.shape[0]),
        "sample_time_start_s": float(sample_times[0]),
        "sample_time_end_s": float(sample_times[-1]),
        "root_xyz_start": root_xyz[0].tolist(),
        "root_xyz_end": root_xyz[-1].tolist(),
        "root_z_min_m": float(np.min(root_xyz[:, 2])),
        "root_z_max_m": float(np.max(root_xyz[:, 2])),
        "max_horizontal_displacement_m": float(np.max(horizontal)),
        "tilt_start_deg": float(tilt_deg[0]),
        "tilt_end_deg": float(tilt_deg[-1]),
        "max_tilt_deg": float(np.max(tilt_deg)),
        "first_tilt_over_45_deg": (
            int(np.flatnonzero(tilt_deg > 45.0)[0]) if np.any(tilt_deg > 45.0) else None
        ),
        "first_root_z_below_0_5_m": (
            int(np.flatnonzero(root_xyz[:, 2] < 0.5)[0])
            if np.any(root_xyz[:, 2] < 0.5)
            else None
        ),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model-xml", type=Path, required=True)
    parser.add_argument("--qpos-csv", type=Path, required=True)
    parser.add_argument("--events-csv", type=Path, required=True)
    parser.add_argument("--predictions-npz", type=Path, required=True)
    parser.add_argument("--open-loop-video", type=Path, required=True)
    parser.add_argument("--simulation-video", type=Path, required=True)
    parser.add_argument("--three-panel-video", type=Path, required=True)
    parser.add_argument("--metrics-json", type=Path, required=True)
    parser.add_argument("--fps", type=float, default=30.0)
    args = parser.parse_args()

    with np.load(args.predictions_npz) as archive:
        frame_count = int(archive["predicted_actions"].shape[0])
    physics_times, qpos = load_qpos_csv(args.qpos_csv)
    action_start = read_event_time(args.events_csv, "learned_action_started")
    sampled_qpos, sample_times = sample_qpos_at_action_times(
        physics_times=physics_times,
        qpos=qpos,
        action_start_time=action_start,
        frame_count=frame_count,
        fps=args.fps,
    )
    render_qpos_video(
        model_xml=args.model_xml,
        qpos=sampled_qpos,
        output_video=args.simulation_video,
        fps=args.fps,
    )
    compose_three_panel_video(
        open_loop_video=args.open_loop_video,
        simulation_video=args.simulation_video,
        output_video=args.three_panel_video,
        expected_frames=frame_count,
        fps=args.fps,
    )
    metrics = summarize_physics(sampled_qpos, sample_times)
    metrics["action_start_sim_time_s"] = action_start
    metrics["fps"] = args.fps
    args.metrics_json.write_text(json.dumps(metrics, indent=2, allow_nan=False) + "\n")


if __name__ == "__main__":
    main()
