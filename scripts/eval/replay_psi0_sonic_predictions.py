#!/usr/bin/env python3
"""Replay saved Psi0 predictions through SONIC's latent-action protocol."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import os
from pathlib import Path
import sys
import time
from typing import Callable

import numpy as np


SONIC_ACTION_DIM = 78
MOTION_TOKEN_DIM = 64
HAND_TRAINING_TO_ACTUATOR = np.array([4, 5, 6, 0, 1, 2, 3], dtype=np.int64)
FSQ_MIN = -0.625
FSQ_MAX = 0.625
FSQ_STEP = 1.0 / 16.0
REQUIRED_LIFECYCLE_EVENTS = (
    "controller_ready",
    "initial_pose_started",
    "initial_pose_complete",
    "support_disabled",
    "vla_action_started",
    "vla_action_complete",
)


@dataclass(frozen=True)
class PredictionBundle:
    """Dense first-step actions and the target-derived initial pose."""

    actions: np.ndarray
    initial_action: np.ndarray
    timestamps: np.ndarray
    fps: float


def _require_actions(name: str, values: np.ndarray) -> np.ndarray:
    values = np.asarray(values, dtype=np.float32)
    if values.ndim != 3 or values.shape[-1] != SONIC_ACTION_DIM:
        raise ValueError(f"{name} must have shape (N, H, 78), got {values.shape}")
    if not np.isfinite(values).all():
        raise ValueError(f"{name} contains non-finite values")
    return values


def load_prediction_bundle(path: Path) -> PredictionBundle:
    """Load the dense evaluator output and select one immediate action per frame."""
    with np.load(path) as archive:
        required = {"predicted_actions", "target_actions", "timestamps"}
        missing = required.difference(archive.files)
        if missing:
            raise ValueError(f"prediction archive is missing: {sorted(missing)}")
        prediction = _require_actions("predicted_actions", archive["predicted_actions"])
        target = _require_actions("target_actions", archive["target_actions"])
        timestamps = np.asarray(archive["timestamps"], dtype=np.float64).reshape(-1)

    if prediction.shape != target.shape:
        raise ValueError(
            "predicted_actions and target_actions must have identical shapes; "
            f"got {prediction.shape} and {target.shape}"
        )
    if prediction.shape[0] < 2:
        raise ValueError("prediction archive must contain at least two frames")
    if timestamps.shape != (prediction.shape[0],):
        raise ValueError("timestamps must contain one value per prediction frame")
    if not np.isfinite(timestamps).all() or np.any(np.diff(timestamps) <= 0.0):
        raise ValueError("timestamps must be finite and strictly increasing")
    deltas = np.diff(timestamps)
    median_delta = float(np.median(deltas))
    if not np.allclose(deltas, median_delta, rtol=0.0, atol=1e-5):
        raise ValueError("timestamps must have a uniform cadence")
    return PredictionBundle(
        actions=prediction[:, 0].copy(),
        initial_action=target[0, 0].copy(),
        timestamps=timestamps.copy(),
        fps=1.0 / median_delta,
    )


def prepare_protocol_action(training_action: np.ndarray) -> np.ndarray:
    """Convert a training-order Psi0 action into a SONIC protocol-v4 action."""
    action = np.asarray(training_action, dtype=np.float32).reshape(-1)
    if action.shape != (SONIC_ACTION_DIM,):
        raise ValueError(f"training action must have shape (78,), got {action.shape}")
    if not np.isfinite(action).all():
        raise ValueError("training action contains non-finite values")
    token = np.clip(action[:MOTION_TOKEN_DIM], FSQ_MIN, FSQ_MAX)
    token = np.round(token / FSQ_STEP) * FSQ_STEP
    token = np.clip(token, FSQ_MIN, FSQ_MAX)
    left_hand = action[64:71][HAND_TRAINING_TO_ACTUATOR]
    right_hand = action[71:78][HAND_TRAINING_TO_ACTUATOR]
    return np.concatenate((token, left_hand, right_hand)).astype(np.float32)


def validate_lifecycle_events(events: list[str]) -> None:
    """Require each safety-gated lifecycle event exactly once and in order."""
    positions = []
    for required in REQUIRED_LIFECYCLE_EVENTS:
        matches = [index for index, event in enumerate(events) if event == required]
        if len(matches) != 1:
            raise ValueError(f"lifecycle event {required!r} must occur exactly once")
        positions.append(matches[0])
    if positions != sorted(positions):
        raise ValueError("lifecycle events are out of official order")


def _touch(path: Path, value: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(value)


def _wait_for_marker(
    marker: Path,
    *,
    timeout_s: float,
    hold: Callable[[], None] | None = None,
    period_s: float = 1.0 / 30.0,
) -> None:
    deadline = time.monotonic() + timeout_s
    while not marker.exists():
        if time.monotonic() >= deadline:
            raise TimeoutError(f"timed out waiting for marker: {marker}")
        if hold is not None:
            hold()
        time.sleep(period_s)


class ProtocolV4Publisher:
    """Publish official command and latent-action messages on one ZMQ PUB socket."""

    def __init__(self, host: str, port: int, topic: str):
        sonic_root = Path(
            os.environ.get(
                "SONIC_SOURCE_ROOT",
                Path(__file__).resolve().parents[2]
                / "third_party"
                / "GR00T-WholeBodyControl",
            )
        ).expanduser()
        if str(sonic_root) not in sys.path:
            sys.path.insert(0, str(sonic_root))
        import zmq

        from gear_sonic.utils.teleop.zmq.zmq_planner_sender import (
            build_command_message,
            pack_pose_message,
        )

        self._build_command_message = build_command_message
        self._pack_pose_message = pack_pose_message
        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.PUB)
        self._socket.bind(f"tcp://{host}:{port}")
        self._topic = topic
        self._frame_index = 0

    def command(self, *, start: bool, planner: bool) -> None:
        self._socket.send(
            self._build_command_message(
                start=start,
                stop=not start,
                planner=planner,
            )
        )

    def select_pose_mode(self) -> None:
        """Select streamed-motion mode without changing control-loop state."""
        self._socket.send(
            self._build_command_message(
                start=False,
                stop=False,
                planner=False,
            )
        )

    def action(self, protocol_action: np.ndarray) -> None:
        action = np.asarray(protocol_action, dtype=np.float32).reshape(-1)
        if action.shape != (SONIC_ACTION_DIM,):
            raise ValueError(
                f"protocol action must have shape (78,), got {action.shape}"
            )
        payload = {
            "token_state": action[None, :64],
            "frame_index": np.array([self._frame_index], dtype=np.int64),
            "left_hand_joints": action[None, 64:71],
            "right_hand_joints": action[None, 71:78],
        }
        self._socket.send(
            self._pack_pose_message(payload, topic=self._topic, version=4)
        )
        self._frame_index += 1

    def close(self) -> None:
        self._socket.close(linger=0)
        self._context.term()


def run_replay(args: argparse.Namespace) -> None:
    bundle = load_prediction_bundle(args.predictions_npz)
    if not np.isclose(bundle.fps, args.action_rate, atol=0.01):
        raise ValueError(
            f"prediction cadence is {bundle.fps:.6f} Hz, expected {args.action_rate:.6f} Hz"
        )
    initial_action = prepare_protocol_action(bundle.initial_action)
    actions = np.stack([prepare_protocol_action(row) for row in bundle.actions])
    publisher = ProtocolV4Publisher(args.host, args.port, args.topic)
    events: list[dict[str, float | str]] = []
    started_at = time.monotonic()

    def record(event: str) -> None:
        elapsed = time.monotonic() - started_at
        events.append({"event": event, "wall_time_s": elapsed})
        print(f"[PSI0 REPLAY] {event} at {elapsed:.3f}s", flush=True)

    period = 1.0 / args.action_rate
    try:
        time.sleep(args.subscriber_warmup_seconds)
        _wait_for_marker(
            args.controller_ready_trigger,
            timeout_s=args.marker_timeout_seconds,
        )
        record("controller_ready")

        # Select POSE mode before priming or starting control.  Mode selection
        # triggers a safety reset that clears buffered streamed motion, so the
        # token warmup must happen strictly after this phase.
        mode_deadline = time.monotonic() + args.pose_mode_select_seconds
        while time.monotonic() < mode_deadline:
            publisher.select_pose_mode()
            time.sleep(0.1)

        buffer_deadline = time.monotonic() + args.pose_buffer_warmup_seconds
        while time.monotonic() < buffer_deadline:
            publisher.action(initial_action)
            time.sleep(period)

        # Prime the pose subscriber before starting control.  The upstream
        # planner-first path can expose operator_state.start to the 50 Hz
        # control thread before the 10 Hz planner has produced planner_motion.
        # In simulation we can safely start in POSE mode while the elastic
        # support is still active, then unhang only after the initial token has
        # been held long enough for the controller to settle.
        record("initial_pose_started")

        initial_deadline = time.monotonic() + args.initial_pose_hold_seconds
        next_command = 0.0
        while time.monotonic() < initial_deadline:
            now = time.monotonic()
            if now >= next_command:
                publisher.command(start=True, planner=False)
                next_command = now + 0.25
            publisher.action(initial_action)
            time.sleep(period)
        record("initial_pose_complete")
        _touch(args.initial_pose_done_marker, "ready\n")

        support_recorded = False
        action_gate_deadline = time.monotonic() + args.marker_timeout_seconds
        while not args.action_execute_trigger.exists():
            if time.monotonic() >= action_gate_deadline:
                raise TimeoutError(
                    f"timed out waiting for marker: {args.action_execute_trigger}"
                )
            if args.support_disabled_marker.exists() and not support_recorded:
                record("support_disabled")
                support_recorded = True
            publisher.action(initial_action)
            time.sleep(period)
        if not support_recorded:
            if not args.support_disabled_marker.exists():
                raise RuntimeError(
                    "action gate opened before the MuJoCo support-disabled marker"
                )
            record("support_disabled")
        record("vla_action_started")
        _touch(args.action_started_marker, "started\n")

        replay_started_at = time.monotonic()
        timing = []
        relative_times = bundle.timestamps - bundle.timestamps[0]
        for frame_index, (action, target_s) in enumerate(zip(actions, relative_times)):
            deadline = replay_started_at + float(target_s)
            remaining = deadline - time.monotonic()
            if remaining > 0.0:
                time.sleep(remaining)
            published_at = time.monotonic()
            publisher.action(action)
            timing.append(
                {
                    "frame_index": frame_index,
                    "target_s": float(target_s),
                    "actual_s": published_at - replay_started_at,
                    "lateness_s": published_at - deadline,
                }
            )

        hold_deadline = time.monotonic() + args.post_hold_seconds
        while time.monotonic() < hold_deadline:
            publisher.action(actions[-1])
            time.sleep(period)
        record("vla_action_complete")
        validate_lifecycle_events([str(item["event"]) for item in events])
        report = {
            "predictions_npz": str(args.predictions_npz),
            "action_rate_hz": args.action_rate,
            "action_frames": int(actions.shape[0]),
            "initial_pose_source": "target_actions[0,0]",
            "startup_mode": "pose_first_while_supported",
            "hand_training_to_actuator": HAND_TRAINING_TO_ACTUATOR.tolist(),
            "fsq": {"min": FSQ_MIN, "max": FSQ_MAX, "step": FSQ_STEP},
            "events": events,
            "max_lateness_s": max(item["lateness_s"] for item in timing),
            "mean_lateness_s": float(np.mean([item["lateness_s"] for item in timing])),
            "timing": timing,
        }
        args.timing_json.parent.mkdir(parents=True, exist_ok=True)
        args.timing_json.write_text(json.dumps(report, indent=2))
        _touch(args.done_marker, "done\n")
    finally:
        publisher.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--predictions-npz", type=Path, required=True)
    parser.add_argument("--controller-ready-trigger", type=Path, required=True)
    parser.add_argument("--initial-pose-done-marker", type=Path, required=True)
    parser.add_argument("--support-disabled-marker", type=Path, required=True)
    parser.add_argument("--action-execute-trigger", type=Path, required=True)
    parser.add_argument("--action-started-marker", type=Path, required=True)
    parser.add_argument("--done-marker", type=Path, required=True)
    parser.add_argument("--timing-json", type=Path, required=True)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5556)
    parser.add_argument("--topic", default="pose")
    parser.add_argument("--action-rate", type=float, default=30.0)
    parser.add_argument("--subscriber-warmup-seconds", type=float, default=1.0)
    parser.add_argument("--pose-mode-select-seconds", type=float, default=0.5)
    parser.add_argument("--pose-buffer-warmup-seconds", type=float, default=0.5)
    parser.add_argument("--initial-pose-hold-seconds", type=float, default=2.0)
    parser.add_argument("--post-hold-seconds", type=float, default=2.0)
    parser.add_argument("--marker-timeout-seconds", type=float, default=120.0)
    return parser.parse_args()


if __name__ == "__main__":
    run_replay(parse_args())
