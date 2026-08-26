#!/usr/bin/env python3
"""Run MuJoCo with gated SONIC initialization and saved Psi0 actions."""

from __future__ import annotations

import csv
from dataclasses import dataclass
import os
from pathlib import Path
import sys

import numpy as np


BODY_SOURCE_INDICES = np.r_[0:22, 29:36]
HAND_TRAINING_TO_ACTUATOR = np.array([4, 5, 6, 0, 1, 2, 3], dtype=np.int64)


@dataclass(frozen=True)
class LifecycleMarkers:
    initial_pose_done: bool = False
    action_started: bool = False
    rollout_done: bool = False


@dataclass(frozen=True)
class SimLifecycleConfig:
    supported_hold_s: float = 1.0
    support_lower_s: float = 2.0
    post_unhang_settle_s: float = 2.5
    post_rollout_s: float = 2.0


class SimLifecycle:
    """Deterministic state machine for the official VLA startup sequence."""

    def __init__(self, config: SimLifecycleConfig):
        self.config = config
        self.physics_started_at: float | None = None
        self.lowering_started_at: float | None = None
        self.support_disabled_at: float | None = None
        self.action_gate_opened = False
        self.action_started_at: float | None = None
        self.rollout_done_at: float | None = None
        self.stop_emitted = False

    @property
    def physics_released(self) -> bool:
        return self.physics_started_at is not None

    def advance(self, sim_time: float, markers: LifecycleMarkers) -> list[str]:
        actions: list[str] = []
        if self.physics_started_at is None:
            if markers.initial_pose_done:
                self.physics_started_at = sim_time
                actions.append("release_physics")
            return actions

        if self.lowering_started_at is None:
            if sim_time - self.physics_started_at >= self.config.supported_hold_s:
                self.lowering_started_at = sim_time
                actions.append("lower_support")
            return actions

        if self.support_disabled_at is None:
            if sim_time - self.lowering_started_at >= self.config.support_lower_s:
                self.support_disabled_at = sim_time
                actions.append("disable_support")
            return actions

        if not self.action_gate_opened:
            if sim_time - self.support_disabled_at >= self.config.post_unhang_settle_s:
                self.action_gate_opened = True
                actions.append("open_action_gate")
            return actions

        if self.action_started_at is None:
            if markers.action_started:
                self.action_started_at = sim_time
                actions.append("learned_action_started")
            return actions

        if self.rollout_done_at is None:
            if markers.rollout_done:
                self.rollout_done_at = sim_time
                actions.append("learned_action_complete")
            return actions

        if not self.stop_emitted:
            if sim_time - self.rollout_done_at >= self.config.post_rollout_s:
                self.stop_emitted = True
                actions.append("stop_simulation")
        return actions


def load_recorded_initial_state(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Load the source 43-D state and root orientation from frame zero."""
    import pyarrow.parquet as pq

    table = pq.read_table(
        path,
        columns=["observation.state", "observation.root_orientation"],
    )
    if table.num_rows < 1:
        raise ValueError(f"recorded episode is empty: {path}")
    state = np.asarray(table["observation.state"][0].as_py(), dtype=np.float32)
    root_quaternion = np.asarray(
        table["observation.root_orientation"][0].as_py(), dtype=np.float64
    )
    if state.shape != (43,) or not np.isfinite(state).all():
        raise ValueError(
            f"recorded initial state must be finite 43-D, got {state.shape}"
        )
    if root_quaternion.shape != (4,) or not np.isfinite(root_quaternion).all():
        raise ValueError(
            "recorded initial root orientation must be finite 4-D, "
            f"got {root_quaternion.shape}"
        )
    return state, root_quaternion


def build_initial_mujoco_qpos(
    current_qpos: np.ndarray,
    body_qpos_indices: np.ndarray,
    left_hand_qpos_indices: np.ndarray,
    right_hand_qpos_indices: np.ndarray,
    state: np.ndarray,
) -> np.ndarray:
    """Initialize joints from the episode while aligning root yaw to the band."""
    qpos = np.asarray(current_qpos, dtype=np.float64).copy()
    body_indices = np.asarray(body_qpos_indices, dtype=np.int64).reshape(-1)
    left_indices = np.asarray(left_hand_qpos_indices, dtype=np.int64).reshape(-1)
    right_indices = np.asarray(right_hand_qpos_indices, dtype=np.int64).reshape(-1)
    state = np.asarray(state, dtype=np.float32).reshape(-1)
    if state.shape != (43,):
        raise ValueError(f"state must have shape (43,), got {state.shape}")
    if body_indices.shape != (29,):
        raise ValueError("body_qpos_indices must contain 29 entries")
    if left_indices.shape != (7,) or right_indices.shape != (7,):
        raise ValueError("each hand qpos index array must contain 7 entries")
    qpos[3:7] = np.array([1.0, 0.0, 0.0, 0.0])
    qpos[body_indices] = state[BODY_SOURCE_INDICES]
    qpos[left_indices] = state[22:29][HAND_TRAINING_TO_ACTUATOR]
    qpos[right_indices] = state[36:43][HAND_TRAINING_TO_ACTUATOR]
    return qpos


def _markers_from_paths(paths: dict[str, Path]) -> LifecycleMarkers:
    return LifecycleMarkers(
        initial_pose_done=paths["initial_pose_done"].exists(),
        action_started=paths["action_started"].exists(),
        rollout_done=paths["rollout_done"].exists(),
    )


def _publish_frozen_low_state(sim_env) -> None:
    observation = sim_env.prepare_obs()
    sim_env.obs = observation
    sim_env.unitree_bridge.PublishLowState(observation)
    if sim_env.unitree_bridge.joystick:
        sim_env.unitree_bridge.PublishWirelessController()


def main(config) -> None:
    sonic_root = Path(os.environ["SONIC_SOURCE_ROOT"]).expanduser().resolve()
    repo_root = Path(__file__).resolve().parents[2]
    for root in (repo_root, sonic_root):
        if str(root) not in sys.path:
            sys.path.insert(0, str(root))

    import mujoco

    from gear_sonic.data.robot_model.instantiation.g1 import (
        instantiate_g1_robot_model,
    )
    from gear_sonic.scripts.run_sim_loop import SimWrapper
    from gear_sonic.utils.mujoco_sim.simulator_factory import SimulatorFactory

    episode_path = Path(os.environ["SIM_INITIAL_EPISODE_PARQUET"])
    qpos_log = Path(os.environ["SIM_QPOS_LOG"])
    event_log = Path(os.environ["SIM_EVENT_LOG"])
    paths = {
        "initial_pose_done": Path(os.environ["SIM_INITIAL_POSE_DONE_MARKER"]),
        "support_disabled": Path(os.environ["SIM_SUPPORT_DISABLED_MARKER"]),
        "action_gate": Path(os.environ["SIM_ACTION_EXECUTE_TRIGGER"]),
        "action_started": Path(os.environ["SIM_ACTION_STARTED_MARKER"]),
        "rollout_done": Path(os.environ["SIM_ROLLOUT_DONE_MARKER"]),
    }
    lifecycle = SimLifecycle(
        SimLifecycleConfig(
            supported_hold_s=float(os.environ.get("SIM_SUPPORTED_HOLD", "1.0")),
            support_lower_s=float(os.environ.get("SIM_SUPPORT_LOWER", "2.0")),
            post_unhang_settle_s=float(os.environ.get("SIM_POST_UNHANG_SETTLE", "2.5")),
            post_rollout_s=float(os.environ.get("SIM_POST_ROLLOUT", "2.0")),
        )
    )
    state, _recorded_root_quaternion = load_recorded_initial_state(episode_path)
    wbc_config = config.load_wbc_yaml()
    wbc_config["ENV_NAME"] = config.env_name
    wrapper = SimWrapper(
        robot_model=instantiate_g1_robot_model(),
        env_name=config.env_name,
        config=wbc_config,
        onscreen=wbc_config.get("ENABLE_ONSCREEN", True),
        offscreen=wbc_config.get("ENABLE_OFFSCREEN", False),
        enable_image_publish=False,
    )
    simulator = wrapper.sim
    sim_env = simulator.sim_env
    if not sim_env.elastic_band.enable:
        raise RuntimeError("MuJoCo elastic support must begin enabled")

    body_indices = sim_env.body_joint_index + sim_env.qpos_offset - 1
    left_indices = sim_env.left_hand_index + sim_env.qpos_offset - 1
    right_indices = sim_env.right_hand_index + sim_env.qpos_offset - 1
    sim_env.mj_data.qpos[:] = build_initial_mujoco_qpos(
        sim_env.mj_data.qpos,
        body_indices,
        left_indices,
        right_indices,
        state,
    )
    sim_env.mj_data.qvel[:] = 0.0
    sim_env.mj_data.qacc[:] = 0.0
    mujoco.mj_forward(sim_env.mj_model, sim_env.mj_data)

    qpos_log.parent.mkdir(parents=True, exist_ok=True)
    event_log.parent.mkdir(parents=True, exist_ok=True)
    qpos_file = qpos_log.open("w", newline="", buffering=1)
    event_file = event_log.open("w", newline="", buffering=1)
    qpos_writer = csv.writer(qpos_file)
    event_writer = csv.writer(event_file)
    qpos_writer.writerow(
        ["sim_time", *[f"qpos_{index}" for index in range(sim_env.mj_model.nq)]]
    )
    event_writer.writerow(["sim_time", "event"])
    event_writer.writerow(["0.000000000", "recorded_initial_state_applied"])
    qpos_writer.writerow(["0.000000000", *sim_env.mj_data.qpos.tolist()])
    print(
        f"[PSI0 SIM] initialized from {episode_path.name}; waiting for initial pose",
        flush=True,
    )

    original_sim_step = sim_env.sim_step

    def handle_lifecycle_action(action: str, sim_time: float) -> None:
        event_writer.writerow([f"{sim_time:.9f}", action])
        print(f"[PSI0 SIM] {action} at {sim_time:.3f}s", flush=True)
        if action == "lower_support":
            sim_env.elastic_band.length = -0.2
        elif action == "disable_support":
            simulator.handle_keyboard_button("9")
            paths["support_disabled"].parent.mkdir(parents=True, exist_ok=True)
            paths["support_disabled"].write_text("ready\n")
        elif action == "open_action_gate":
            paths["action_gate"].parent.mkdir(parents=True, exist_ok=True)
            paths["action_gate"].write_text("ready\n")
        elif action == "stop_simulation":
            simulator._running = False

    def gated_sim_step():
        before_time = float(sim_env.mj_data.time)
        markers = _markers_from_paths(paths)
        for action in lifecycle.advance(before_time, markers):
            handle_lifecycle_action(action, before_time)
        if not lifecycle.physics_released:
            _publish_frozen_low_state(sim_env)
            return

        original_sim_step()
        sim_time = float(sim_env.mj_data.time)
        qpos_writer.writerow([f"{sim_time:.9f}", *sim_env.mj_data.qpos.tolist()])
        markers = _markers_from_paths(paths)
        for action in lifecycle.advance(sim_time, markers):
            handle_lifecycle_action(action, sim_time)

    sim_env.sim_step = gated_sim_step
    try:
        SimulatorFactory.start_simulator(
            simulator,
            as_thread=False,
            enable_image_publish=False,
            mp_start_method=config.mp_start_method,
            camera_port=config.camera_port,
        )
    finally:
        qpos_file.close()
        event_file.close()


if __name__ == "__main__":
    sonic_root = Path(os.environ["SONIC_SOURCE_ROOT"]).expanduser().resolve()
    if str(sonic_root) not in sys.path:
        sys.path.insert(0, str(sonic_root))
    import tyro

    from gear_sonic.utils.mujoco_sim.configs import SimLoopConfig

    main(tyro.cli(SimLoopConfig))
