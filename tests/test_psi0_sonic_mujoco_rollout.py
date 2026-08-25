from pathlib import Path

import numpy as np
import pytest

from scripts.eval.render_psi0_sonic_mujoco_rollout import (
    compose_three_panel_video,
    sample_qpos_at_action_times,
)
from scripts.eval.replay_psi0_sonic_predictions import (
    ProtocolV4Publisher,
    REQUIRED_LIFECYCLE_EVENTS,
    load_prediction_bundle,
    prepare_protocol_action,
    validate_lifecycle_events,
)
from scripts.eval.run_psi0_sonic_mujoco_rollout import (
    LifecycleMarkers,
    SimLifecycle,
    SimLifecycleConfig,
)


def _actions(rows: int, horizon: int) -> tuple[np.ndarray, np.ndarray]:
    prediction = np.arange(rows * horizon * 78, dtype=np.float32).reshape(
        rows, horizon, 78
    )
    target = prediction + 0.25
    return prediction, target


def test_load_prediction_bundle_uses_dense_first_step_and_target_initial_pose(
    tmp_path: Path,
):
    prediction, target = _actions(3, 2)
    path = tmp_path / "predictions.npz"
    np.savez_compressed(
        path,
        predicted_actions=prediction,
        target_actions=target,
        timestamps=np.array([0.0, 1 / 30, 2 / 30], dtype=np.float64),
    )

    bundle = load_prediction_bundle(path)

    np.testing.assert_array_equal(bundle.actions, prediction[:, 0])
    np.testing.assert_array_equal(bundle.initial_action, target[0, 0])
    np.testing.assert_allclose(bundle.fps, 30.0)


@pytest.mark.parametrize(
    ("prediction_shape", "target_shape"),
    [
        ((3, 2, 77), (3, 2, 77)),
        ((3, 2, 78), (3, 3, 78)),
        ((1, 2, 78), (1, 2, 78)),
    ],
)
def test_load_prediction_bundle_rejects_invalid_contract(
    tmp_path: Path,
    prediction_shape: tuple[int, ...],
    target_shape: tuple[int, ...],
):
    path = tmp_path / "bad.npz"
    np.savez_compressed(
        path,
        predicted_actions=np.zeros(prediction_shape, dtype=np.float32),
        target_actions=np.zeros(target_shape, dtype=np.float32),
        timestamps=np.arange(prediction_shape[0], dtype=np.float64) / 30.0,
    )

    with pytest.raises(ValueError):
        load_prediction_bundle(path)


def test_prepare_protocol_action_quantizes_tokens_and_reorders_both_hands():
    training_action = np.zeros(78, dtype=np.float32)
    training_action[:6] = [0.64, -0.66, 0.03, 0.09, -0.09, 0.624]
    training_action[64:71] = np.arange(7, dtype=np.float32)
    training_action[71:78] = np.arange(10, 17, dtype=np.float32)

    protocol_action = prepare_protocol_action(training_action)

    np.testing.assert_array_equal(
        protocol_action[:6],
        np.array([0.625, -0.625, 0.0, 0.0625, -0.0625, 0.625]),
    )
    np.testing.assert_array_equal(protocol_action[64:71], [4, 5, 6, 0, 1, 2, 3])
    np.testing.assert_array_equal(protocol_action[71:78], [14, 15, 16, 10, 11, 12, 13])


def test_pose_mode_can_be_selected_without_starting_or_stopping_control():
    class CaptureSocket:
        def __init__(self):
            self.sent = []

        def send(self, payload):
            self.sent.append(payload)

    publisher = ProtocolV4Publisher.__new__(ProtocolV4Publisher)
    publisher._socket = CaptureSocket()
    publisher._build_command_message = lambda **fields: fields

    publisher.select_pose_mode()

    assert publisher._socket.sent == [{"start": False, "stop": False, "planner": False}]


def test_validate_lifecycle_events_requires_official_order():
    validate_lifecycle_events(list(REQUIRED_LIFECYCLE_EVENTS))

    wrong = list(REQUIRED_LIFECYCLE_EVENTS)
    wrong[2], wrong[3] = wrong[3], wrong[2]
    with pytest.raises(ValueError, match="order"):
        validate_lifecycle_events(wrong)


def test_sim_lifecycle_initializes_while_supported_then_unhangs_and_releases_actions():
    lifecycle = SimLifecycle(
        SimLifecycleConfig(
            supported_hold_s=1.0,
            support_lower_s=2.0,
            post_unhang_settle_s=2.5,
            post_rollout_s=1.0,
        )
    )

    assert lifecycle.advance(0.0, LifecycleMarkers()) == []
    assert lifecycle.advance(0.0, LifecycleMarkers(initial_pose_done=True)) == [
        "release_physics"
    ]
    assert lifecycle.advance(0.99, LifecycleMarkers(initial_pose_done=True)) == []
    assert lifecycle.advance(1.0, LifecycleMarkers(initial_pose_done=True)) == [
        "lower_support"
    ]
    assert lifecycle.advance(3.0, LifecycleMarkers(initial_pose_done=True)) == [
        "disable_support"
    ]
    assert lifecycle.advance(5.49, LifecycleMarkers(initial_pose_done=True)) == []
    assert lifecycle.advance(
        5.5,
        LifecycleMarkers(initial_pose_done=True),
    ) == ["open_action_gate"]
    assert lifecycle.advance(
        5.6,
        LifecycleMarkers(
            initial_pose_done=True,
            action_started=True,
        ),
    ) == ["learned_action_started"]
    assert lifecycle.advance(
        29.1,
        LifecycleMarkers(
            initial_pose_done=True,
            action_started=True,
            rollout_done=True,
        ),
    ) == ["learned_action_complete"]
    assert (
        lifecycle.advance(
            30.09,
            LifecycleMarkers(
                initial_pose_done=True,
                action_started=True,
                rollout_done=True,
            ),
        )
        == []
    )
    assert lifecycle.advance(
        30.1,
        LifecycleMarkers(
            initial_pose_done=True,
            action_started=True,
            rollout_done=True,
        ),
    ) == ["stop_simulation"]


def test_sample_qpos_at_action_times_uses_nearest_physics_state():
    physics_times = np.arange(0.0, 1.01, 0.01)
    qpos = np.column_stack((physics_times, physics_times**2))

    sampled, sample_times = sample_qpos_at_action_times(
        physics_times=physics_times,
        qpos=qpos,
        action_start_time=0.2,
        frame_count=4,
        fps=10.0,
    )

    np.testing.assert_allclose(sample_times, [0.2, 0.3, 0.4, 0.5])
    np.testing.assert_allclose(sampled[:, 0], [0.2, 0.3, 0.4, 0.5])


def _write_video(path: Path, colors: list[int], width: int, height: int, fps: float):
    import cv2

    writer = cv2.VideoWriter(
        str(path),
        cv2.VideoWriter_fourcc(*"mp4v"),
        fps,
        (width, height),
    )
    assert writer.isOpened()
    for color in colors:
        writer.write(np.full((height, width, 3), color, dtype=np.uint8))
    writer.release()


def test_compose_three_panel_video_preserves_all_synchronized_frames(tmp_path: Path):
    import cv2

    open_loop = tmp_path / "open_loop.mp4"
    simulation = tmp_path / "simulation.mp4"
    output = tmp_path / "three_panel.mp4"
    _write_video(open_loop, [20, 40, 60], width=64, height=24, fps=10.0)
    _write_video(simulation, [80, 100, 120], width=32, height=24, fps=10.0)

    compose_three_panel_video(
        open_loop_video=open_loop,
        simulation_video=simulation,
        output_video=output,
        expected_frames=3,
        fps=10.0,
    )

    capture = cv2.VideoCapture(str(output))
    assert int(capture.get(cv2.CAP_PROP_FRAME_COUNT)) == 3
    assert int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)) == 96
    assert int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)) == 24
    assert capture.get(cv2.CAP_PROP_FPS) == pytest.approx(10.0)
    capture.release()
