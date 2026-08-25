from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np
import pytest

from scripts.eval.evaluate_psi0_sonic_open_loop import (
    prepare_output_directory,
    render_comparison_video,
    render_summary_image,
    summarize_predictions,
    validate_episode_contract,
    valid_horizon_mask,
)


def test_valid_horizon_mask_truncates_at_episode_end() -> None:
    mask = valid_horizon_mask(
        frame_indices=np.array([0, 3, 4]),
        episode_length=5,
        horizon=3,
    )

    np.testing.assert_array_equal(
        mask,
        np.array(
            [
                [True, True, True],
                [True, True, False],
                [True, False, False],
            ]
        ),
    )


def test_summarize_predictions_reports_sonic_groups_and_ignores_padding() -> None:
    target = np.zeros((2, 3, 78), dtype=np.float32)
    prediction = np.ones_like(target)
    prediction[1, 1:] = 100.0
    mask = np.array([[True, True, True], [True, False, False]])

    summary = summarize_predictions(
        prediction,
        target,
        mask,
        latencies_ms=np.array([90.0, 110.0]),
    )

    assert summary["full_horizon"]["latent"]["mae"] == pytest.approx(1.0)
    assert summary["full_horizon"]["left_hand"]["mae"] == pytest.approx(1.0)
    assert summary["full_horizon"]["right_hand"]["mae"] == pytest.approx(1.0)
    assert summary["first_step"]["all"]["mae"] == pytest.approx(1.0)
    assert summary["latency_ms"]["median"] == pytest.approx(100.0)
    assert summary["valid_action_rows"] == 4


def test_summarize_predictions_rejects_non_sonic_action_dimension() -> None:
    prediction = np.zeros((1, 2, 77), dtype=np.float32)
    target = np.zeros_like(prediction)
    mask = np.ones((1, 2), dtype=bool)

    with pytest.raises(ValueError, match="78"):
        summarize_predictions(
            prediction,
            target,
            mask,
            latencies_ms=np.array([1.0]),
        )


def test_render_comparison_video_preserves_frame_alignment(tmp_path: Path) -> None:
    input_path = tmp_path / "input.mp4"
    output_path = tmp_path / "output.mp4"
    writer = cv2.VideoWriter(
        str(input_path),
        cv2.VideoWriter_fourcc(*"mp4v"),
        10.0,
        (64, 48),
    )
    assert writer.isOpened()
    for frame_index in range(5):
        writer.write(np.full((48, 64, 3), frame_index * 40, dtype=np.uint8))
    writer.release()

    prediction = np.zeros((5, 78), dtype=np.float32)
    target = np.ones_like(prediction)
    render_comparison_video(
        input_video=input_path,
        output_video=output_path,
        prediction=prediction,
        target=target,
        latencies_ms=np.arange(5, dtype=np.float32),
        fps=10.0,
    )

    capture = cv2.VideoCapture(str(output_path))
    assert capture.isOpened()
    assert int(capture.get(cv2.CAP_PROP_FRAME_COUNT)) == 5
    assert int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)) == 128
    assert int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)) == 48
    capture.release()


@pytest.mark.parametrize(
    ("states", "actions", "video_frames", "inference_steps", "message"),
    [
        (
            np.zeros((5, 42), dtype=np.float32),
            np.zeros((5, 78), dtype=np.float32),
            5,
            4,
            "43-D states",
        ),
        (
            np.zeros((5, 43), dtype=np.float32),
            np.zeros((5, 77), dtype=np.float32),
            5,
            4,
            "78-D actions",
        ),
        (
            np.zeros((5, 43), dtype=np.float32),
            np.zeros((5, 78), dtype=np.float32),
            4,
            4,
            "video frame count",
        ),
        (
            np.zeros((5, 43), dtype=np.float32),
            np.zeros((5, 78), dtype=np.float32),
            5,
            2,
            "four inference steps",
        ),
    ],
)
def test_validate_episode_contract_rejects_incompatible_inputs(
    states: np.ndarray,
    actions: np.ndarray,
    video_frames: int,
    inference_steps: int,
    message: str,
) -> None:
    with pytest.raises(ValueError, match=message):
        validate_episode_contract(
            states=states,
            actions=actions,
            video_frame_count=video_frames,
            num_inference_steps=inference_steps,
        )


def test_prepare_output_directory_rejects_existing_result(tmp_path: Path) -> None:
    result_path = tmp_path / "result"
    result_path.mkdir()

    with pytest.raises(FileExistsError, match="already exists"):
        prepare_output_directory(result_path)


def test_render_summary_image_writes_readable_png(tmp_path: Path) -> None:
    output_path = tmp_path / "summary.png"
    prediction = np.zeros((5, 78), dtype=np.float32)
    target = np.ones_like(prediction)

    render_summary_image(
        output_image=output_path,
        prediction=prediction,
        target=target,
        latencies_ms=np.linspace(90.0, 110.0, 5),
        fps=30.0,
        checkpoint_step=40000,
        episode_index=83,
    )

    image = cv2.imread(str(output_path))
    assert image is not None
    assert image.shape == (900, 1400, 3)
