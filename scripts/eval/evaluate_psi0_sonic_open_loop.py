"""Dense open-loop evaluation of Psi0 on a recorded SONIC episode."""

from __future__ import annotations

from collections.abc import Mapping
import json
import os
from pathlib import Path
import time
from typing import Any

import numpy as np


SONIC_ACTION_DIM = 78
ACTION_GROUPS: Mapping[str, slice] = {
    "all": slice(0, 78),
    "latent": slice(0, 64),
    "left_hand": slice(64, 71),
    "right_hand": slice(71, 78),
}


def validate_episode_contract(
    *,
    states: np.ndarray,
    actions: np.ndarray,
    video_frame_count: int,
    num_inference_steps: int,
) -> None:
    """Validate the recorded episode before loading the GPU checkpoint."""
    states = np.asarray(states)
    actions = np.asarray(actions)
    if states.ndim != 2 or states.shape[1] != 43:
        raise ValueError(f"Expected 43-D states, got shape {states.shape}")
    if actions.ndim != 2 or actions.shape[1] != SONIC_ACTION_DIM:
        raise ValueError(f"Expected 78-D actions, got shape {actions.shape}")
    if states.shape[0] != actions.shape[0]:
        raise ValueError("state and action row counts must match")
    if video_frame_count != states.shape[0]:
        raise ValueError(
            "The video frame count must match the parquet episode length; "
            f"got {video_frame_count} and {states.shape[0]}"
        )
    if num_inference_steps != 4:
        raise ValueError(
            "This checkpoint evaluation requires four inference steps; "
            f"got {num_inference_steps}"
        )


def prepare_output_directory(output_directory: Path) -> None:
    """Create a new result directory without overwriting earlier evaluations."""
    try:
        output_directory.mkdir(parents=True, exist_ok=False)
    except FileExistsError as error:
        raise FileExistsError(
            f"Output directory already exists: {output_directory}"
        ) from error


def valid_horizon_mask(
    frame_indices: np.ndarray,
    episode_length: int,
    horizon: int,
) -> np.ndarray:
    """Return which future action rows exist for each episode-local frame."""
    indices = np.asarray(frame_indices)
    if indices.ndim != 1:
        raise ValueError("frame_indices must be one-dimensional")
    if episode_length <= 0:
        raise ValueError("episode_length must be positive")
    if horizon <= 0:
        raise ValueError("horizon must be positive")
    if np.any(indices < 0) or np.any(indices >= episode_length):
        raise ValueError("frame_indices must be within the episode")
    return indices[:, None] + np.arange(horizon)[None, :] < episode_length


def _group_metrics(
    prediction: np.ndarray,
    target: np.ndarray,
    mask: np.ndarray,
    group: slice,
) -> dict[str, float | None]:
    predicted_values = prediction[:, :, group][mask].reshape(-1)
    target_values = target[:, :, group][mask].reshape(-1)
    error = predicted_values - target_values
    correlation: float | None
    if (
        predicted_values.size < 2
        or np.std(predicted_values) == 0.0
        or np.std(target_values) == 0.0
    ):
        correlation = None
    else:
        correlation = float(np.corrcoef(predicted_values, target_values)[0, 1])
    return {
        "mae": float(np.mean(np.abs(error))),
        "rmse": float(np.sqrt(np.mean(np.square(error)))),
        "pearson": correlation,
    }


def _transition_l2(values: np.ndarray, group: slice) -> dict[str, float]:
    transitions = np.linalg.norm(np.diff(values[:, group], axis=0), axis=1)
    if transitions.size == 0:
        return {"mean": 0.0, "max": 0.0}
    return {
        "mean": float(np.mean(transitions)),
        "max": float(np.max(transitions)),
    }


def summarize_predictions(
    prediction: np.ndarray,
    target: np.ndarray,
    valid_mask: np.ndarray,
    latencies_ms: np.ndarray,
) -> dict[str, Any]:
    """Summarize dense SONIC action chunks using only valid future rows."""
    prediction = np.asarray(prediction, dtype=np.float64)
    target = np.asarray(target, dtype=np.float64)
    valid_mask = np.asarray(valid_mask, dtype=bool)
    latencies_ms = np.asarray(latencies_ms, dtype=np.float64)
    if prediction.shape != target.shape or prediction.ndim != 3:
        raise ValueError("prediction and target must have matching (N, H, D) shapes")
    if prediction.shape[-1] != SONIC_ACTION_DIM:
        raise ValueError(f"SONIC actions must have dimension {SONIC_ACTION_DIM}")
    if valid_mask.shape != prediction.shape[:2]:
        raise ValueError("valid_mask must match prediction's (N, H) dimensions")
    if latencies_ms.shape != (prediction.shape[0],):
        raise ValueError("latencies_ms must contain one value per prediction")
    if not np.all(valid_mask[:, 0]):
        raise ValueError("the first action in every prediction must be valid")

    first_prediction = prediction[:, :1]
    first_target = target[:, :1]
    first_mask = np.ones((prediction.shape[0], 1), dtype=bool)
    first_step = {
        name: _group_metrics(first_prediction, first_target, first_mask, group)
        for name, group in ACTION_GROUPS.items()
    }
    full_horizon = {
        name: _group_metrics(prediction, target, valid_mask, group)
        for name, group in ACTION_GROUPS.items()
    }

    horizon_mae: dict[str, list[float | None]] = {}
    for name, group in ACTION_GROUPS.items():
        values: list[float | None] = []
        for horizon_index in range(prediction.shape[1]):
            horizon_valid = valid_mask[:, horizon_index]
            if not np.any(horizon_valid):
                values.append(None)
                continue
            error = (
                prediction[horizon_valid, horizon_index, group]
                - target[horizon_valid, horizon_index, group]
            )
            values.append(float(np.mean(np.abs(error))))
        horizon_mae[name] = values

    transition_l2 = {
        name: {
            "prediction": _transition_l2(prediction[:, 0], group),
            "target": _transition_l2(target[:, 0], group),
        }
        for name, group in ACTION_GROUPS.items()
    }
    return {
        "samples": int(prediction.shape[0]),
        "prediction_horizon": int(prediction.shape[1]),
        "valid_action_rows": int(np.count_nonzero(valid_mask)),
        "first_step": first_step,
        "full_horizon": full_horizon,
        "horizon_mae": horizon_mae,
        "transition_l2": transition_l2,
        "latency_ms": {
            "mean": float(np.mean(latencies_ms)),
            "median": float(np.median(latencies_ms)),
            "p95": float(np.percentile(latencies_ms, 95)),
            "max": float(np.max(latencies_ms)),
        },
    }


def render_comparison_video(
    *,
    input_video: Path,
    output_video: Path,
    prediction: np.ndarray,
    target: np.ndarray,
    latencies_ms: np.ndarray,
    fps: float,
) -> None:
    """Render each recorded VLA input frame beside first-step diagnostics."""
    import cv2

    prediction = np.asarray(prediction, dtype=np.float32)
    target = np.asarray(target, dtype=np.float32)
    latencies_ms = np.asarray(latencies_ms, dtype=np.float32)
    if prediction.shape != target.shape or prediction.ndim != 2:
        raise ValueError("prediction and target must have matching (N, D) shapes")
    if prediction.shape[1] != SONIC_ACTION_DIM:
        raise ValueError(f"SONIC actions must have dimension {SONIC_ACTION_DIM}")
    if latencies_ms.shape != (prediction.shape[0],):
        raise ValueError("latencies_ms must contain one value per frame")
    if fps <= 0:
        raise ValueError("fps must be positive")

    capture = cv2.VideoCapture(str(input_video))
    if not capture.isOpened():
        raise RuntimeError(f"Could not open input video: {input_video}")
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    if width <= 0 or height <= 0:
        capture.release()
        raise RuntimeError(f"Input video has invalid dimensions: {input_video}")

    output_video.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(
        str(output_video),
        cv2.VideoWriter_fourcc(*"mp4v"),
        fps,
        (width * 2, height),
    )
    if not writer.isOpened():
        capture.release()
        raise RuntimeError(f"Could not open output video: {output_video}")

    latent_mae = np.mean(np.abs(prediction[:, :64] - target[:, :64]), axis=1)
    left_mae = np.mean(np.abs(prediction[:, 64:71] - target[:, 64:71]), axis=1)
    right_mae = np.mean(np.abs(prediction[:, 71:78] - target[:, 71:78]), axis=1)
    scale_y = height / 480.0
    font_scale = max(0.2, 0.58 * scale_y)
    thickness = max(1, round(1.5 * scale_y))

    def y(value: int) -> int:
        return max(0, min(height - 1, round(value * scale_y)))

    def draw_trace(
        canvas: np.ndarray,
        values: np.ndarray,
        top: int,
        bottom: int,
        color: tuple[int, int, int],
        value_min: float,
        value_max: float,
    ) -> None:
        if values.size < 2:
            return
        x_coordinates = np.linspace(12, width - 12, values.size)
        denominator = max(value_max - value_min, 1e-6)
        normalized = np.clip((values - value_min) / denominator, 0.0, 1.0)
        y_coordinates = bottom - normalized * max(bottom - top, 1)
        points = np.column_stack((x_coordinates, y_coordinates)).astype(np.int32)
        cv2.polylines(canvas, [points], False, color, thickness, cv2.LINE_AA)

    try:
        for frame_index in range(prediction.shape[0]):
            ok, frame = capture.read()
            if not ok:
                raise RuntimeError(
                    f"Input video ended before prediction row {frame_index}"
                )
            dashboard = np.full((height, width, 3), 20, dtype=np.uint8)
            cv2.putText(
                dashboard,
                f"PSI0 OPEN LOOP | frame {frame_index:04d} | {frame_index / fps:05.2f}s",
                (10, y(28)),
                cv2.FONT_HERSHEY_SIMPLEX,
                font_scale,
                (240, 240, 240),
                thickness,
                cv2.LINE_AA,
            )
            cv2.putText(
                dashboard,
                (
                    f"latent MAE {latent_mae[frame_index]:.4f}  "
                    f"left {left_mae[frame_index]:.4f}  "
                    f"right {right_mae[frame_index]:.4f}  "
                    f"{latencies_ms[frame_index]:.1f} ms"
                ),
                (10, y(58)),
                cv2.FONT_HERSHEY_SIMPLEX,
                font_scale,
                (210, 210, 210),
                thickness,
                cv2.LINE_AA,
            )
            cv2.putText(
                dashboard,
                "CURRENT 64-D SONIC LATENT   target=green  prediction=orange",
                (10, y(90)),
                cv2.FONT_HERSHEY_SIMPLEX,
                font_scale * 0.8,
                (180, 180, 180),
                thickness,
                cv2.LINE_AA,
            )
            latent_values = np.concatenate(
                (prediction[frame_index, :64], target[frame_index, :64])
            )
            latent_min = float(np.min(latent_values))
            latent_max = float(np.max(latent_values))
            draw_trace(
                dashboard,
                target[frame_index, :64],
                y(105),
                y(220),
                (70, 220, 100),
                latent_min,
                latent_max,
            )
            draw_trace(
                dashboard,
                prediction[frame_index, :64],
                y(105),
                y(220),
                (40, 150, 255),
                latent_min,
                latent_max,
            )
            cv2.putText(
                dashboard,
                "CURRENT 14-D HAND ACTION",
                (10, y(250)),
                cv2.FONT_HERSHEY_SIMPLEX,
                font_scale * 0.8,
                (180, 180, 180),
                thickness,
                cv2.LINE_AA,
            )
            hand_values = np.concatenate(
                (prediction[frame_index, 64:], target[frame_index, 64:])
            )
            hand_min = float(np.min(hand_values))
            hand_max = float(np.max(hand_values))
            draw_trace(
                dashboard,
                target[frame_index, 64:],
                y(265),
                y(335),
                (70, 220, 100),
                hand_min,
                hand_max,
            )
            draw_trace(
                dashboard,
                prediction[frame_index, 64:],
                y(265),
                y(335),
                (40, 150, 255),
                hand_min,
                hand_max,
            )
            cv2.putText(
                dashboard,
                "ROLLING FIRST-STEP MAE   latent=white  left=cyan  right=magenta",
                (10, y(365)),
                cv2.FONT_HERSHEY_SIMPLEX,
                font_scale * 0.75,
                (180, 180, 180),
                thickness,
                cv2.LINE_AA,
            )
            rolling_values = np.concatenate(
                (
                    latent_mae[: frame_index + 1],
                    left_mae[: frame_index + 1],
                    right_mae[: frame_index + 1],
                )
            )
            rolling_max = max(float(np.max(rolling_values)), 1e-6)
            draw_trace(
                dashboard,
                latent_mae[: frame_index + 1],
                y(380),
                y(460),
                (240, 240, 240),
                0.0,
                rolling_max,
            )
            draw_trace(
                dashboard,
                left_mae[: frame_index + 1],
                y(380),
                y(460),
                (255, 220, 40),
                0.0,
                rolling_max,
            )
            draw_trace(
                dashboard,
                right_mae[: frame_index + 1],
                y(380),
                y(460),
                (220, 60, 220),
                0.0,
                rolling_max,
            )
            writer.write(np.concatenate((frame, dashboard), axis=1))

        extra_frame, _ = capture.read()
        if extra_frame:
            raise RuntimeError("Input video contains more frames than predictions")
    finally:
        capture.release()
        writer.release()


def render_summary_image(
    *,
    output_image: Path,
    prediction: np.ndarray,
    target: np.ndarray,
    latencies_ms: np.ndarray,
    fps: float,
    checkpoint_step: int,
    episode_index: int,
) -> None:
    """Write a dependency-light episode summary plot with OpenCV."""
    import cv2

    prediction = np.asarray(prediction, dtype=np.float32)
    target = np.asarray(target, dtype=np.float32)
    latencies_ms = np.asarray(latencies_ms, dtype=np.float32)
    if prediction.shape != target.shape or prediction.ndim != 2:
        raise ValueError("prediction and target must have matching (N, D) shapes")
    if prediction.shape[1] != SONIC_ACTION_DIM:
        raise ValueError(f"SONIC actions must have dimension {SONIC_ACTION_DIM}")
    if latencies_ms.shape != (prediction.shape[0],):
        raise ValueError("latencies_ms must contain one value per frame")
    if fps <= 0:
        raise ValueError("fps must be positive")

    canvas = np.full((900, 1400, 3), 248, dtype=np.uint8)
    cv2.putText(
        canvas,
        f"Psi0 checkpoint {checkpoint_step} | SONIC v1.1 episode {episode_index}",
        (60, 45),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        (20, 20, 20),
        2,
        cv2.LINE_AA,
    )
    seconds = np.arange(prediction.shape[0], dtype=np.float32) / fps
    series = [
        (
            "First-step 64-D latent MAE",
            [
                (
                    np.mean(np.abs(prediction[:, :64] - target[:, :64]), axis=1),
                    (20, 20, 20),
                    "latent",
                )
            ],
        ),
        (
            "First-step hand-action MAE",
            [
                (
                    np.mean(np.abs(prediction[:, 64:71] - target[:, 64:71]), axis=1),
                    (220, 150, 20),
                    "left",
                ),
                (
                    np.mean(np.abs(prediction[:, 71:78] - target[:, 71:78]), axis=1),
                    (180, 30, 180),
                    "right",
                ),
            ],
        ),
        (
            "Hand-action L2 magnitude",
            [
                (
                    np.linalg.norm(prediction[:, 64:71], axis=1),
                    (20, 130, 240),
                    "pred left",
                ),
                (
                    np.linalg.norm(target[:, 64:71], axis=1),
                    (30, 170, 50),
                    "target left",
                ),
                (
                    np.linalg.norm(prediction[:, 71:78], axis=1),
                    (30, 30, 220),
                    "pred right",
                ),
                (
                    np.linalg.norm(target[:, 71:78], axis=1),
                    (180, 100, 20),
                    "target right",
                ),
            ],
        ),
        (
            "Inference latency (ms)",
            [(latencies_ms, (120, 40, 130), "latency")],
        ),
    ]
    panel_left, panel_right = 90, 1350
    for panel_index, (title, traces) in enumerate(series):
        panel_top = 75 + panel_index * 200
        panel_bottom = panel_top + 155
        cv2.rectangle(
            canvas,
            (panel_left, panel_top),
            (panel_right, panel_bottom),
            (205, 205, 205),
            1,
        )
        cv2.putText(
            canvas,
            title,
            (panel_left, panel_top - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (35, 35, 35),
            1,
            cv2.LINE_AA,
        )
        all_values = np.concatenate([trace[0] for trace in traces])
        value_min = min(0.0, float(np.min(all_values)))
        value_max = max(float(np.max(all_values)), value_min + 1e-6)
        for trace_index, (values, color, label) in enumerate(traces):
            x_coordinates = np.linspace(panel_left + 5, panel_right - 5, values.size)
            normalized = (values - value_min) / (value_max - value_min)
            y_coordinates = (
                panel_bottom - 5 - normalized * (panel_bottom - panel_top - 10)
            )
            points = np.column_stack((x_coordinates, y_coordinates)).astype(np.int32)
            if points.shape[0] > 1:
                cv2.polylines(
                    canvas,
                    [points],
                    False,
                    color,
                    2,
                    cv2.LINE_AA,
                )
            cv2.putText(
                canvas,
                label,
                (panel_right - 150, panel_top + 18 + trace_index * 18),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                color,
                1,
                cv2.LINE_AA,
            )
        cv2.putText(
            canvas,
            f"0.0 s                                               {seconds[-1]:.2f} s",
            (panel_left, panel_bottom + 18),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (80, 80, 80),
            1,
            cv2.LINE_AA,
        )
    output_image.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output_image), canvas):
        raise RuntimeError(f"Could not write summary image: {output_image}")


def main() -> None:
    """Run dense episode-video inference and write reproducible artifacts."""
    import argparse

    import cv2
    import pyarrow.parquet as pq
    import torch
    from tqdm.auto import tqdm

    parser = argparse.ArgumentParser(
        description="Dense Psi0 open-loop evaluation on a recorded SONIC episode"
    )
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--checkpoint-step", type=int, default=40000)
    parser.add_argument("--gpu", type=int, default=0)
    parser.add_argument("--episode-index", type=int, default=83)
    parser.add_argument("--num-inference-steps", type=int, default=4)
    parser.add_argument("--episode-parquet", type=Path, required=True)
    parser.add_argument("--input-video", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()

    project_root = Path(__file__).resolve().parents[2]
    os.chdir(project_root)
    required_paths = {
        "run directory": args.run_dir,
        "run argv": args.run_dir / "argv.txt",
        "run configuration": args.run_dir / "run_config.json",
        "checkpoint": args.run_dir / "checkpoints" / f"ckpt_{args.checkpoint_step}",
        "episode parquet": args.episode_parquet,
        "episode video": args.input_video,
    }
    missing = [name for name, path in required_paths.items() if not path.exists()]
    if missing:
        raise FileNotFoundError("Missing required paths: " + ", ".join(missing))

    parquet_table = pq.read_table(args.episode_parquet)
    recorded_states = np.asarray(parquet_table["states"].to_pylist(), dtype=np.float32)
    recorded_actions = np.asarray(parquet_table["action"].to_pylist(), dtype=np.float32)
    timestamps = np.asarray(parquet_table["timestamp"].to_pylist(), dtype=np.float64)
    video_probe = cv2.VideoCapture(str(args.input_video))
    if not video_probe.isOpened():
        raise RuntimeError(f"Could not open input video: {args.input_video}")
    video_frame_count = int(video_probe.get(cv2.CAP_PROP_FRAME_COUNT))
    video_fps = float(video_probe.get(cv2.CAP_PROP_FPS))
    video_width = int(video_probe.get(cv2.CAP_PROP_FRAME_WIDTH))
    video_height = int(video_probe.get(cv2.CAP_PROP_FRAME_HEIGHT))
    video_probe.release()
    validate_episode_contract(
        states=recorded_states,
        actions=recorded_actions,
        video_frame_count=video_frame_count,
        num_inference_steps=args.num_inference_steps,
    )
    if not np.isclose(video_fps, 30.0, atol=0.01):
        raise ValueError(f"Expected a 30 Hz SONIC video, got {video_fps}")
    prepare_output_directory(args.output_dir)

    from psi.config.config import LaunchConfig
    from psi.config.data_lerobot import LerobotDataConfig
    from psi.models.psi0 import Psi0Model
    from psi.utils import parse_args_to_tyro_config, seed_everything

    dynamic_config: LaunchConfig = parse_args_to_tyro_config(args.run_dir / "argv.txt")
    launch_config = dynamic_config.model_validate_json(
        (args.run_dir / "run_config.json").read_text()
    )
    seed_everything(launch_config.seed or 42)
    device = f"cuda:{args.gpu}"
    model = Psi0Model.from_pretrained(
        args.run_dir,
        args.checkpoint_step,
        launch_config,
        device=device,
    )
    model.to(device)
    model.eval()

    data_config: LerobotDataConfig = launch_config.data  # type: ignore[assignment]
    action_state_transform = data_config.transform.field
    dataset = data_config(
        split="train",
        transform_kwargs={"vlm_processor": model.vlm_processor},
    )
    total_episodes = dataset.raw_dataset.meta.total_episodes
    if not 0 <= args.episode_index < total_episodes:
        raise IndexError(
            f"Episode {args.episode_index} is outside [0, {total_episodes})"
        )
    start_frame = int(
        dataset.raw_dataset.base_dataset.episode_data_index["from"][
            args.episode_index
        ].item()
    )
    end_frame = int(
        dataset.raw_dataset.base_dataset.episode_data_index["to"][
            args.episode_index
        ].item()
    )
    episode_length = end_frame - start_frame
    if episode_length != recorded_states.shape[0]:
        raise ValueError(
            "Configured training dataset episode length does not match the supplied "
            f"parquet/video: {episode_length} != {recorded_states.shape[0]}"
        )

    predicted_chunks: list[np.ndarray] = []
    target_chunks: list[np.ndarray] = []
    latencies_ms: list[float] = []
    instructions: set[str] = set()
    for global_frame_index in tqdm(
        range(start_frame, end_frame),
        desc="Forwarding recorded ego frames",
        unit="frame",
    ):
        frame = dataset[global_frame_index]
        instructions.add(str(frame["instruction"]))
        batch_images = [frame["raw_images"]]
        batch_states = torch.from_numpy(frame["states"]).unsqueeze(0).to(device)
        torch.cuda.synchronize(args.gpu)
        inference_start = time.perf_counter()
        with torch.inference_mode():
            normalized_prediction = model.predict_action(
                observations=batch_images,
                states=batch_states,
                instructions=[frame["instruction"]],
                num_inference_steps=args.num_inference_steps,
                traj2ds=None,
            )
        torch.cuda.synchronize(args.gpu)
        latencies_ms.append((time.perf_counter() - inference_start) * 1000.0)
        prediction = action_state_transform.denormalize(normalized_prediction)
        predicted_chunks.append(prediction[0].float().cpu().numpy())
        target_chunks.append(np.asarray(frame["raw_actions"], dtype=np.float32))

    prediction_array = np.stack(predicted_chunks).astype(np.float32)
    target_array = np.stack(target_chunks).astype(np.float32)
    latency_array = np.asarray(latencies_ms, dtype=np.float32)
    local_frame_indices = np.arange(episode_length, dtype=np.int64)
    valid_mask = valid_horizon_mask(
        local_frame_indices,
        episode_length=episode_length,
        horizon=prediction_array.shape[1],
    )
    summary = summarize_predictions(
        prediction_array,
        target_array,
        valid_mask,
        latency_array,
    )
    metrics = {
        "evaluation": "Psi0 dense dataset-video open-loop evaluation",
        "open_loop": True,
        "observation_feedback": "recorded dataset observations only",
        "contract": {
            "sonic_release": "GEAR-SONIC v1.1",
            "dataset": "g1_pinch_real_001_clean_sonic_v1_1_psi0_30hz",
            "episode_index": args.episode_index,
            "episode_frames": episode_length,
            "video_fps": video_fps,
            "video_resolution": [video_width, video_height],
            "input_video": str(args.input_video),
            "episode_parquet": str(args.episode_parquet),
            "instructions": sorted(instructions),
            "state_dimension": int(recorded_states.shape[1]),
            "action_dimension": int(prediction_array.shape[2]),
            "action_groups": {
                "latent": 64,
                "left_hand": 7,
                "right_hand": 7,
            },
            "prediction_horizon": int(prediction_array.shape[1]),
            "checkpoint_step": args.checkpoint_step,
            "num_inference_steps": args.num_inference_steps,
            "h100_physical_gpu": 2,
            "container_cuda_device": args.gpu,
        },
        "metrics": summary,
    }
    np.savez_compressed(
        args.output_dir / "predictions.npz",
        frame_indices=local_frame_indices,
        timestamps=timestamps,
        predicted_actions=prediction_array,
        target_actions=target_array,
        valid_horizon_mask=valid_mask,
        inference_latency_ms=latency_array,
    )
    (args.output_dir / "metrics.json").write_text(
        json.dumps(metrics, indent=2, allow_nan=False) + "\n"
    )

    first_prediction = prediction_array[:, 0]
    first_target = target_array[:, 0]
    render_summary_image(
        output_image=args.output_dir / "first_step_comparison.png",
        prediction=first_prediction,
        target=first_target,
        latencies_ms=latency_array,
        fps=video_fps,
        checkpoint_step=args.checkpoint_step,
        episode_index=args.episode_index,
    )

    render_comparison_video(
        input_video=args.input_video,
        output_video=args.output_dir / "episode_000083_vla_open_loop.mp4",
        prediction=first_prediction,
        target=first_target,
        latencies_ms=latency_array,
        fps=video_fps,
    )
    print(json.dumps(metrics, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
