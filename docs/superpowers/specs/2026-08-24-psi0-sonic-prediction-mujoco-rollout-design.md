# Psi0 SONIC Prediction-to-MuJoCo Rollout Design

## Objective

Extend the episode-83 dataset-video open-loop evaluation with a physical MuJoCo visualization of the saved Psi0 action predictions. The recorded video and state remain the VLA inputs; MuJoCo is only an action-output consumer and never feeds observations back into Psi0.

## Control sequence

The documented GEAR-SONIC VLA workflow in `docs/source/tutorials/vla_inference.md` and `gear_sonic/scripts/run_vla_inference.py` is `PLANNER` start (`k`), initial pose / POSE switch (`i`), then inference unpause (`p`). A live diagnostic run showed that the current `ZMQManager` exposes `operator_state.start` to the 50 Hz control thread before the 10 Hz planner has produced `planner_motion`. The controller consequently enters the encoder with an empty motion and stops.

For this MuJoCo evaluation, use the same protocol-v4 initial-pose and inference semantics with a POSE-first safety sequence:

1. Initialize MuJoCo from episode 83's recorded robot state while elastic support is enabled and physics is frozen.
2. Select streamed-motion/POSE mode with `start=false`, `stop=false`, and `planner=false`. This lets the mode-switch safety reset finish without starting control.
3. Refill the post-reset pose buffer with the episode's first target action, then start control in POSE mode and hold that checkpoint-specific initial pose.
4. Release physics, lower support, apply MuJoCo key `9`, and wait 2.5 seconds unsupported while continuing to publish the initial pose.
5. Execute the saved Psi0 first-step prediction sequence at 30 Hz, equivalent to unpausing inference with keyboard `p`.

Every POSE action uses protocol v4: 64-D `token_state`, 7-D left hand, and 7-D right hand. Psi0's hand outputs are converted from training order to Unitree actuator order using `[4, 5, 6, 0, 1, 2, 3]`. Motion tokens are clipped and quantized to the SONIC FSQ grid `[-0.625, 0.625]` with step `1/16`.

## Components

- A prediction replay helper validates `predictions.npz`, builds the episode-specific initial action, separates POSE selection from control start, holds the initial pose, and streams 703 first-step predictions.
- A MuJoCo runner preserves the existing recorded-state initialization and elastic-band handling, but opens separate filesystem gates for controller readiness, initial-pose completion, support removal, and learned-action execution.
- An offline renderer samples the logged MuJoCo qpos trajectory at the 30 Hz action timestamps and renders a tracking third-person view.
- A compositor creates a synchronized 1920x480 video: recorded ego input, physical MuJoCo output, and prediction-versus-target diagnostics.

## Artifacts and acceptance

The rollout directory contains replay/deploy/simulator logs, lifecycle markers, qpos and event CSV files, replay timing JSON, physical-stability metrics, a 703-frame MuJoCo video, and a 703-frame three-panel result video. Acceptance requires explicit lifecycle events in order, 703 published learned actions, finite qpos, a standing robot through the rollout, exact video length/FPS, and visual inspection of representative start, middle, and final frames.
