# Psi0 SONIC Prediction-to-MuJoCo Rollout Plan

**Goal:** Execute episode 83's saved dense Psi0 predictions through official GEAR-SONIC v1.1 in MuJoCo and add the resulting robot motion to the synchronized evaluation video.

**Architecture:** Preserve the recorded-input open-loop inference result, replay only its first-step 78-D predictions through SONIC protocol v4, select POSE mode before control start so the mode-reset race cannot clear the initial token, gate unhang and learned-action execution with filesystem markers, log MuJoCo qpos, render the physical result offline, and compose three synchronized panels.

**Tech stack:** Python 3.10, NumPy, PyArrow, pyzmq, OpenCV, MuJoCo, GEAR-SONIC v1.1 C++ deploy, pytest, Ruff.

---

### Task 1: Define prediction replay and deployment transforms

- [x] Add failing tests for NPZ shape validation, first-step extraction, FSQ quantization, hand-order mapping, and lifecycle ordering.
- [x] Verify the tests fail for missing replay helpers.
- [x] Implement the minimal pure helpers and verify the tests pass.

### Task 2: Implement the guarded VLA lifecycle replay

- [x] Add failing tests for mode selection without control start, supported initial-pose hold, unhang, and learned-action timing.
- [x] Implement a protocol-v4 replayer with explicit marker gates and timing report.
- [x] Extend the MuJoCo runner to release physics only after initial-pose completion, unhang and settle, then log the learned rollout.
- [x] Run targeted tests and static checks.

### Task 3: Render and compose the simulator result

- [x] Add failing tests for 30 Hz qpos sampling and three-panel frame synchronization.
- [x] Implement offline third-person MuJoCo rendering and three-panel composition.
- [x] Verify exact frame count, FPS, dimensions, and early/extra-frame rejection.

### Task 4: Execute GEAR-SONIC v1.1 MuJoCo rollout

- [x] Launch the simulator, v1.1 deploy binary with `zmq_manager`, and prediction replayer using isolated ports/processes.
- [x] Verify lifecycle events and 703 action publications; stop only the exact evaluation processes.
- [x] Compute physical-stability metrics and render the MuJoCo trajectory.
- [x] Compose and visually inspect the synchronized three-panel video.
- [x] Run the full targeted test/lint/compile verification suite and report artifact paths.
