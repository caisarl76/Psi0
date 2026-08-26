# Psi0 + SONIC Real-Robot Emergency Stop Design

**Status:** Approved design, pending implementation plan
**Date:** 2026-08-26
**Scope:** Safe termination of the local GEAR-SONIC deployment process before Psi0 real-robot testing

## Context

Psi0 real-robot inference depends on the official GEAR-SONIC v1.1 C++ controller. The controller already supports an `O`/`o` emergency-stop key. That path sets `operator_state.stop`, leaves the main loop, joins the control threads, and publishes one zero-torque, damping-only command before exit.

Raw process termination is not equivalent. The current `G1Deploy` destructor is empty and the non-ROS2 manager path has no `SIGINT` or `SIGTERM` handler. Abruptly terminating `g1_deploy_onnx_ref` can therefore bypass the controller's `Stop()` cleanup. `SIGKILL` can never be handled and must not be an automated software stop mechanism.

The validated workspace also contains an official NVlabs GR00T-WholeBodyControl submodule pinned to revision `c374bae5b9039cd0ee71377e654d11ce1bc69e1d` plus a version-specific ZMQ manager compatibility patch. The implementation must preserve that exact controller baseline rather than silently replacing it with another revision.

## Goals

- Provide one operator command that stops only the intended GEAR-SONIC deployment.
- Make `O`, `Ctrl+C`, `SIGINT`, and `SIGTERM` converge on the existing graceful `Stop()` path.
- Preserve the damping-only final command and orderly thread shutdown.
- Give the operator positive evidence that graceful shutdown completed.
- Prove the mechanism during active MuJoCo control before any real G1 command.
- Require a successful stop rehearsal on real hardware before enabling Psi0 action publication.

## Non-Goals

- This is not a replacement for the Unitree hardware E-stop.
- This does not add shadow mode or a runtime shadow/live toggle.
- This does not automatically use `SIGKILL`.
- This does not redesign SONIC control, Psi0 inference, action limits, or fall detection.
- This does not terminate unrelated simulator, camera, training, Docker, or SSH processes.

## Selected Approach

Use three independent layers:

1. **Native controller stop:** Keep `O`/`o` as the primary stop because it already reaches `G1Deploy::Stop()`.
2. **Signal bridge:** Patch the pinned controller so `SIGINT` and `SIGTERM` assign a `volatile std::sig_atomic_t` shutdown flag. The normal main loop observes the flag, sets `operator_state.stop`, and then calls `G1Deploy::Stop()` from ordinary program context.
3. **Exact-process supervisor:** Launch the controller in a dedicated tmux session and record its session name, PID, process start time, command identity, and log path. A stop script validates this identity before sending any input or signal.

The Unitree hardware E-stop remains the independent final layer for controller hangs, host failures, or network failures.

## Components

### Version-Pinned Controller Patch

Add a second patch alongside the existing ZMQ manager compatibility patch. The setup script applies both patches only when the GR00T-WBC submodule is at the validated revision.

The C++ patch will:

- install handlers for `SIGINT` and `SIGTERM`;
- have each handler perform only an assignment to `volatile std::sig_atomic_t`;
- check the flag in the existing main loop;
- set `operator_state.stop` from normal program context;
- preserve the single existing call to `custom.Stop()` and its damping command;
- log the received signal and the completion of graceful shutdown.

The signal handler will not call `Stop()`, DDS, CUDA, I/O, locks, or allocation directly.

### Dedicated Launch Supervisor

The real deployment launcher will run in a uniquely named tmux session. It will write a manifest under a runtime directory containing:

- schema/version;
- tmux session and pane;
- shell PID and resolved `g1_deploy_onnx_ref` PID;
- `/proc/<pid>/stat` start time to prevent PID-reuse mistakes;
- resolved executable path and command line;
- environment mode (`sim` or `real`);
- log path and launch timestamp.

The launcher must refuse to overwrite a manifest for a still-running matching process. A stale manifest may be replaced only after identity validation proves the old process is gone.

### Emergency-Stop Command

Provide a single top-level command for the operator. It follows this state machine:

1. Read the manifest and validate tmux session, PID, start time, executable, and command line.
2. Send `O` to the recorded tmux pane.
3. Wait up to 500 ms for the `[DEBUG] Stopping G1Deploy...` marker, which proves the main loop entered normal shutdown.
4. If that marker is absent, send `SIGINT` to the exact validated controller PID.
5. Wait up to another 500 ms for the same shutdown-entry marker.
6. If the marker is still absent, print a loud failure directing immediate use of the hardware E-stop. Return nonzero without automatically sending `SIGKILL`.
7. Once shutdown entry is confirmed, allow up to 5 seconds for the `Stop` completion marker and exact process exit. A timeout is an abnormal shutdown and again directs use of the hardware E-stop.

Repeated stop commands are idempotent: an already-stopped deployment reports that state successfully. Missing, malformed, stale, or mismatched manifests fail closed and never fall back to `pkill`, name matching, or broad process searches.

## Shutdown Contract

A software stop is successful only when all of the following are observed:

- the exact controller process has exited;
- the log contains the graceful shutdown sequence, including the `Stop` marker;
- no matching child controller process remains in the recorded process group;
- the stop command returns success and records the stop reason and timestamp.

Process disappearance without the graceful marker is reported as an abnormal termination, not a successful safety stop.

## Operator Workflow

### MuJoCo Rehearsal

1. Start MuJoCo and the SONIC v1.1 controller with the supervisor in `sim` mode.
2. Enable active control and release the robot as in the validated evaluation workflow.
3. Run the emergency-stop command from a separate terminal.
4. Confirm the damping shutdown marker, exact process exit, and simulator behavior.
5. Repeat once using the primary `O` path and once using the signal fallback path.

### Real-Hardware Gate

1. Confirm gantry/support, a clear 3 m zone, a tested hardware E-stop, and a dedicated operator at the stop terminal.
2. Start GEAR-SONIC v1.1 without starting the Psi0 client.
3. Enter the safe initialization/standing state only.
4. Run the emergency-stop command and confirm graceful damping shutdown.
5. Restart the controller only after this rehearsal passes.
6. Connect Psi0 and run the separately approved short, safety-clamped action trial.

The Psi0 client is never the only available stop mechanism. The emergency-stop terminal and hardware E-stop remain staffed throughout the trial.

## Error Handling

- **tmux unavailable:** refuse supervised real deployment.
- **Manifest identity mismatch:** refuse to signal any process and request operator inspection.
- **Graceful marker missing:** treat the stop as failed even if the PID disappears.
- **Signal fallback fails:** direct immediate use of the hardware E-stop; do not escalate automatically to `SIGKILL`.
- **Controller crashes:** record abnormal termination and prohibit automatic restart.
- **Compatibility patch mismatch:** refuse to patch or build against an unvalidated GR00T-WBC revision.

## Testing

### Static and Unit Tests

- The official submodule URL and pinned revision are exact.
- Both compatibility patches apply cleanly and idempotently to the pinned revision.
- Signal handlers do only flag assignment.
- The main loop routes signal requests through the normal `Stop()` path.
- Manifest parsing rejects missing fields, PID reuse, wrong executable paths, wrong command lines, and stale sessions.
- Stop targeting never uses unscoped `pkill`, `killall`, or name-only matching.
- Repeated stop requests are safe and idempotent.

### Integration Tests

- A harmless fixture process in tmux proves exact PID/start-time targeting.
- Unrelated similarly named processes remain alive.
- The `O` path exits gracefully and records its marker.
- Forced primary-path timeout exercises the `SIGINT` fallback and reaches the same marker.
- A deliberately mismatched manifest sends no signal.

### MuJoCo Acceptance Test

- Run the built SONIC v1.1 controller during active MuJoCo control.
- Exercise both `O` and `SIGINT` paths.
- Verify graceful shutdown logs and absence of a surviving controller process.
- Capture the command, log, manifest, timestamps, and outcome as evidence.

## Repository Integration

Implementation will be isolated from the dirty primary workspace. It will import only the directly required pending integration artifacts: the official GR00T-WBC submodule pin, the existing v1.1 compatibility patch/application mechanism, and matching SONIC v1.1 deployment defaults. Dataset, training, and unrelated local changes will not be included.

Expected top-level changes include:

- a version-pinned graceful-signal patch under `patches/gr00t-wholebodycontrol/`;
- an updated idempotent compatibility-patch installer;
- a supervised real/sim launch entrypoint;
- a dedicated emergency-stop command;
- tests for patching, process identity, stop escalation, and scope;
- operator documentation with exact rehearsal and real-hardware gates.

## Acceptance Criteria

- No real-robot actuation occurs before the MuJoCo stop rehearsal passes.
- `O`, `SIGINT`, and `SIGTERM` all reach the same damping-only `Stop()` path.
- One documented command can stop the exact supervised deployment from another terminal.
- The command never signals an unverified process and never automatically uses `SIGKILL`.
- The operator receives unambiguous success or failure output.
- A real-hardware stop rehearsal passes before Psi0 action publication is enabled.
