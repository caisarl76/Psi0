# Psi0 + SONIC Real-Robot Emergency Stop Design

**Status:** REVISED — BLOCKED pending independent approval and the prerequisites listed below
**Date:** 2026-08-26
**Scope:** Safe stop of the pinned GEAR-SONIC v1.1 controller before any Psi0 real-robot action publication

This revision withdraws the earlier approval claim. It is a design for review, not authorization to implement or actuate a real robot.

## Review Basis

The earlier design incorrectly treated entry into `G1Deploy::Stop()` as timely damping. Direct review of NVlabs GR00T-WholeBodyControl revision `c374bae5b9039cd0ee71377e654d11ce1bc69e1d` shows:

- `LowCommandWriter()` publishes the latest buffered body command at 500 Hz and also republishes the latest Dex3 hand commands.
- `Stop()` sets a plain `bool`, joins input, control, writer, and planner threads, and only then creates and publishes one body damping command.
- the ZMQ manager has stop-unaware planner-initialization waits lasting up to five seconds;
- policy and planner inference may also delay thread exit;
- `OperatorState::stop`, `start`, and `play` are plain booleans shared across threads; and
- `[DEBUG] Stopping G1Deploy...` is printed before `Stop()` and therefore proves only that shutdown was requested.

The existing `Stop` text proves that a host-side DDS `Write()` call returned. It does not prove that the G1 received or applied damping. Only the hardware E-stop is independent of the controller process, host, DDS path, and robot network.

## Safety Claims and Non-Claims

The design may claim only the following, with the stated evidence:

| Claim | Required evidence |
|---|---|
| shutdown requested | run-bound controller event |
| body/hand safe command latched | run-bound writer event after the latch becomes irreversible |
| safe command published by host | successful local DDS write plus exact serialized command fields |
| safe command observed in MuJoCo | independent `rt/lowcmd` and hand-topic subscribers see the required fields |
| controller exited gracefully | valid lifecycle history, required stop evidence for the environment, and pidfd-reported exit |
| robot applied damping | **not available from the current protocol** |

No log line, PID disappearance, DDS return value, camera observation, or MuJoCo result is described as actuator-side acknowledgement on real hardware.

## Goals

- Make every controller stop source latch a safe command in the 500 Hz writer before teardown waits.
- Ensure control, initialization, planner, and hand paths cannot overwrite or bypass the latch.
- Remove data races from shared lifecycle and operator state.
- Target the exact launched controller through a supervisor-owned Linux pidfd.
- Produce run-bound, structured evidence instead of searching unscoped logs.
- Verify the actual body and hand command contents and latency in MuJoCo.
- Fail closed across concurrent requests, startup races, abnormal exits, and stale state.
- Make the standard Psi0 real client launcher refuse to proceed without an exact, manually signed rehearsal receipt.

## Non-Goals

- This is not a replacement for the Unitree hardware E-stop.
- The software paths are not independent safety fault domains.
- This does not add shadow mode.
- This does not automatically use `SIGKILL`, `pkill`, `killall`, name matching, or process-group signalling.
- This does not claim robot-side damping acknowledgement.
- This does not approve real-robot testing while any blocking prerequisite remains.

## Safety Timing Contract

The relevant timestamps use `CLOCK_MONOTONIC_RAW` from the controller or the closest available monotonic clock:

- `t_request`: the first accepted stop request;
- `t_latched`: the irreversible writer latch is set;
- `t_body_publish`: the first complete body damping DDS write returns;
- `t_hand_publish`: both first hand-relax DDS writes return; and
- `t_sim_observed`: an independent MuJoCo-side observer receives all three safe commands.

The controller publishes at 500 Hz, so one writer period is 2 ms. The design target is two periods and the hard host-side ceiling is five periods:

- target `max(t_body_publish, t_hand_publish) - t_request <= 4 ms`;
- hard failure at `max(t_body_publish, t_hand_publish) - t_request > 10 ms`; and
- MuJoCo hard failure at `t_sim_observed - t_request > 20 ms`.

These are software containment limits: at most five 500 Hz periods before the host has published safe commands. They are not a robot braking-time claim. Before any real launch, the named robot safety owner must record an approved `max_request_to_host_publish_ms` no greater than 10 ms in the rehearsal policy. There is no permissive real-hardware default. If the hardware risk assessment requires a lower value, that lower value becomes the test threshold and receipt field.

The old 500 ms and five-second intervals are removed from the safety path. Process teardown may take longer, but safe-command publication must continue while teardown is blocked. A hung writer or DDS call is a software-stop failure requiring the hardware E-stop.

## Controller Safety Kernel

### Single synchronized shutdown state

Introduce one controller-owned `ShutdownCoordinator` with an atomic, monotonic phase and a first-writer-wins reason:

```text
STARTING -> RUNNING -> STOP_REQUESTED -> DAMPING_LATCHED -> TEARDOWN
```

No transition may move backward. `RequestShutdown(reason)` is idempotent and records every repeated request without changing the original `t_request` or reason. All current assignments to `operator_state.stop` are replaced by this API. The main, input, control, planner, and writer threads read the coordinator with acquire semantics.

`OperatorState::start` and `OperatorState::play` are also synchronized through atomic fields or a locked snapshot API. Merely making `stop` volatile is forbidden. Tests must run a thread sanitizer build of the coordinator and input/control interaction where supported.

### Signal handling before actuation

`sigaction` handlers for `SIGINT` and `SIGTERM` are installed before constructing `G1Deploy` or starting any controller thread. The handler performs only one async-signal-safe assignment to a `volatile std::sig_atomic_t` request flag.

The writer examines that flag before every publish and converts it into the same irreversible shutdown latch. The main loop also converts it through `RequestShutdown()` in normal context. This covers a signal that arrives while the constructor is loading models or after writer creation but before the constructor returns.

Startup semantics are explicit:

- signal before any publisher or actuation thread exists: cancel launch and record `damping_required=false`;
- signal after the publisher exists but before `RUNNING`: latch and publish safe commands before teardown;
- repeated signals: increment an audit counter but do not re-enter teardown; and
- signal during teardown: keep the latch set and continue publishing safe commands.

`Ctrl+C` is the terminal-generated `SIGINT` case and has its own acceptance test.

### Writer-level irreversible latch

The 500 Hz writer is the safety authority. It must not depend on `motor_command_buffer_` after shutdown is requested.

On each tick it:

1. acquires the publish gate;
2. observes the signal flag and atomic shutdown phase;
3. transitions to `DAMPING_LATCHED` if necessary;
4. constructs safe commands directly, rather than copying a possibly stale buffer;
5. rechecks the latch immediately before DDS publication; and
6. publishes only safe commands for the rest of the process lifetime.

The publish gate serializes a normal write already in progress with latch activation. At most one normal DDS write that began before `t_request` may complete afterward. No normal write may begin after `t_latched`.

The required body command covers all 29 motors:

```text
mode = enabled
tau = 0
q = 0
dq = 0
kp = 0
kd = 8
```

The required Dex3 command covers every motor on both hands and uses the existing timeout/relax semantics:

```text
timeout bit = 1
tau = 0
q = 0
dq = 0
kp = 0
kd = 0
```

Hand relaxation is constructed and published directly on the latched path; it must not pass through normal position clipping or smoothing.

### Preventing later overwrite

Initialization, policy inference, planner inference, and hand updates use `SetCommandIfRunning(...)`. That API checks the atomic phase before and while committing a buffer update. A command computed before the stop may be discarded afterward, but never committed as a live command after the latch.

The writer-level latch remains authoritative even if a producer check is missed. A long-running inference may finish, but its result cannot reach DDS.

### Stop-aware waits and teardown order

All polling waits in input and planner callbacks become predicate-based waits that return when shutdown is requested. The five-second ZMQ planner waits must not delay callback return. Blocking inference is checked before invocation and after return; it is not treated as interruptible.

Teardown order is:

1. latch safe body and hand publication;
2. keep the writer running at 500 Hz;
3. request and join input, control, and planner threads;
4. complete the environment-specific safe handoff; and
5. stop and join the writer last.

If another thread hangs, the writer continues safe publication. The supervisor reports teardown failure but does not kill the controller automatically.

For MuJoCo, the safe handoff is the independent observer confirmation described below. For real hardware, the current protocol has no actuator acknowledgement or documented command-source handoff. Therefore the controller must continue publishing damping until an operator confirms either the physical hardware E-stop or a separately validated Unitree damp-mode takeover. Real process exit without one of those confirmations is not approved by this design.

## Exact-Process Supervisor

### Ownership and launch

A long-lived Linux supervisor owns the controller process. It creates a new session/process group, then the child directly `execve()`s the resolved `target/release/g1_deploy_onnx_ref` binary. `just run`, shell wrappers, command substitution, and tmux foreground-process discovery are not part of process identity.

The preferred creation primitive is `clone3(CLONE_PIDFD)`, which returns the process handle atomically. A compatibility path may use `fork()` only when the child blocks on a private start-gate pipe until the parent has successfully opened and validated a pidfd; the parent releases the child to `setsid()` and `execve()` only afterward. Failure to obtain the pidfd cancels launch before exec. The supervisor retains that descriptor until terminal state.

Stop clients communicate with the supervisor over a run-specific Unix-domain socket and provide the run UUID. Only the supervisor calls `pidfd_send_signal()` or polls for exact process exit. A serialized PID is never used as a signal handle.

Tmux may display the supervisor and inherited controller terminal, but it is not an authority. The immutable pane ID is recorded for operator navigation; automated stop never uses `tmux send-keys`.

### Immutable run identity

Before actuation the supervisor records:

- schema version and cryptographically random run UUID;
- environment (`sim` or `real`);
- supervisor PID, `/proc` start time, executable, SID, and PGID;
- controller PID, `/proc` start time, SID, PGID, and resolved executable;
- NUL-preserved argv encoded as an array and as base64 of `/proc/<pid>/cmdline`;
- NUL-preserved selected environment entries;
- parent ancestry with PID, start time, and executable for each entry;
- pidfd ownership status and supervisor socket path;
- immutable tmux session ID and pane ID, if used;
- exclusive log path, device/inode, and starting offset zero;
- controller, patch, executable, model, config, network-interface, and hardware hashes; and
- lifecycle state, revision, and monotonic sequence.

PID, start time, SID, PGID, executable, argv, and ancestry are re-read for audit, but the retained pidfd is the signal/exit authority. A pane mismatch is diagnostic, not permission to retarget another process.

## Structured Evidence

Each run gets a newly created `0700` directory and exclusive `0600` files. Reusing or appending to a previous run log is forbidden. The controller sends acknowledgements to the supervisor through a dedicated inherited file descriptor; lifecycle decisions do not grep stdout.

Every JSONL acknowledgement contains:

```text
schema, run_uuid, sequence, event, reason,
controller_monotonic_ns, supervisor_receive_monotonic_ns,
environment, command_digest, evidence_scope
```

Required events include:

- `controller_starting`;
- `controller_running`;
- `shutdown_requested`;
- `damping_latched`;
- `body_damping_published_host`;
- `left_hand_relax_published_host`;
- `right_hand_relax_published_host`;
- `damping_observed_sim` when applicable;
- `teardown_started`;
- `writer_stopped`; and
- `controller_exit_requested`.

Sequence numbers are strictly increasing per run. Run UUID, command digest, inode, and starting offset prevent old or rotated logs from satisfying a new run. `[DEBUG] Stopping G1Deploy...` remains human-readable only and means `shutdown_requested`, never damping confirmation.

For each body publish event, the digest covers all serialized motor fields and CRC. Hand events cover all serialized hand fields. On real hardware the evidence scope is `host_dds_write_only`.

## Lifecycle and Idempotence

The supervisor is the single manifest writer. It uses an advisory lock plus write-to-new-file, `fsync`, atomic rename, and directory `fsync` for every transition.

Common states are:

```text
LAUNCHING -> RUNNING -> STOP_REQUESTED -> DAMPING_LATCHED
```

Simulation continues:

```text
DAMPING_LATCHED -> DAMPING_CONFIRMED -> EXITED_GRACEFUL
```

Real hardware continues only as far as the available evidence:

```text
DAMPING_LATCHED -> DAMPING_PUBLISHED_HOST -> HANDOFF_CONFIRMED_MANUAL
                 -> EXITED_GRACEFUL
```

Any invalid transition, identity loss, writer/DDS failure, deadline violation, unexpected process disappearance, missing acknowledgement, or exit before required handoff produces `EXITED_ABNORMAL` or `STOP_FAILED_ACTIVE` as appropriate.

Rules:

- the first stop caller performs the transition; concurrent callers attach to that run and receive the same outcome;
- only `EXITED_GRACEFUL` for the same run UUID is idempotent success;
- a missing, partial, stale, rotated, or identity-mismatched manifest is an error, not “already stopped”;
- startup cancellation before actuation may be graceful only when `damping_required=false` is proven;
- abnormal evidence is moved to an immutable archive;
- restart remains blocked until an explicit operator acknowledgement records identity, reason, and timestamp; and
- acknowledgement never rewrites an abnormal result as graceful.

`EXITED_GRACEFUL` also requires a `/proc` ancestry audit showing that no controller descendant from the recorded run remains. Discovery of a survivor is abnormal; it does not authorize a process-group signal.

## Operator Stop Interface

The one stop command:

1. locks and reads the current manifest;
2. verifies schema, run UUID, lifecycle, supervisor identity, socket ownership, and controller audit identity;
3. asks the supervisor to request shutdown through the retained pidfd;
4. waits for run-bound structured events, not text markers; and
5. reports the environment-specific result.

The manual `O` key calls the same controller `RequestShutdown("operator_o")` API. `SIGINT`, `SIGTERM`, Ctrl+C, internal safety faults, and supervisor requests converge on that API and writer latch.

There is no automated `O` injection, PID-based `kill()`, process-group kill, `SIGKILL`, or fallback target search. If the supervisor or pidfd is unavailable, the command fails closed and tells the operator to use the hardware E-stop. It must not improvise a process target.

In real mode, the command reports `DAMPING_PUBLISHED_HOST` distinctly and keeps the controller alive until manual handoff confirmation. It must not print “robot damped” or “safe” based only on host evidence.

## Psi0 Real-Client Gate

The refusing entrypoint is `real/scripts/deploy_psi0-sonic-rtc-client.sh`. It must not exec the client unless a rehearsal receipt is supplied and validates exactly.

The receipt binds:

- controller source revision and applied patch digests;
- built controller executable digest;
- supervisor/stop-tool digest;
- SONIC encoder, decoder, observation config, planner, and robot config digests;
- Psi0 client and checkpoint digests;
- network interface identity;
- robot serial/hardware identity and current boot ID;
- approved request-to-host-publish deadline;
- MuJoCo rehearsal run UUID and automated result;
- real stop-rehearsal run UUID and `HANDOFF_CONFIRMED_MANUAL` evidence;
- named operator and robot safety owner sign-off; and
- creation time and expiry.

This is an enforced check in the supported launcher and a manual operational sign-off, not a cryptographic safety boundary. Directly invoking Python can bypass it, so the design does not claim that the controller itself prevents every unauthorized VLA publisher. Documentation must state that limitation.

The current launcher directly starts `psi_rtc_sonic_client.py`, and that file is absent from the pinned official checkout. The client artifact, source revision, and digest are therefore a blocking prerequisite rather than an assumed dependency.

## Verification Design

### Unit and static tests

- all shared shutdown/start/play state uses atomics or locked snapshots;
- all stop-producing paths call `RequestShutdown()`;
- `sigaction` is installed before controller construction;
- handlers perform only the `sig_atomic_t` assignment;
- writer latch is monotonic and writer output dominates producer buffers;
- body damping and both hand-relax messages have exact required fields;
- no normal command can be committed or published after the latch;
- ZMQ planner waits return promptly when stop is requested;
- lifecycle transitions reject skips, regressions, partial data, and concurrent mutation;
- stop tooling contains no broad or PID-only signalling path; and
- the official real client launcher refuses absent, stale, mismatched, expired, or unsigned receipts.

### Process integration matrix

Exercise each request source in `STARTING`, `RUNNING`, and `TEARDOWN` where meaningful:

- manual `O`;
- terminal Ctrl+C;
- direct `SIGINT` through pidfd;
- direct `SIGTERM` through pidfd;
- repeated identical signals;
- mixed and concurrent stop calls; and
- internal controller safety failure.

Fault cases include:

- PID reuse and `/proc` start-time mismatch;
- tmux pane death, replacement, and foreground-process change;
- supervisor death and socket replacement;
- stale, rotated, truncated, and wrong-inode logs;
- missing, partial, corrupt, and old-schema manifests;
- controller exit between validation and stop request;
- hung input callback, planner initialization, policy inference, and planner inference;
- blocked or failed DDS publication; and
- unrelated processes with matching names and argv prefixes.

No fault case may signal a replacement or unrelated process. Abnormal state must survive restart attempts until acknowledged.

### MuJoCo acceptance

An independent subscriber on `rt/lowcmd`, `rt/dex3/left/cmd`, and `rt/dex3/right/cmd` records receipt timestamps and full payloads while active nonzero commands are being published.

For every request source and injected hang:

- body and both hands meet the configured deadline;
- the first observed body command has all 29 `tau=0`, `q=0`, `dq=0`, `kp=0`, and `kd=8`;
- both hands have the timeout bit set and all command fields zero;
- every later observed command through writer shutdown remains a safe command;
- injected input/control/planner hangs do not stop the damping writer;
- the structured run sequence is complete and correctly scoped; and
- pidfd observation proves exact controller exit.

Latency is measured over repeated loaded-host trials. Any single hard-ceiling miss fails the receipt; averages or percentiles cannot hide a miss.

### Real-hardware acceptance

Real testing remains blocked until all of the following exist:

- approved MuJoCo receipt for the exact artifacts;
- clear area, physical support, tested hardware E-stop, and dedicated E-stop operator;
- approved host-publish deadline from the robot safety owner;
- a documented and rehearsed safe command-source handoff; and
- manual sign-off acknowledging that host DDS publication is not actuator acknowledgement.

The rehearsal runs without Psi0 action publication. A hardware E-stop or separately validated Unitree damp-mode takeover must be confirmed before the controller writer exits. Only then may the manual receipt be created. The separately reviewed short Psi0 trial may use that receipt; this document does not approve the trial itself.

## Repository and Artifact Prerequisites

Commit `82df10c` is not self-contained. Its branch contains neither the GR00T-WBC gitlink nor the compatibility patch mechanism. The dirty primary workspace currently shows the intended inputs, but uncommitted workspace state is not a release artifact.

The prerequisite integration commit must contain and test, at minimum:

- submodule URL `https://github.com/NVlabs/GR00T-WholeBodyControl.git`;
- gitlink `c374bae5b9039cd0ee71377e654d11ce1bc69e1d`;
- existing ZMQ compatibility patch SHA-256 `34d20ee831999b08cdfc0e7215f6cbf8e8b0ac4ee0f5691afa994eb669229a06`;
- compatibility installer SHA-256 `234ec9536138516f7157341e9081dfa508a3388816a1a7bc1d794a0b752ddf92`; and
- the exact, present Psi0 RTC client source and its repository commit/digest.

Those hashes document the reviewed local candidates; they do not substitute for a committed prerequisite. The emergency-stop patch must be a later, separately reviewable artifact against that exact baseline. No implementation plan begins until the prerequisite commit and this revised design are independently approved.

## Acceptance Criteria for Design Approval

- The reviewer accepts the writer-level body and hand latch and timing semantics.
- The reviewer accepts that only the hardware E-stop is an independent fault domain.
- Signal installation, startup, repetition, teardown, and data-race handling are explicit.
- Stable pidfd ownership replaces PID/tmux targeting for all automated signals.
- Structured evidence is run-bound and distinguishes simulation observation from host-only real evidence.
- Lifecycle locking, concurrent calls, abnormal archival, restart blocking, and idempotence are explicit.
- The client-gate limitation is described as enforced launcher workflow plus manual sign-off, not actuator proof.
- The complete failure and signal matrix is part of acceptance testing.
- A prerequisite artifact commit makes the pinned controller, patch mechanism, and client source reproducible.
- No real-robot command or implementation planning starts before a new approval verdict.
