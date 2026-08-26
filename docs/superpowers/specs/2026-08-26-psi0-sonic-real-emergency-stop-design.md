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
- `[DEBUG] Stopping G1Deploy...` is printed before `Stop()` and therefore proves only that shutdown was requested;
- `main()` begins argument parsing before any signal handler is installed;
- body and Dex3 publishers are initialized before the command-writer thread exists; and
- the current `SetThreadPriority()` changes the constructor/main thread rather than the writer thread.

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

Every process uses `CLOCK_MONOTONIC_RAW`; there is no fallback clock. The supervisor and controller run on the same boot, so their values are directly comparable. The run manifest binds the Linux boot ID. Failure to read this clock blocks launch.

The source timestamp is defined before audit or shutdown work:

| Request source | Mandatory source timestamp |
|---|---|
| stop CLI | `t_cli_entry`, captured at process entry before reading files or connecting |
| outer-terminal Ctrl+C/SIGTERM | `t_terminal_entry`, captured in the foreground supervisor's signal callback |
| supervisor test/API signal | `t_signal_send`, captured immediately before `pidfd_send_signal()` |
| manual `O` | `t_o_callback`, captured at entry to the controller input callback that recognizes `O` |
| internal safety fault | `t_fault_detected`, captured at the branch that first detects the fault |

Additional timestamps are `t_supervisor_receive`, `t_signal_send`, `t_handler_entry`, `t_latched`, `t_body_publish`, `t_left_hand_publish`, `t_right_hand_publish`, and `t_sim_observed`. The signal handler captures `t_handler_entry` before setting its flag using `clock_gettime(CLOCK_MONOTONIC_RAW)` and Linux `sig_atomic_t` fields for seconds, nanoseconds, signal number, and a publish-last ready flag. Both stop signals are masked while either handler runs.

For a valid stop CLI request, the end-to-end interval begins at `t_cli_entry`, not when the supervisor accepts it. For terminal and controller-local sources it begins at the corresponding callback-entry timestamp. Decomposition into client-to-supervisor, signal-delivery, handler-to-latch, and latch-to-publish intervals is mandatory so scheduling or delivery delay cannot disappear from the measurement.

The supervisor performs only bounded, in-memory socket credential and run-UUID validation before signalling the retained pidfd. The `pidfd_send_signal()` call precedes manifest locks, log writes, `fsync`, hashing, `/proc` audit, and terminal output. Those audit operations occur afterward. A request that cannot reach and validate against the live supervisor has no software-stop success claim and directs immediate use of the hardware E-stop.

The nominal writer cadence is 500 Hz, but cadence is not a latency guarantee. The measured target is 4 ms and the measured hard ceiling is the safety-owner-approved `max_request_to_host_publish_ms`, which must be no greater than 10 ms. Both are measured from the applicable source timestamp to the last of the three host DDS writes. MuJoCo observation has a separately approved ceiling no greater than 20 ms. Loaded-host rehearsal, not multiplication of nominal writer periods, establishes whether the system meets these ceilings.

Before the first-actuation boundary, a stop cancels startup without publishing and must prevent that boundary from being crossed. After the boundary, the complete end-to-end damping deadline applies. The old 500 ms and five-second intervals are not part of the safety path. Process teardown may take longer, but safe-command publication continues while teardown is blocked. A hung writer or DDS call is a software-stop failure requiring the hardware E-stop.

Standard SIGINT and SIGTERM are not queued and may coalesce. The design records the first observed signal timestamp and a lower-bound count of handler invocations; it does not promise an audit record for every physical signal occurrence. Distinct stop-CLI requests remain individually auditable at the supervisor.

## Controller Safety Kernel

### Single synchronized shutdown state

Introduce one controller-owned `ShutdownCoordinator` with an atomic, monotonic phase and a first-writer-wins reason:

```text
PROCESS_ENTRY -> HANDLER_READY -> RESOURCES_LOADING -> SAFETY_WRITER_READY
              -> PUBLICATION_ARMED -> RUNNING

PROCESS_ENTRY/HANDLER_READY/RESOURCES_LOADING/SAFETY_WRITER_READY
              -> STARTUP_CANCELLED

PUBLICATION_ARMED/RUNNING -> STOP_REQUESTED -> DAMPING_LATCHED -> TEARDOWN
```

No transition may move backward. `RequestShutdown(reason, source_timestamp)` is idempotent and preserves the first source timestamp and reason. It may record only those repeated requests that the operating system actually delivers. All current assignments to `operator_state.stop` are replaced by this API. The main, input, control, planner, and writer threads read the coordinator with acquire semantics.

`OperatorState::start` and `OperatorState::play` are also synchronized through atomic fields or a locked snapshot API. Merely making `stop` volatile is forbidden. Tests must run a thread sanitizer build of the coordinator and input/control interaction where supported.

### Signal handling before actuation

The foreground supervisor installs its own handlers at supervisor process entry. It then temporarily blocks SIGINT and SIGTERM before creating the child. After the pidfd and cgroup placement are secured, the parent restores its mask immediately; only the child retains the blocked mask across `execve()`. At the first statements of controller `main()`, before logging, argument validation, allocation, CUDA/DDS initialization, or construction, the controller:

1. verifies that both signals are blocked;
2. installs `sigaction` handlers;
3. sends a fixed-size `HANDLER_READY` startup record over the inherited control socket; and
4. unblocks the signals only after the supervisor acknowledges that record.

Failure at any step exits before actuation. The handler captures the first callback-entry timestamp and signal number in preallocated signal-safe fields, then sets a `volatile std::sig_atomic_t` request flag. It performs no allocation, DDS, logging, locking, or lifecycle persistence.

Argument parsing and model/resource loading occur only after `HANDLER_READY`. Body and Dex3 publishers may be created during `RESOURCES_LOADING`, but no body or hand `Write()` is permitted. The safety writer is then created in an unarmed state before input, control, or planner producer threads. It emits `SAFETY_WRITER_READY` only after its real-time configuration, preallocated safe messages, and DDS ownership are verified.

The supervisor grants a one-shot arm token only when no stop is pending. The writer consumes that token, rechecks the signal and shutdown state, and atomically enters `PUBLICATION_ARMED`. This transition is the exact first-actuation boundary: no body or hand DDS write may occur before it, and the safety writer must already be runnable when it occurs. Producer threads start only after the boundary.

A signal or stop request observed before `PUBLICATION_ARMED` transitions to `STARTUP_CANCELLED`, records `damping_required=false`, and makes the arm transition impossible. A request at or after the boundary latches damping. A signal during teardown leaves the latch set. Coalesced signals do not re-enter teardown.

### Writer-level irreversible latch

The 500 Hz writer is the sole owner of body and hand DDS publication and the safety authority. `Stop()`, input, control, planner, evidence, and supervisor code may not call those DDS `Write()` methods. The writer must not depend on `motor_command_buffer_` after shutdown is requested.

On each tick it:

1. observes the signal flag and atomic shutdown phase;
2. transitions to `DAMPING_LATCHED` if necessary;
3. selects preconstructed safe commands instead of producer buffers;
4. rechecks the latch immediately before each DDS publication; and
5. publishes only safe commands for the rest of the process lifetime.

There is no cross-thread publish mutex: the writer alone owns publication. Producer buffers use a separate lock-free snapshot or bounded synchronization path that cannot be held by the writer while calling DDS. At most one normal DDS write that began before the source event may complete afterward. No normal write may begin after `t_latched`.

The required body command covers all 29 motors:

```text
mode = 1 (enabled)
tau = 0
q = 0
dq = 0
kp = 0
kd = 8
```

The required Dex3 command covers motor IDs 0 through 6 on both hand topics and exactly matches the pinned helper's status/timeout encoding. For motor ID `i`, the byte is:

```text
mode = (i & 0x0f) | (0x01 << 4) | (0x01 << 7) = 0x90 | i
motor ID = i
status = 0x01
timeout = 0x01
tau = 0
q = 0
dq = 0
kp = 0
kd = 0
```

Hand relaxation is constructed and published directly on the latched path; it must not pass through normal position clipping or smoothing.

### Writer real-time envelope

Real launch requires the actual writer thread—not the constructor thread—to pass all of these checks:

- `SCHED_FIFO` at the explicit numeric `writer_rt_priority` from the signed host safety profile, higher than all controller producer/evidence threads and validated against DDS/kernel thread priorities;
- affinity to a configured isolated CPU that is not used by input, control, planner, CUDA inference, evidence, or supervisor persistence;
- `mlockall(MCL_CURRENT | MCL_FUTURE)` plus prefaulted writer stack and message buffers;
- preconstructed body and both hand safe messages, with no allocation or formatting on the writer path;
- sole ownership of body and hand DDS writers, with no cross-thread mutex acquisition in the publish loop; and
- documented cpuset, IRQ placement, and competing real-time threads in the rehearsal receipt.

There is no default real-hardware priority. The host safety profile records the numeric value and the complete priority ordering so the writer cannot starve required DDS/kernel work. Failure to set or read back scheduling policy, priority, affinity, memory locking, or resource isolation blocks `PUBLICATION_ARMED` in real mode. Simulation may run without privileges only when explicitly labelled non-qualifying; it cannot produce a real-hardware receipt.

The 10 ms ceiling is a measured acceptance limit under CPU, memory, DDS, network, logging, and inference stress. The 2 ms nominal period explains the desired cadence but is not cited as proof of the bound. DDS `Write()` can still block; such a miss is a failed software stop and invokes the hardware-E-stop procedure.

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

A long-lived Linux supervisor owns the controller process. The child creates a new session/process group, then directly `execve()`s the resolved `target/release/g1_deploy_onnx_ref` binary. `just run`, shell wrappers, command substitution, and tmux foreground-process discovery are not part of process identity.

The preferred creation primitive is `clone3(CLONE_PIDFD | CLONE_INTO_CGROUP)`, which atomically returns the process handle and places the child in a per-run cgroup v2. A compatibility path may use `fork()` only when the child blocks on a private start-gate pipe until the parent has opened and validated a pidfd and placed the still-blocked child in the cgroup; the parent releases the child to `setsid()` and `execve()` only afterward. Failure to obtain either containment mechanism cancels real launch before exec. The supervisor retains the pidfd and cgroup directory until terminal state.

Stop clients communicate with the supervisor over a run-specific Unix-domain socket and provide the run UUID. Only the supervisor calls `pidfd_send_signal()` or polls for exact process exit. A serialized PID is never used as a signal handle.

### Foreground terminal topology

The supervisor remains the foreground process of the outer tmux PTY. The detached controller owns a nested slave PTY used for its keyboard input. The supervisor relays ordinary input bytes to that PTY but does not treat the pane foreground PID as controller identity.

Consequently, outer-terminal Ctrl+C reaches the supervisor, not the detached controller. The supervisor signal callback captures `t_terminal_entry`, suppresses default supervisor termination, and sets a preallocated dispatch flag. The safety-dispatch path immediately forwards SIGINT through the retained pidfd before any audit persistence. Outer-terminal SIGTERM follows the same rule. The supervisor remains alive through controller handoff/exit, and the controller's process-entry handler records its own delivery timestamp.

The supervisor safety-dispatch thread is separate from logging and manifest persistence. In real mode it uses a signed-profile `SCHED_FIFO` priority lower than the writer but higher than ordinary supervisor work, a dedicated non-writer CPU, locked/prefaulted memory, preallocated request slots, and no filesystem operations. Failure to configure or read back this envelope prevents the supervisor from granting the arm token.

Manual `O` is an ordinary byte relayed to the controller input callback and remains dependent on that callback being responsive. It is never synthesized as an automated fallback. A hung input callback is stopped through the supervisor signal path or hardware E-stop, not `O`.

Tmux is presentation only. The immutable session/pane IDs and the nested PTY identity are recorded for topology tests and operator navigation; automated stop never uses `tmux send-keys`.

### Immutable run identity

Before actuation the supervisor records:

- schema version and cryptographically random run UUID;
- environment (`sim` or `real`);
- supervisor PID, `/proc` start time, executable, SID, and PGID;
- controller PID, `/proc` start time, SID, PGID, and resolved executable;
- NUL-preserved argv encoded as an array and as base64 of `/proc/<pid>/cmdline`;
- NUL-preserved selected environment entries;
- parent ancestry with PID, start time, and executable for each entry;
- pidfd ownership status, per-run cgroup path, and supervisor socket path;
- outer tmux PTY and nested controller PTY device/inode and foreground PGIDs;
- immutable tmux session ID and pane ID, if used;
- exclusive log path, device/inode, and starting offset zero;
- controller, patch, executable, model, config, network-interface, and hardware hashes; and
- lifecycle state, revision, and monotonic sequence.

PID, start time, SID, PGID, executable, argv, and ancestry are re-read for audit, but the retained pidfd is the controller signal/exit authority. The per-run cgroup is the descendant-membership authority. Its ownership prevents the controller from migrating descendants out of it. A pane or PTY mismatch is diagnostic, not permission to retarget another process.

## Structured Evidence

Each run gets a newly created `0700` directory and exclusive `0600` files. Reusing or appending to a previous run log is forbidden. Lifecycle decisions do not grep stdout.

The controller/supervisor channel is a bounded `SOCK_SEQPACKET` socket inherited across exec. Startup-gate messages (`HANDLER_READY` and `SAFETY_WRITER_READY`) are fixed-size, preallocated records sent with `MSG_DONTWAIT | MSG_NOSIGNAL`. If either cannot be delivered and acknowledged, launch exits before `PUBLICATION_ARMED`.

After arming, safety threads never perform evidence I/O. They write fixed-size records to per-producer preallocated, bounded rings whose `try_push` operation is wait-free; queue full sets an atomic `evidence_lost` bit and immediately returns. A separate lower-priority evidence thread drains the rings, adds JSON serialization, and sends with `MSG_DONTWAIT | MSG_NOSIGNAL`. It handles `EAGAIN`, `EPIPE`, socket closure, and supervisor death without blocking, allocating in, or terminating the writer. SIGPIPE is blocked or ignored for the process.

Evidence loss makes the run abnormal and prevents a receipt, but it never delays or disables safe DDS publication. Loss of the supervisor socket after arming is itself an internal stop source: it atomically requests shutdown, while the writer continues damping without attempting socket I/O.

Every JSONL acknowledgement contains:

```text
schema, run_uuid, sequence, event, reason, source_kind,
source_monotonic_raw_ns, controller_monotonic_raw_ns,
supervisor_receive_monotonic_raw_ns,
environment, command_digest, evidence_scope
```

Required events include:

- `controller_starting`;
- `handler_ready`;
- `safety_writer_ready`;
- `publication_armed` or `startup_cancelled`;
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

Sequence numbers are strictly increasing per run. A missing sequence or `evidence_lost` is abnormal rather than silently repaired. Run UUID, command digest, inode, and starting offset prevent old or rotated logs from satisfying a new run. `[DEBUG] Stopping G1Deploy...` remains human-readable only and means `shutdown_requested`, never damping confirmation.

For each body publish event, the digest covers all serialized motor fields and CRC. Hand events cover all serialized hand fields. On real hardware the evidence scope is `host_dds_write_only`.

The supervisor's safety-dispatch path is in-memory and non-persistent. For stop CLI, Ctrl+C, SIGTERM, or supervisor API requests, it sends the pidfd signal first. Only afterward may the supervisor acquire the manifest lock, append logs, call `fsync`, inspect `/proc`, hash artifacts, or print status. Manifest durability is audit work, not a prerequisite for damping.

## Lifecycle and Idempotence

The supervisor is the single manifest writer. It uses an advisory lock plus write-to-new-file, `fsync`, atomic rename, and directory `fsync` for every transition.

Common states are:

```text
LAUNCHING_SIGNALS_BLOCKED -> HANDLER_READY -> RESOURCES_LOADING
                           -> SAFETY_WRITER_READY -> PUBLICATION_ARMED
                           -> RUNNING -> STOP_REQUESTED -> DAMPING_LATCHED

LAUNCHING_SIGNALS_BLOCKED/HANDLER_READY/RESOURCES_LOADING/SAFETY_WRITER_READY
                           -> STARTUP_CANCELLED -> EXITED_GRACEFUL
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

`DAMPING_PUBLISHED_HOST` requires successful host writes for the body and both hand topics. `HANDOFF_CONFIRMED_MANUAL` must name the handoff mechanism and cover all three command sources.

Any invalid transition, identity loss, writer/DDS failure, deadline violation, unexpected process disappearance, missing acknowledgement, or exit before required handoff produces `EXITED_ABNORMAL` or `STOP_FAILED_ACTIVE` as appropriate.

Rules:

- the first stop caller performs the transition; concurrent callers attach to that run and receive the same outcome;
- only `EXITED_GRACEFUL` for the same run UUID is idempotent success;
- a missing, partial, stale, rotated, or identity-mismatched manifest is an error, not “already stopped”;
- startup cancellation before actuation may be graceful only when `damping_required=false` is proven;
- abnormal evidence is moved to an immutable archive;
- restart remains blocked until an explicit operator acknowledgement records identity, reason, and timestamp; and
- acknowledgement never rewrites an abnormal result as graceful.

`EXITED_GRACEFUL` also requires the protected per-run cgroup to be empty. Reparenting does not remove a descendant from that cgroup, so this check does not depend on reconstructing `/proc` ancestry after exit. Discovery of a survivor is abnormal; it does not authorize `cgroup.kill`, a process-group signal, or PID-only signalling. Real launch is refused if the supervisor cannot create and protect the per-run cgroup.

## Operator Stop Interface

The one stop command:

1. captures `t_cli_entry` before file or socket work;
2. reads the atomic current-run locator without taking the manifest persistence lock;
3. connects to the protected run socket and submits the run UUID plus timestamp;
4. lets the supervisor perform bounded in-memory peer/run validation and immediately signal the retained pidfd;
5. performs manifest, `/proc`, log, and identity audit only after dispatch; and
6. waits for run-bound structured events and reports the environment-specific result.

The manual `O` callback captures its entry timestamp and calls the same controller `RequestShutdown("operator_o", t_o_callback)` API. `SIGINT`, `SIGTERM`, Ctrl+C forwarding, internal safety faults, and supervisor requests converge on the same writer latch with their own source timestamps.

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
- approved source-to-host-publish and MuJoCo-observation deadlines;
- writer and supervisor-dispatch scheduling/priority, CPU affinity, memory-locking, cpuset/IRQ isolation, and stress profile;
- per-run cgroup and outer/nested PTY topology results;
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
- SIGINT/SIGTERM remain blocked across exec until `sigaction` is installed at the first statements of `main()` and `HANDLER_READY` is acknowledged;
- handlers use only preallocated signal-safe timestamp/signal fields and the `sig_atomic_t` assignment;
- no body or hand DDS write occurs before `PUBLICATION_ARMED` and the safety writer is already ready at that transition;
- writer latch is monotonic and writer output dominates producer buffers;
- body damping and both hand-relax messages have exact required fields;
- hand mode bytes are exactly `0x90 | motor_id` for IDs 0 through 6 on both topics;
- no normal command can be committed or published after the latch;
- the writer thread passes scheduling, affinity, memory-locking, allocation, lock-ownership, and isolation checks;
- evidence queue saturation, EPIPE, SIGPIPE, and supervisor death cannot block or terminate the writer;
- ZMQ planner waits return promptly when stop is requested;
- lifecycle transitions reject skips, regressions, partial data, and concurrent mutation;
- per-run cgroup membership survives child reparenting and must be empty for graceful exit;
- stop tooling contains no broad or PID-only signalling path; and
- the official real client launcher refuses absent, stale, mismatched, expired, or unsigned receipts.

### Process integration matrix

Startup injection tests deliver SIGINT, SIGTERM, Ctrl+C, and stop-CLI requests before exec, before `HANDLER_READY`, during resource loading, after publisher creation, at `SAFETY_WRITER_READY`, and across the arm transition. Every pre-boundary case must end in `STARTUP_CANCELLED` with zero body and hand DDS writes. Every post-boundary case must meet the damping deadline.

The running/teardown matrix uses only meaningful source/fault pairs:

| Source | Required fault combinations |
|---|---|
| manual `O` | normal, control hang, planner hang, policy/planner inference hang while input remains responsive |
| terminal Ctrl+C | normal and every input/control/planner/inference hang |
| stop CLI / supervisor API | normal and every input/control/planner/inference hang |
| pidfd SIGINT and SIGTERM | normal and every input/control/planner/inference hang |
| internal fault | each actual fault-detection site, without inventing an unreachable same-thread combination |
| supervisor socket loss/death | normal and active inference/control hangs |

An input-thread hang is intentionally not paired with manual `O`, because that callback cannot observe the key. The expected alternatives are the supervisor signal path and hardware E-stop. Repeated standard signals assert idempotence and a lower-bound observed count, not one audit event per physical signal. Mixed and concurrent stop-CLI requests must converge on one lifecycle result.

Fault cases include:

- PID reuse and `/proc` start-time mismatch;
- tmux pane death, replacement, and foreground-process change;
- outer/nested PTY foreground-group mismatch and Ctrl+C forwarding failure;
- supervisor death and socket replacement;
- stale, rotated, truncated, and wrong-inode logs;
- missing, partial, corrupt, and old-schema manifests;
- controller exit between validation and stop request;
- hung input callback, planner initialization, policy inference, and planner inference;
- blocked or failed DDS publication;
- evidence queue saturation, socket backpressure, EPIPE, and log/manifest `fsync` delay;
- descendant fork, reparent, and attempted cgroup escape; and
- unrelated processes with matching names and argv prefixes.

No fault case may signal a replacement or unrelated process. Abnormal state must survive restart attempts until acknowledged.

### MuJoCo acceptance

An independent subscriber on `rt/lowcmd`, `rt/dex3/left/cmd`, and `rt/dex3/right/cmd` records receipt timestamps and full payloads while active nonzero commands are being published.

For every applicable source/fault pair:

- end-to-end, signal-delivery, handler-to-latch, and latch-to-publish intervals use `CLOCK_MONOTONIC_RAW` and meet the configured deadline;
- the first observed body command has all 29 `tau=0`, `q=0`, `dq=0`, `kp=0`, and `kd=8`;
- both hand topics contain seven commands whose mode bytes are exactly `0x90 | motor_id`, with zero `tau`, `q`, `dq`, `kp`, and `kd`;
- every later observed command through writer shutdown remains a safe command;
- injected input/control/planner hangs do not stop the damping writer;
- evidence and manifest backpressure do not change the damping latency or payload;
- when evidence loss is not injected, the structured run sequence is complete and correctly scoped;
- when evidence loss is injected, the run becomes abnormal without changing damping latency or payload; and
- pidfd observation proves exact controller exit and the protected per-run cgroup is empty.

The PTY acceptance test proves that Ctrl+C on the outer tmux PTY timestamps and reaches the foreground supervisor, is forwarded exactly through the controller pidfd, reaches the process-entry-installed handler, and latches damping. It separately proves that ordinary bytes, including `O`, traverse the nested PTY and that pane foreground-process changes cannot retarget the stop.

Latency is measured over repeated loaded-host trials with CPU, memory, DDS/network, logging, and inference stress. Any single hard-ceiling miss fails the receipt; averages or percentiles cannot hide a miss.

### Real-hardware acceptance

Real testing remains blocked until all of the following exist:

- approved MuJoCo receipt for the exact artifacts;
- clear area, physical support, tested hardware E-stop, and dedicated E-stop operator;
- approved host-publish deadline from the robot safety owner;
- verified writer scheduling, affinity, memory-locking, cgroup, PTY, and resource-isolation receipt fields;
- a documented and rehearsed safe command-source handoff; and
- manual sign-off acknowledging that host DDS publication is not actuator acknowledgement.

The rehearsal runs without Psi0 action publication. A hardware E-stop or separately validated Unitree damp-mode takeover must be confirmed for the body and both Dex3 hand topics before the controller writer exits. Only then may the manual receipt be created. The separately reviewed short Psi0 trial may use that receipt; this document does not approve the trial itself.

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
- The first-actuation boundary cannot be crossed before handler and safety-writer readiness.
- End-to-end timing begins at source/callback entry on mandatory `CLOCK_MONOTONIC_RAW`; safety dispatch precedes persistence.
- Stable pidfd ownership replaces PID/tmux targeting for all automated signals.
- Foreground-supervisor PTY forwarding makes Ctrl+C delivery explicit and tested.
- Structured evidence is nonblocking, run-bound, and distinguishes simulation observation from host-only real evidence.
- Writer scheduling and resource isolation are verified, and the 10 ms ceiling is described only as measured acceptance.
- Dex3 mode bytes and both hand-topic handoff semantics are exact.
- Lifecycle locking, concurrent calls, abnormal archival, restart blocking, and idempotence are explicit.
- Per-run cgroup membership replaces post-exit ancestry inference for descendant containment.
- The client-gate limitation is described as enforced launcher workflow plus manual sign-off, not actuator proof.
- The complete failure and signal matrix is part of acceptance testing.
- A prerequisite artifact commit makes the pinned controller, patch mechanism, and client source reproducible.
- No real-robot command or implementation planning starts before a new approval verdict.
