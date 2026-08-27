# Psi0 + SONIC Real-Robot Emergency Stop Design

**Status:** APPROVED — DESIGN ONLY; implementation planning remains blocked by the repository and artifact prerequisites below
**Date:** 2026-08-26
**Approval recorded:** 2026-08-27, independent design review of `c801971d06355c24318a63a63aa79147f2a89dec`
**Scope:** Safe stop of the pinned GEAR-SONIC v1.1 controller before any Psi0 real-robot action publication

This approval covers the design only. It is not authorization to begin implementation planning, change the deployment host, or actuate a real robot while the documented prerequisites remain open.

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

## Normative Concurrency and Containment Basis

The C++ concurrency contract used here follows the [working draft's signal and memory-model rules](https://eel.is/c++draft/intro.multithread): a signal handler may execute on an unspecified thread, the special `volatile std::sig_atomic_t` allowance does not create general inter-thread publication, and release/acquire atomic operations synchronize published records. For that reason this design uses synchronous signal-wait threads and explicit atomic gates rather than asynchronous handler fields.

The containment contract follows the Linux kernel's [cgroup-v2 delegation and containment rules](https://www.kernel.org/doc/html/latest/admin-guide/cgroup-v2.html#delegation-containment): a non-root migration requires write access to the destination and common-ancestor `cgroup.procs`, while namespace containment also depends on reachability. The design therefore combines distinct credentials, inaccessible delegation files, a cgroup namespace, and a read-only restricted mount; no one of those controls is treated as sufficient alone.

The signal-authority contract follows [`pidfd_send_signal(2)`](https://man7.org/linux/man-pages/man2/pidfd_send_signal.2.html) and the [`kill(2)` permission rules](https://man7.org/linux/man-pages/man2/kill.2.html): a pidfd is a stable process reference, not an authorization token. The caller still needs matching credentials or `CAP_KILL` in the target's user namespace. Cross-process `/proc` inspection is treated separately because `/proc/<pid>/exe`, environment, and namespace reads are subject to [ptrace access checks](https://man7.org/linux/man-pages/man2/ptrace.2.html), including the specific rule for [`/proc/<pid>/exe`](https://man7.org/linux/man-pages/man5/proc_pid_exe.5.html).

The namespace-creation contract follows [`user_namespaces(7)`](https://man7.org/linux/man-pages/man7/user_namespaces.7.html) and [`unshare(2)`](https://man7.org/linux/man-pages/man2/unshare.2.html): creating a child user namespace grants the creator a full capability set in that child, and combining `CLONE_NEWUSER` with other namespace flags creates the user namespace first. An empty bounding set and `NoNewPrivs` do not prevent that grant. This design therefore prohibits controller-created or controller-joined namespaces with a pre-execution seccomp filter; it does not claim that ordinary capability dropping is sufficient. The filter contract follows [`seccomp(2)`](https://man7.org/linux/man-pages/man2/seccomp.2.html), including architecture/ABI checks and filter inheritance across `fork()`, `clone()`, and `execve()`.

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
- Target the exact launched controller through a supervisor-owned Linux pidfd and explicitly verified signal authority.
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
| outer-terminal Ctrl+C/SIGTERM | `t_terminal_wait_return`, captured immediately when the supervisor's dedicated `sigwaitinfo()` thread returns |
| supervisor test/API signal | `t_signal_send`, captured immediately before `pidfd_send_signal()` |
| manual `O` | `t_o_callback`, captured at entry to the controller input callback that recognizes `O` |
| internal safety fault | `t_fault_detected`, captured at the branch that first detects the fault |

Additional timestamps are `t_supervisor_receive`, `t_signal_send`, `t_controller_signal_wait_return`, `t_latched`, `t_body_publish`, `t_left_hand_publish`, `t_right_hand_publish`, and `t_sim_observed`. The supervisor and controller use dedicated synchronous signal-wait threads; there is no asynchronous handler-to-writer publication and no `volatile std::sig_atomic_t` timestamp protocol.

For a valid stop CLI request, the end-to-end interval begins at `t_cli_entry`, not when the supervisor accepts it. For terminal and controller-local sources it begins at the corresponding wait-return or callback-entry timestamp. Decomposition into client-to-supervisor, signal-send-to-wait-return, wait-return-to-latch, and latch-to-publish intervals is mandatory so scheduling or delivery delay cannot disappear from the measurement.

The supervisor performs only bounded, in-memory socket credential and run-UUID validation before signalling the retained pidfd. The `pidfd_send_signal()` call precedes manifest locks, log writes, `fsync`, pinned-descriptor verification, permitted cgroup inspection, and terminal output. Those audit operations occur afterward. A request that cannot reach and validate against the live supervisor, or for which `pidfd_send_signal()` returns `EPERM` or any other error, has no software-stop success claim and directs immediate use of the hardware E-stop.

The nominal writer cadence is 500 Hz, but cadence is not a latency guarantee. The measured target is 4 ms and the measured hard ceiling is the safety-owner-approved `max_request_to_host_publish_ms`, which must be no greater than 10 ms. Both are measured from the applicable source timestamp to the last of the three host DDS writes. MuJoCo observation has a separately approved ceiling no greater than 20 ms. Loaded-host rehearsal, not multiplication of nominal writer periods, establishes whether the system meets these ceilings.

A `RequestShutdown()` operation that linearizes before the first-actuation boundary cancels startup without publishing and prevents that boundary from being crossed. If `TryArm()` linearizes first, the complete end-to-end damping deadline applies, including when the request's source timestamp predates the boundary. The old 500 ms and five-second intervals are not part of the safety path. Process teardown may take longer, but safe-command publication continues while teardown is blocked. A hung writer or DDS call is a software-stop failure requiring the hardware E-stop.

Standard SIGINT and SIGTERM are not queued and may coalesce. The design records the first signal returned by `sigwaitinfo()` and a lower-bound count of returned signals; it does not promise an audit record for every physical signal occurrence. Distinct stop-CLI requests remain individually auditable at the supervisor.

## Controller Safety Kernel

### Single synchronized shutdown state and arm gate

Introduce one controller-owned `ShutdownCoordinator` with an authoritative activation gate, a synchronized monotonic reporting phase, and a first-writer-wins reason:

```text
PROCESS_ENTRY -> SIGNAL_WAITER_READY -> RESOURCES_LOADING -> SAFETY_WRITER_READY
              -> PUBLICATION_ARMED -> RUNNING

PROCESS_ENTRY/SIGNAL_WAITER_READY/RESOURCES_LOADING/SAFETY_WRITER_READY
              -> STARTUP_CANCELLED

PUBLICATION_ARMED/RUNNING -> STOP_REQUESTED -> DAMPING_LATCHED -> TEARDOWN
```

No transition may move backward. The activation decision is one packed, always-lock-free `std::atomic<uint64_t>` containing `PREARM`, `ARMED`, `STARTUP_CANCELLED`, or `STOP_REQUESTED` plus the winning preallocated request-slot index. The implementation must fail its build or real-mode startup if `std::atomic<uint64_t>::is_always_lock_free` and a runtime `is_lock_free()` check are not both true. The more detailed phase shown above is a separate monotonic atomic used for reporting; it may reflect but never override the activation gate. Each possible request producer has an exclusively owned, preallocated record. It writes the reason and source timestamp into that record, then publishes the record by a release compare-and-swap on the packed gate; readers use acquire loads before reading the winning record. No activation or lifecycle metadata is published through plain or volatile storage.

The only activation operations are linearizable:

```text
TryArm:           PREARM -> ARMED
RequestShutdown: PREARM -> STARTUP_CANCELLED(request_slot)
RequestShutdown: ARMED  -> STOP_REQUESTED(request_slot)
```

The successful compare-and-swap is the linearization point. Cancellation wins only if its `PREARM -> STARTUP_CANCELLED` operation precedes `TryArm` in the atomic modification order. Otherwise arming wins, `RequestShutdown` performs `ARMED -> STOP_REQUESTED`, and the complete post-arm damping deadline applies. Source or sender timestamps measure latency but do not order this race. Making a remote sender timestamp decide it would require a separately designed shared cross-process gate; this design intentionally does not make that claim.

`RequestShutdown(reason, source_timestamp)` is idempotent and preserves the winning record. It may record only repeated requests actually delivered by the operating system. All current assignments to `operator_state.stop` are replaced by this API. The main, input, control, planner, and writer threads read the coordinator with acquire semantics.

`OperatorState::start` and `OperatorState::play` are also synchronized through atomic fields or a locked snapshot API. Merely making `stop` volatile is forbidden. Tests must run a thread sanitizer build of the coordinator and input/control interaction where supported.

### Synchronous signal ownership before actuation

SIGINT and SIGTERM are synchronously owned; the controller installs no asynchronous stop handler. The bootstrap helper blocks both signals in the supervisor child before supervisor `execve()`. At the first statements of supervisor startup, before it creates any thread or requests any controller child, the supervisor verifies and reasserts that mask and reads back its final UID/GID, user namespace, `NoNewPrivs`, `PR_SET_DUMPABLE=0`, and `CAP_KILL`-only sets. Signals remain blocked and pending while it creates one dedicated `sigwaitinfo()` thread and proves it ready to the helper. Every other supervisor thread inherits and retains the blocked mask for the process lifetime. The helper may release a controller gate only after this readiness acknowledgement. The supervisor signal thread timestamps immediately when `sigwaitinfo()` returns and dispatches through the retained pidfd before audit work. A signal that terminates the helper or supervisor before the inherited mask is established is pre-controller and therefore pre-actuation.

The controller child inherits the blocked mask across `execveat()`. At the first statements of controller `main()`, before logging, argument validation, allocation, CUDA/DDS initialization, or construction, the controller:

1. verifies that SIGINT and SIGTERM are blocked and leaves them blocked in the main thread and every later thread;
2. reapplies and reads back `PR_SET_NO_NEW_PRIVS`, `PR_SET_DUMPABLE=0`, final UID/GID, and empty effective/permitted/inheritable/ambient/bounding capability sets;
3. verifies `PR_GET_SECCOMP == SECCOMP_MODE_FILTER` and that the inherited read-only, fully sealed profile memfd identifies the expected filter digest, kernel build, architecture, and ABI;
4. creates the sole controller signal thread, which verifies its inherited mask and enters the synchronous wait path;
5. completes a preinitialized `pthread_mutex_t`/`pthread_cond_t` release/acquire startup handshake proving that the waiter is in its synchronous wait loop; and
6. sends a fixed-size `SIGNAL_WAITER_READY` startup record over the inherited control socket.

Failure at any step exits before actuation. A signal already pending across `execveat()` remains pending until the waiter consumes it. On return from `sigwaitinfo()`, the signal thread captures `t_controller_signal_wait_return` with `CLOCK_MONOTONIC_RAW` and directly calls `RequestShutdown()`; it does not communicate through a flag. Its preallocated request slot and the release/acquire coordinator transition provide inter-thread publication. The signal thread performs no DDS, filesystem persistence, formatting, or lifecycle logging.

The controller blocks after sending `SIGNAL_WAITER_READY`. For disposable signal-authority children the supervisor deliberately sends the test signal without acknowledging startup. For the final child, the supervisor acknowledges the record only after it has received and checked the pinned evidence and observed `bootstrap_exited`. Argument parsing and model/resource loading occur only after that acknowledgement. Body and Dex3 publishers may be created during `RESOURCES_LOADING`, but no body or hand `Write()` is permitted. The safety writer is then created in an unarmed state before input, control, or planner producer threads. It emits `SAFETY_WRITER_READY` only after its real-time configuration, preallocated safe messages, and DDS ownership are verified.

The supervisor has an analogous packed, always-lock-free arm/stop atomic and exclusively owned preallocated request slots. `SupervisorTryAuthorizeArm()` and every stop ingress use release compare-and-swap with acquire readers. Stop-CLI handling and the signal-wait thread call `SupervisorRequestStop()` directly; there is no plain, volatile, or asynchronous dispatch flag. If stop wins the supervisor gate, the arm token is withheld. If arm authorization wins, every concurrent or later stop ingress still calls `pidfd_send_signal()` directly and idempotently, so a preempted first dispatcher cannot suppress delivery. This supervisor gate is fail-closed coordination, not the controller's first-actuation boundary.

`SupervisorTryAuthorizeArm()` must refuse unless containment, namespace-prohibition policy, and both signal-authority probes passed; the final controller pidfd and pinned evidence match; the final controller waiter/writer are ready; and the bootstrap-helper pidfd reports exit. The supervisor may send a one-shot arm token only after that operation succeeds. Receipt of that token is not the controller linearization point. The writer calls `TryArm()` on the single controller activation atomic. A stop producer racing with it calls `RequestShutdown()` on that same atomic, so exactly one of cancellation or arming wins. `TryArm()` success is the exact first-actuation boundary: no body or hand DDS write may occur before it, and the safety writer must already be runnable. Producer threads start only after it.

If cancellation wins, `STARTUP_CANCELLED` records `damping_required=false`, arming becomes impossible, and zero body/hand writes are required. If arming wins, later delivery of a request whose sender timestamp predates the boundary does not retroactively cancel arming: damping is required and the end-to-end deadline still begins at that earlier source timestamp. A signal during teardown leaves the latch set. Coalesced signals do not re-enter teardown.

### Writer-level irreversible latch

The 500 Hz writer is the sole owner of body and hand DDS publication and the safety authority. `Stop()`, input, control, planner, evidence, and supervisor code may not call those DDS `Write()` methods. The writer must not depend on `motor_command_buffer_` after shutdown is requested.

On each tick it:

1. acquires the atomic shutdown phase;
2. transitions to `DAMPING_LATCHED` if necessary;
3. selects preconstructed safe commands instead of producer buffers;
4. rechecks the latch immediately before each DDS publication; and
5. publishes only safe commands for the rest of the process lifetime.

There is no cross-thread publish mutex: the writer alone owns publication. Producer buffers use a separate lock-free snapshot or bounded synchronization path that cannot be held by the writer while calling DDS. The writer captures `t_latched` immediately after its successful `STOP_REQUESTED -> DAMPING_LATCHED` transition. Normal writes that passed their final check may begin or complete during source-to-latch or signal-delivery latency, and an already-started write may complete or be delivered after `t_latched`. The enforceable invariant is that no normal body or hand `Write()` invocation begins after the latch transition; all such invocations begun after `t_latched` use the safe messages.

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

The controller signal-wait thread has its own signed-profile `SCHED_FIFO` priority below the writer and above all ordinary controller work, locked/prefaulted memory, and affinity to the non-writer safety-dispatch CPU. Its measured signal-wait-return-to-latch contribution is part of the same hard ceiling. Failure to configure or read back this thread blocks real-mode arming.

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

A short-lived, root-owned, single-threaded bootstrap helper creates the security boundary and a long-lived least-privileged supervisor owns the controller thereafter. The helper blocks SIGINT/SIGTERM in the supervisor child before its `execve()`, creates the run-specific user namespace and cgroups, opens and hashes the controller executable, and launches gated controller children. The helper never signals a controller and cannot authorize publication. It passes a pidfd referring to itself to the supervisor, exits before the final arm decision, and must be observed exited through that pidfd.

The preferred creation primitive is `clone3(CLONE_PIDFD | CLONE_INTO_CGROUP)`, used by the helper before the controller seccomp filter exists to atomically obtain the process handle and place each child in its designated cgroup. A compatibility path may use `fork()` only when the child blocks on a private start-gate pipe until the helper has opened and validated a pidfd and placed the still-blocked child in the cgroup. The privileged child trampoline configures the private namespaces/mounts, clears inherited privileged descriptors, drops to the controller credentials and zero capabilities in the run and ancestor user namespaces, creates a new session/process group, sets `NoNewPrivs`, installs and verifies the immutable namespace-prohibition seccomp filter defined below, and waits at the gate. Gate closure causes `_exit()` before controller execution. The target is executed only with `execveat()` from the pinned executable descriptor; shells, path re-resolution, `just run`, command substitution, and tmux foreground-process discovery are excluded. The helper transfers each pidfd to the final-credential supervisor over a private `SOCK_SEQPACKET` socket with `SCM_RIGHTS`. Failure at any point cancels real launch before execution.

Stop clients communicate with the supervisor over a run-specific Unix-domain socket and provide the run UUID. Only the supervisor calls `pidfd_send_signal()` for controller processes or treats pidfd polling as safety exit evidence. The helper may use `waitid()` only to reap already-exited disposable children; it may not signal them or turn PID-based wait results into safety evidence. A serialized PID is never used as a signal handle.

### Signal authority, credentials, and cgroup-v2 containment

Directory ownership and pidfd possession alone are not authority claims. Real launch requires a unified cgroup-v2 hierarchy, a run-specific user namespace, and three distinct security identities:

- the bootstrap helper has exactly `CAP_SYS_ADMIN`, `CAP_SYS_PTRACE`, `CAP_CHOWN`, `CAP_SETUID`, `CAP_SETGID`, `CAP_SYS_RESOURCE`, and `CAP_SETPCAP` in its effective, permitted, and bounding sets in the initial user namespace, with no ambient or inheritable capabilities; `CAP_SYS_PTRACE` is used only for the pre-exit pinned inspection below, and the helper is absent before arming;
- the runtime supervisor's real/effective/saved/filesystem UID/GID values are all namespace ID 0, mapped one-to-one to dedicated host supervisor UID/GID values; it clears supplementary groups, sets `PR_SET_NO_NEW_PRIVS` and `PR_SET_DUMPABLE=0`, and retains only `CAP_KILL` in the run-specific user namespace; its effective, permitted, and bounding sets contain only `CAP_KILL`, and its ambient/inheritable sets are empty; and
- the controller's real/effective/saved/filesystem UID/GID values are all namespace ID 1, mapped one-to-one to different dedicated host controller UID/GID values; it clears supplementary groups, sets `PR_SET_NO_NEW_PRIVS` and `PR_SET_DUMPABLE=0`, and has empty effective, permitted, inheritable, ambient, and bounding capability sets.

The helper writes explicit two-entry UID/GID maps (`0 -> host supervisor`, `1 -> host controller`) and denies `setgroups` before the GID map. The supervisor and controller remain in the same host PID namespace, satisfying the pidfd caller/target PID-namespace relationship. The helper pins the run-user-namespace descriptor and its `NS_GET_PARENT` chain, and the supervisor proves after its final credential transition that its current user-namespace descriptor matches that pin. `CAP_KILL` is effective only in that run-specific user namespace: it authorizes the supervisor to signal the controller but grants no signal authority over processes in the parent/initial user namespace. The controller cannot become namespace UID 0 or gain a capability in the run or any ancestor user namespace because it has no set-ID capability, an empty bounding set, `NoNewPrivs`, no privileged executable, and no accessible ancestor-namespace descriptor. Preventing it from gaining capabilities in a newly created child user namespace is a separate seccomp obligation defined next.

The pinned supervisor and controller executables must have no set-user-ID/set-group-ID bits or file capabilities. Required controller access to GPU/device nodes, model/config files, and the DDS network interface is granted to the mapped host controller UID through explicit, recorded ACLs or ordinary file modes—not supplementary groups or capabilities. Missing or broader-than-approved access blocks launch.

### Irreversible namespace-creation prohibition

Real launch supports only the pinned normal x86-64 ABI and kernel build, with `CONFIG_SECCOMP_FILTER` and `CONFIG_CHECKPOINT_RESTORE` enabled. Before `execveat()`, while the controller trampoline is still single-threaded and after `PR_SET_NO_NEW_PRIVS`, it installs one classic seccomp-BPF filter in `SECCOMP_MODE_FILTER`. The filter is inherited by the controller and every later thread or child and cannot be weakened by a later filter. Its exact BPF bytecode and SHA-256 digest are build artifacts. It implements this contract:

- a `seccomp_data.arch` value other than `AUDIT_ARCH_X86_64`, or any syscall number carrying `__X32_SYSCALL_BIT`, returns `ENOSYS` without executing the syscall;
- `unshare()` and `setns()` return `EPERM` for every argument combination;
- legacy x86-64 `clone()` returns `EPERM` when its flags contain any of `CLONE_NEWUSER`, `CLONE_NEWNS`, `CLONE_NEWCGROUP`, `CLONE_NEWIPC`, `CLONE_NEWNET`, `CLONE_NEWPID`, `CLONE_NEWTIME`, or `CLONE_NEWUTS`, and otherwise remains available for pthreads, `fork()` wrappers, and ordinary descendants;
- `clone3()` always returns `ENOSYS`, because classic seccomp sees only the user pointer to `struct clone_args` and cannot safely validate the pointed-to flags; and
- `fork()` and `vfork()` remain available, but their children inherit the same filter and therefore cannot create or join namespaces.

The legacy `clone()` rule loads the low 32 bits of `seccomp_data.args[0]` and rejects a nonzero intersection with one compile-time `NAMESPACE_MASK` containing every flag listed above. The filter generator takes syscall numbers and flag values from the pinned kernel UAPI headers, asserts the expected x86-64 values at build time, and emits the normal-ABI syscall-table digest. No handwritten duplicate syscall-number table is accepted.

No seccomp rejection uses `SECCOMP_RET_KILL_PROCESS`; a prohibited call fails without directly killing the damping writer. The controller may install additional filters only if they are more restrictive; no runtime path may replace or remove the inherited filter. A new architecture, ABI, kernel build, namespace-creation syscall, glibc/CUDA runtime, or controller dependency invalidates the receipt and requires a syscall-table review plus a complete filtered startup and loaded-runtime rehearsal. Returning `ENOSYS` for `clone3()` is accepted only if every required runtime path demonstrably falls back to permitted non-namespace `clone()` or does not need the call.

The deployment-host safety profile additionally requires `/proc/sys/kernel/unprivileged_userns_clone` to exist and read `0`. That mutable host setting is defense in depth, not the safety authority: the inherited seccomp filter remains effective if a privileged host process later changes the sysctl. The root bootstrap helper proves it can still create the one approved run user namespace before any controller trampoline exists; the final supervisor has no capability to change the sysctl. At this review, the inspected workstation reads `1`, so it cannot produce a real-hardware receipt; this design revision does not change the host setting.

For every probe and controller child, the helper uses `PTRACE_SEIZE`/`PTRACE_INTERRUPT` while the single-threaded trampoline is still stopped at its pre-`execveat()` gate, dumps filter index 0 with `PTRACE_SECCOMP_GET_FILTER`, proves that exactly one filter exists and matches the pinned bytecode, then detaches with signal 0. After execution, it also verifies `PR_GET_SECCOMP == SECCOMP_MODE_FILTER` through the controller's startup record and reads the gated process's `Seccomp`/`Seccomp_filters` status fields before allowing resource loading. Failure, an extra filter, or unsupported readback blocks real launch. The controller emits no readiness event until its own process-entry check confirms seccomp mode 2. The helper records the filter, kernel build ID, normal-ABI syscall-table digest, glibc/CUDA versions, and host-policy readback in the immutable bootstrap record.

The host cgroup-v2 mount must have `nsdelegate`; absence blocks real launch. The delegation root is `0750`, owned by the mapped host supervisor UID/GID. Only that identity receives write access to the delegation directory and the kernel delegation files it needs, including `cgroup.procs`, `cgroup.threads`, and `cgroup.subtree_control`; the mapped host controller identity receives none. Each per-run cgroup is created by and remains owned by the supervisor identity with directory mode `0750`, and every writable cgroup interface remains inaccessible to the controller UID/GID. The numeric UIDs/GIDs, mount ID, cgroup filesystem magic, `nsdelegate` setting, delegation-root device/inode, per-run cgroup device/inode, ownership, and effective modes are recorded before arming.

The helper places the child in the run cgroup and creates a private cgroup and mount namespace rooted at that run cgroup. Inside the controller mount namespace, `/sys/fs/cgroup` exposes only the run hierarchy through a read-only, recursively bind-mounted cgroup-v2 view; the host hierarchy, ancestors, and sibling cgroups are not mounted. Mount propagation is private. The controller cannot rely on namespace visibility alone: before `execveat()` the trampoline closes every cgroup/mount-namespace descriptor not explicitly allowlisted, clears supplementary groups, sets `PR_SET_NO_NEW_PRIVS`, drops every effective, permitted, inheritable, ambient, and bounding capability, including `CAP_SYS_ADMIN`, `CAP_SETUID`, `CAP_SETGID`, and DAC-bypass capabilities, and installs the namespace-prohibition filter. Required writer real-time priority and memory locking are granted only through bounded `RLIMIT_RTPRIO` and `RLIMIT_MEMLOCK` values in the signed profile, not retained capabilities. Descendants inherit the same credentials, namespace boundaries, no-new-privileges setting, capability sets, seccomp filter, mount view, and cgroup membership.

These controls address both kernel migration conditions: the controller can write neither a destination `cgroup.procs` nor the common ancestor's `cgroup.procs`, and neither a destination outside the namespace nor a writable host cgroup hierarchy is reachable. `CAP_KILL` grants no cgroup or identity-changing authority. The supervisor is the only non-root runtime component permitted to migrate or enumerate the run through the host cgroup hierarchy.

Before `TryArm()`, the helper launches a fixed, hashed containment probe at the final-credential supervisor's request through the exact same child setup path and credentials. The probe and a descendant it forks must each fail to:

- write themselves or one another to the run root, a child, a sibling, or an ancestor `cgroup.procs`/`cgroup.threads`;
- create a child cgroup or make the cgroup mount writable;
- open a host/sibling cgroup path through `/proc`, inherited descriptors, or namespace escape; and
- use `setns()` or mount operations to obtain a different cgroup view.

The same probe exercises the installed seccomp policy directly:

- `unshare()` is called with every individual `CLONE_NEW*` flag above and with `CLONE_NEWUSER` combined with each other namespace flag; every call must return `EPERM`;
- legacy `clone()` is called with the same individual and combined namespace flags; every call must return `EPERM` and create no child;
- ordinary and namespace-bearing `clone3()` calls must both return `ENOSYS` and create no child;
- `setns()` with an invalid descriptor and against every self-openable current namespace descriptor must return `EPERM`, proving rejection occurs before descriptor validity or target permissions matter; and
- a permitted `fork()` and a permitted non-namespace `clone()` child must repeat the prohibited calls with the same results, proving filter inheritance.

The helper inspects both live probe PIDs while it still has bootstrap authority and pins the resulting evidence. The final-credential supervisor independently verifies their membership through its owned host cgroup hierarchy and verifies the pinned namespace, mount, and seccomp descriptors described below. Expected cgroup/mount failures are `EACCES`, `EPERM`, `EROFS`, or namespace `ENOENT`; seccomp calls must return the exact errors above. An unexpected success, wrong errno, created namespace/child, missing check, inherited writable cgroup descriptor, filter mismatch, or unverifiable result blocks the arm token. The actual controller is launched through the same immutable, hashed setup routine and filter.

Before the helper may launch the final controller, signal authority is tested from the supervisor's final runtime credentials:

1. the helper launches an exact controller executable in a disposable pre-arm run using the final controller UID/GID, user/PID namespaces, blocked signal mask, capabilities, `NoNewPrivs`, and seccomp-filter settings;
2. after `SIGNAL_WAITER_READY`, the runtime supervisor calls `pidfd_send_signal(pidfd, SIGINT, nullptr, 0)`, and the child must report the matching `sigwaitinfo()` signal plus supervisor `si_pid`/`si_uid`, transition to `STARTUP_CANCELLED`, perform zero DDS writes, and exit;
3. the test repeats with a fresh exact controller child and SIGTERM; and
4. only after both deliveries succeed does the helper launch the final controller from the same pinned executable and immutable setup routine, transfer its pidfd, and exit.

The signal calls in steps 2 and 3 must be made by the long-lived supervisor after it has the exact final UID/GID, user namespace, `CAP_KILL`-only capability sets, `NoNewPrivs`, scheduling, affinity, and memory-lock configuration. The second call is exactly `pidfd_send_signal(pidfd, SIGTERM, nullptr, 0)`. Runtime code permits only SIGINT and SIGTERM with `info=nullptr` and `flags=0`. A bootstrap-originated call, matching PID without pidfd, `EPERM`, wrong `si_pid`/`si_uid`, missing cancellation event, seccomp mismatch, or helper survival blocks arming. The preflight results bind the executable/configuration hashes, user/PID namespace inode, UID/GID maps, seccomp/kernel/ABI profile, boot ID, and runtime-supervisor credential digest used by the final run.

Each disposable child has a fresh preflight UUID and exclusive evidence stream. The final run manifest links both UUIDs and their digests, but preflight lifecycle events can never satisfy final-controller readiness or damping requirements and cannot be reused across a boot.

### Pinned identity and post-drop audit boundary

Cross-UID `/proc` reads are not a runtime dependency. Before the helper exits and while every child is gated, it validates and records:

- controller PID, `/proc` start time, SID, PGID, credentials, all capability sets, `NoNewPrivs`, signal mask, and cgroup membership;
- device/inode identities for pinned user, cgroup, mount, and PID namespace descriptors plus the restricted mount-table digest;
- the NUL-preserved argv and selected environment arrays constructed for `execveat()`;
- the pinned executable descriptor's device, inode, size, mode, empty file-capability xattr, and content digest; and
- the controller's parent ancestry and the outer/nested PTY descriptors.

The helper transfers the controller pidfd, read-only executable descriptor, namespace descriptors, cgroup directory descriptor, PTY descriptors, and immutable bootstrap record to the supervisor. Descriptor numbers are not identity; `fstat()` device/inode values and content digests are. The controller receives only its allowlisted control/PTY/DDS resources and the read-only, fully sealed seccomp-profile memfd; it receives no cgroup, namespace, supervisor-socket, or helper descriptor.

After the helper exits, the supervisor is allowed to rely only on pidfd polling/signalling, its owned cgroup hierarchy, `fstat()`/rehashing of pinned descriptors, its own files/sockets, and run-bound controller events. It does not retain `CAP_SYS_PTRACE` and does not promise to re-read `/proc/<controller>/exe`, `environ`, `cmdline`, namespace symlinks, or ancestry. Any such post-drop read is diagnostic only; denial is expected and cannot invalidate the pinned evidence or silently weaken a check.

### Foreground terminal topology

The supervisor remains the foreground process of the outer tmux PTY. The detached controller owns a nested slave PTY used for its keyboard input. The supervisor relays ordinary input bytes to that PTY but does not treat the pane foreground PID as controller identity.

Consequently, outer-terminal Ctrl+C makes SIGINT pending for the foreground supervisor, not the detached controller. Because every supervisor thread keeps SIGINT/SIGTERM blocked, only the dedicated `sigwaitinfo()` thread consumes it. That thread captures `t_terminal_wait_return` immediately on return and calls `pidfd_send_signal()` directly before audit persistence; there is no asynchronous callback flag or cross-thread handoff. Outer-terminal SIGTERM follows the same rule. The supervisor remains alive through controller handoff/exit, and the controller's dedicated waiter records its own delivery timestamp.

The supervisor signal-wait and run-socket stop-ingress threads are separate from logging and manifest persistence. In real mode both use signed-profile `SCHED_FIFO` priorities lower than the writer but higher than ordinary supervisor work, a dedicated non-writer CPU, locked/prefaulted memory, exclusively owned preallocated request slots, and no filesystem operations before signalling. Failure to configure or read back either envelope prevents the supervisor from granting the arm token.

Manual `O` is an ordinary byte relayed to the controller input callback and remains dependent on that callback being responsive. It is never synthesized as an automated fallback. A hung input callback is stopped through the supervisor signal path or hardware E-stop, not `O`.

Tmux is presentation only. The immutable session/pane IDs and the nested PTY identity are recorded for topology tests and operator navigation; automated stop never uses `tmux send-keys`.

### Immutable run identity

Before actuation the supervisor records:

- schema version and cryptographically random run UUID;
- environment (`sim` or `real`);
- supervisor PID, `/proc` start time, executable, SID, and PGID;
- controller PID plus the helper-pinned `/proc` start time, SID, PGID, credential/capability snapshot, and parent ancestry;
- pinned executable descriptor device/inode/size/digest and the fact that it is the sole `execveat()` source;
- NUL-preserved argv and selected environment arrays constructed before execution;
- controller pidfd ownership plus pinned user/cgroup/mount/PID namespace and cgroup-directory descriptor identities;
- exact run-specific UID/GID maps, supervisor `CAP_KILL`-only sets, controller empty capability sets in the run/ancestor user namespaces, `NoNewPrivs`, seccomp bytecode/digest/readback, kernel/ABI/host-policy profile, and signal-authority preflight results;
- per-run cgroup path and supervisor socket path;
- outer tmux PTY and nested controller PTY device/inode and foreground PGIDs;
- immutable tmux session ID and pane ID, if used;
- exclusive log path, device/inode, and starting offset zero;
- controller, patch, executable, model, config, network-interface, and hardware hashes; and
- lifecycle state, revision, and monotonic sequence.

The retained pidfd plus verified runtime `CAP_KILL` is the controller signal authority, and pidfd polling is the exit authority. The protected per-run cgroup is the descendant-membership authority. Pinned descriptors and the bootstrap record replace cross-UID `/proc` re-reads for executable, argv, environment, namespaces, and ancestry. Containment depends on the distinct credentials, delegation boundary, inaccessible common-ancestor interfaces, read-only namespaced mount, and cleared controller capabilities—not ownership alone. A pane or PTY mismatch is diagnostic, not permission to retarget another process.

## Structured Evidence

Each run gets a newly created `0700` directory and exclusive `0600` files. Reusing or appending to a previous run log is forbidden. Lifecycle decisions do not grep stdout.

The controller/supervisor channel is a bounded `SOCK_SEQPACKET` socket inherited across exec. Startup-gate messages (`SIGNAL_WAITER_READY` and `SAFETY_WRITER_READY`) are fixed-size, preallocated records sent with `MSG_DONTWAIT | MSG_NOSIGNAL`. If either cannot be delivered and acknowledged, launch exits before `PUBLICATION_ARMED`.

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

- `supervisor_signal_waiter_ready`;
- `containment_verified`;
- `signal_authority_sigint_verified`;
- `signal_authority_sigterm_verified`;
- `final_controller_pidfd_received`;
- `controller_seccomp_verified`;
- `bootstrap_exited`;
- `controller_starting`;
- `signal_waiter_ready`;
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

The supervisor's safety-dispatch path is in-memory and non-persistent. For stop CLI, Ctrl+C, SIGTERM, or supervisor API requests, it sends the pidfd signal first. Only afterward may the supervisor acquire the manifest lock, append logs, call `fsync`, inspect its owned cgroup/pinned descriptors, hash artifacts, or print status. Cross-UID `/proc` re-reading is not part of this path. Manifest durability is audit work, not a prerequisite for damping.

## Lifecycle and Idempotence

The supervisor is the single manifest writer. It uses an advisory lock plus write-to-new-file, `fsync`, atomic rename, and directory `fsync` for every transition.

Common states are:

```text
LAUNCHING_SIGNALS_BLOCKED -> SUPERVISOR_SIGNAL_WAITER_READY
                           -> CONTAINMENT_VERIFIED
                           -> SIGNAL_AUTHORITY_VERIFIED
                           -> FINAL_CONTROLLER_LAUNCHED
                           -> SIGNAL_WAITER_READY -> CONTROLLER_SANDBOX_VERIFIED
                           -> BOOTSTRAP_EXITED
                           -> RESOURCES_LOADING -> SAFETY_WRITER_READY
                           -> PUBLICATION_ARMED -> RUNNING
                           -> STOP_REQUESTED -> DAMPING_LATCHED

LAUNCHING_SIGNALS_BLOCKED/SUPERVISOR_SIGNAL_WAITER_READY/CONTAINMENT_VERIFIED/
SIGNAL_AUTHORITY_VERIFIED/FINAL_CONTROLLER_LAUNCHED/SIGNAL_WAITER_READY/
CONTROLLER_SANDBOX_VERIFIED/BOOTSTRAP_EXITED/RESOURCES_LOADING/SAFETY_WRITER_READY
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
5. performs manifest, pinned-descriptor/cgroup, log, and identity audit only after dispatch; and
6. waits for run-bound structured events and reports the environment-specific result.

The manual `O` callback captures its entry timestamp and calls the same controller `RequestShutdown("operator_o", t_o_callback)` API. `SIGINT`, `SIGTERM`, Ctrl+C forwarding, internal safety faults, and supervisor requests converge on the same writer latch with their own source timestamps.

There is no automated `O` injection, PID-based `kill()`, process-group kill, `SIGKILL`, or fallback target search. If the supervisor or pidfd is unavailable, the command fails closed and tells the operator to use the hardware E-stop. It must not improvise a process target.

In real mode, the command reports `DAMPING_PUBLISHED_HOST` distinctly and keeps the controller alive until manual handoff confirmation. It must not print “robot damped” or “safe” based only on host evidence.

## Psi0 Real-Client Gate

The refusing entrypoint is `real/scripts/deploy_psi0-sonic-rtc-client.sh`. It must not exec the client unless a rehearsal receipt is supplied and validates exactly.

The receipt binds:

- controller source revision and applied patch digests;
- built controller executable digest;
- bootstrap-helper, supervisor, and stop-tool digests;
- SONIC encoder, decoder, observation config, planner, and robot config digests;
- Psi0 client and checkpoint digests;
- network interface identity;
- robot serial/hardware identity and current boot ID;
- approved source-to-host-publish and MuJoCo-observation deadlines;
- writer, controller signal-waiter, and supervisor stop-ingress scheduling/priority, CPU affinity, memory-locking, cpuset/IRQ isolation, and stress profile;
- run-specific UID/GID maps; supervisor `CAP_KILL`-only user-namespace authority; controller empty capability sets in the run/ancestor namespaces; `NoNewPrivs`; exact seccomp bytecode/digest/readback; pinned kernel, architecture, ABI, syscall-table, glibc/CUDA and host-userns-policy values; cgroup-v2 delegation/mount/namespace identity; interface ownership/modes; and containment-probe result;
- SIGINT/SIGTERM final-credential pidfd preflight records and pinned executable/namespace/cgroup evidence;
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
- SIGINT/SIGTERM are blocked before supervisor thread creation, inherited blocked across controller `execveat()`, and consumed only by the dedicated supervisor/controller `sigwaitinfo()` threads;
- no asynchronous signal handler, `sig_atomic_t` handoff, or plain-field signal publication remains;
- the packed activation atomic is always lock-free and its release/acquire publication makes the winning request record visible;
- exhaustive arm/stop race tests prove one linearization order: cancel-win produces zero writes, while arm-win requires timely damping even when the source timestamp predates arming;
- the runtime supervisor has exactly `CAP_KILL` in the recorded run user namespace, no capability in the parent namespace, and no `kill()`, `tgkill()`, process-group, or PID-only signal call site;
- the controller cannot change to supervisor UID/GID, gain a capability in the run or an ancestor user namespace, access supervisor/helper sockets, or access writable cgroup delegation files;
- the exact pre-`execveat()` seccomp filter is present on the controller and every descendant: alternate architectures/x32 and all `clone3()` calls return `ENOSYS`, `unshare()`/`setns()` return `EPERM`, and legacy `clone()` rejects every individual or combined namespace flag while allowing the required non-namespace thread/process forms;
- direct, libc-wrapper, forked-child, raw-syscall, and adversarial mixed-flag tests prove that no controller path can create or join a child user, mount, cgroup, PID, network, IPC, UTS, or time namespace;
- filtered full startup, CUDA/TensorRT initialization, inference, planner, input, DDS, evidence, shutdown, and lazy-thread-creation paths complete on the pinned kernel/glibc/CUDA stack without relying on `clone3()`;
- SIGINT and SIGTERM are each delivered through pidfd to a fresh exact controller preflight child by the final-credential supervisor, producing `STARTUP_CANCELLED` and zero DDS writes;
- post-helper audit uses only pinned descriptors, pidfd, owned cgroup files, and run-bound evidence; tests deny cross-UID `/proc` reads and prove no safety result depends on them;
- no body or hand DDS write occurs before `PUBLICATION_ARMED` and the safety writer is already ready at that transition;
- writer latch is monotonic and writer output dominates producer buffers;
- body damping and both hand-relax messages have exact required fields;
- hand mode bytes are exactly `0x90 | motor_id` for IDs 0 through 6 on both topics;
- no normal command can be committed or published after the latch;
- the writer thread passes scheduling, affinity, memory-locking, allocation, lock-ownership, and isolation checks;
- evidence queue saturation, EPIPE, SIGPIPE, and supervisor death cannot block or terminate the writer;
- ZMQ planner waits return promptly when stop is requested;
- lifecycle transitions reject skips, regressions, partial data, and concurrent mutation;
- the controller and its probe descendant cannot write any reachable `cgroup.procs`/`cgroup.threads`, reach a host/sibling hierarchy, remount cgroup v2, create/join another namespace, or gain capability in the run/ancestor user namespaces;
- per-run cgroup membership survives child reparenting and must be empty for graceful exit;
- stop tooling contains no broad or PID-only signalling path; and
- the official real client launcher refuses absent, stale, mismatched, expired, or unsigned receipts.

### Process integration matrix

Startup injection tests deliver SIGINT, SIGTERM, Ctrl+C, and stop-CLI requests before exec, before `SIGNAL_WAITER_READY`, during resource loading, after publisher creation, at `SAFETY_WRITER_READY`, and across the arm transition. The expected result is defined by the packed activation atomic's modification order, not by an unsynchronized observation or source timestamp: cancellation-win cases end in `STARTUP_CANCELLED` with zero body and hand DDS writes; arm-win cases meet the complete damping deadline. A deterministic race harness pauses each contender immediately before its compare-and-swap and exercises both orders thousands of times under ThreadSanitizer where supported.

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

- PID reuse and pinned bootstrap-record/start-time mismatch;
- tmux pane death, replacement, and foreground-process change;
- outer/nested PTY foreground-group mismatch and Ctrl+C forwarding failure;
- supervisor death and socket replacement;
- stale, rotated, truncated, and wrong-inode logs;
- missing, partial, corrupt, and old-schema manifests;
- controller exit between validation and stop request;
- hung input callback, planner initialization, policy inference, and planner inference;
- blocked or failed DDS publication;
- evidence queue saturation, socket backpressure, EPIPE, and log/manifest `fsync` delay;
- descendant fork, reparent, and attempted cgroup escape;
- mismatched UID/GID maps, missing or parent-namespace `CAP_KILL`, nonzero controller capability in the run/ancestor namespaces, `pidfd_send_signal()` `EPERM`, surviving helper, a runtime path that requires ptrace-gated `/proc` access, missing/mismatched seccomp filter, wrong architecture or x32 syscall attempt, namespace-bearing `clone()` acceptance, any `unshare()`/`setns()` acceptance, `clone3()` execution instead of `ENOSYS`, required runtime failure without `clone3()`, changed kernel/syscall table/glibc/CUDA profile, writable cgroup mount, inherited cgroup descriptor, failed namespace isolation, and containment/signal-probe deception or timeout; and
- unrelated processes with matching names and argv prefixes.

No fault case may signal a replacement or unrelated process. Abnormal state must survive restart attempts until acknowledged.

### MuJoCo acceptance

An independent subscriber on `rt/lowcmd`, `rt/dex3/left/cmd`, and `rt/dex3/right/cmd` records receipt timestamps and full payloads while active nonzero commands are being published.

For every applicable source/fault pair:

- end-to-end, signal-send-to-wait-return, wait-return-to-latch, and latch-to-publish intervals use `CLOCK_MONOTONIC_RAW` and meet the configured deadline;
- host-side writer instrumentation records the begin timestamp, topic, safe/normal classification, and monotonic publish sequence for every `Write()` invocation without adding blocking I/O to the writer;
- for each of the body, left-hand, and right-hand topics, the first host `Write()` invocation begun after `t_latched` is the required safe command and no later host invocation is normal;
- the first independently observed body damping command associated with the post-latch host publish has all 29 `tau=0`, `q=0`, `dq=0`, `kp=0`, and `kd=8`;
- the first independently observed post-latch command on each hand topic contains seven commands whose mode bytes are exactly `0x90 | motor_id`, with zero `tau`, `q`, `dq`, `kp`, and `kd`;
- a normal command whose final check and `Write()` began before the latch may complete or arrive after `t_latched`; it is treated as an in-flight pre-latch command, not as the first post-latch host publish;
- after the first independently observed damping/relax command on a topic, every later observed command through writer shutdown remains safe;
- injected input/control/planner hangs do not stop the damping writer;
- evidence and manifest backpressure do not change the damping latency or payload;
- when evidence loss is not injected, the structured run sequence is complete and correctly scoped;
- when evidence loss is injected, the run becomes abnormal without changing damping latency or payload; and
- pidfd observation proves exact controller exit and the protected per-run cgroup is empty.

The observer timestamps callback entry with `CLOCK_MONOTONIC_RAW`. `t_sim_observed` is the latest of the first qualifying post-latch safe observations on the body, left-hand, and right-hand topics. Host instrumentation proves which `Write()` calls began after the latch; observer ordering proves what the independent simulation transport received. Neither substitutes for the other.

The PTY acceptance test proves that Ctrl+C on the outer tmux PTY becomes pending for and is timestamped by the foreground supervisor's sole `sigwaitinfo()` thread, is forwarded exactly through the controller pidfd, is returned by the controller's process-entry-created waiter, and latches damping. It separately proves that ordinary bytes, including `O`, traverse the nested PTY and that pane foreground-process changes cannot retarget the stop.

Latency is measured over repeated loaded-host trials with CPU, memory, DDS/network, logging, and inference stress. Any single hard-ceiling miss fails the receipt; averages or percentiles cannot hide a miss.

### Real-hardware acceptance

Real testing remains blocked until all of the following exist:

- approved MuJoCo receipt for the exact artifacts;
- clear area, physical support, tested hardware E-stop, and dedicated E-stop operator;
- approved host-publish deadline from the robot safety owner;
- current-boot SIGINT/SIGTERM preflight evidence from the final-credential `CAP_KILL`-only supervisor plus helper-exit confirmation;
- exact seccomp readback and namespace-attempt evidence on the pinned kernel/ABI/runtime stack, with `/proc/sys/kernel/unprivileged_userns_clone=0` recorded;
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

Those hashes document the reviewed local candidates; they do not substitute for a committed prerequisite. The emergency-stop patch must be a later, separately reviewable artifact against that exact baseline. No implementation plan begins until the prerequisite integration commit is merged and verified; design approval alone does not clear that gate.

## Acceptance Criteria for Design Approval

- The reviewer accepts the writer-level body and hand latch and timing semantics.
- The reviewer accepts that only the hardware E-stop is an independent fault domain.
- Signal installation, startup, repetition, teardown, and data-race handling are explicit.
- The first-actuation boundary cannot be crossed before signal-waiter and safety-writer readiness, and arm versus cancellation has one tested atomic order.
- End-to-end timing begins at source/callback entry on mandatory `CLOCK_MONOTONIC_RAW`; safety dispatch precedes persistence.
- Stable pidfd ownership plus tested `CAP_KILL` in the controller's run-specific user namespace replaces PID/tmux targeting for all automated signals.
- Foreground-supervisor PTY forwarding makes Ctrl+C delivery explicit and tested.
- Structured evidence is nonblocking, run-bound, and distinguishes simulation observation from host-only real evidence.
- Writer scheduling and resource isolation are verified, and the 10 ms ceiling is described only as measured acceptance.
- Dex3 mode bytes and both hand-topic handoff semantics are exact.
- Lifecycle locking, concurrent calls, abnormal archival, restart blocking, and idempotence are explicit.
- Distinct credentials, inaccessible delegation/common-ancestor interfaces, a read-only namespaced cgroup view, zero controller capabilities in the run/ancestor user namespaces, the namespace-prohibition filter, and pre-arm escape probes establish per-run cgroup containment; ownership alone is not evidence.
- A pre-`execveat()`, inherited and read-back seccomp filter prohibits `unshare()`, `setns()`, namespace-bearing legacy `clone()`, and all `clone3()` calls; filtered startup and loaded-runtime tests prove the pinned stack remains functional.
- Pinned pre-helper-exit evidence replaces ptrace-gated cross-UID `/proc` re-reads without granting `CAP_SYS_PTRACE`.
- The client-gate limitation is described as enforced launcher workflow plus manual sign-off, not actuator proof.
- The complete failure and signal matrix is part of acceptance testing.
- A prerequisite artifact commit makes the pinned controller, patch mechanism, and client source reproducible.
- Design approval alone authorizes neither real-robot commands nor implementation planning; the prerequisite integration commit must merge before planning, and real-hardware qualification requires the separate gates above.
