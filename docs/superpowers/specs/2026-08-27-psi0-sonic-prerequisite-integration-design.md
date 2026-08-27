# Psi0 + SONIC Prerequisite Artifact Integration Design

**Status:** APPROVED FOR DESIGN-DOCUMENT CREATION ONLY; artifact implementation remains blocked pending review of this committed document
**Date:** 2026-08-27
**Scope:** Make the pinned GEAR-SONIC v1.1 source, compatibility patch, installer, and Psi0 RTC client source reproducible without enabling VLA publication

This document defines the repository prerequisite required by the approved real-robot emergency-stop design. It does not authorize artifact implementation, emergency-stop implementation planning, publication, simulation, deployment-host changes, or robot operation.

## Relationship to the Emergency-Stop Design

The controlling safety contract is `docs/superpowers/specs/2026-08-26-psi0-sonic-real-emergency-stop-design.md`. That design requires a self-contained prerequisite artifact commit before emergency-stop implementation planning begins.

The prerequisite commit exists only to make reviewed source inputs reproducible. It must not make the Psi0 RTC publication path operational. In particular, `real/scripts/deploy_psi0-sonic-rtc-client.sh` remains an unconditional refusal until the later emergency-stop implementation replaces it with receipt validation followed by execution.

## Reviewed Inputs

The prerequisite integration is pinned to these exact inputs:

| Artifact | Source | Required identity |
| --- | --- | --- |
| GEAR-SONIC source | `https://github.com/NVlabs/GR00T-WholeBodyControl.git` | gitlink `c374bae5b9039cd0ee71377e654d11ce1bc69e1d` |
| ZMQ streamed-mode compatibility patch | reviewed dirty-workspace candidate | SHA-256 `34d20ee831999b08cdfc0e7215f6cbf8e8b0ac4ee0f5691afa994eb669229a06` |
| Compatibility installer | reviewed dirty-workspace candidate | SHA-256 `234ec9536138516f7157341e9081dfa508a3388816a1a7bc1d794a0b752ddf92` |
| Psi0 RTC client | `physical-superintelligence-lab/GR00T-WholeBodyControl` | commit `d40376994ff094787591ab76a691c50b8e131786`, path `psi_rtc_sonic_client.py`, SHA-256 `fe555caa6aa91bca350edf3fec71064532da4c7e3cb484ea2b2ab7ac0dc3726e` |
| Patched ZMQ header | clean `c374bae5...` plus the reviewed patch | SHA-256 `d8d91661459765e3d634acdfec5f52a942346acaab99627ad5d65126f5cd6e00` |

The pristine pinned ZMQ header has SHA-256 `327723b44dcb3cc38c4ca65f9bcb2aebce06c062224f7d1a28738b8fc15e934b`. Tests use both pristine and patched hashes to prove that the reviewed patch was applied to the intended input, rather than to an overlapping local modification.

## Repository Layout

The later artifact commit will add or modify only the following prerequisite-owned paths:

- `.gitmodules` declares the official NVlabs submodule URL.
- `third_party/GR00T-WholeBodyControl` is a mode-`160000` gitlink at the pinned revision.
- `patches/gr00t-wholebodycontrol/0001-zmq-manager-propagate-start.patch` stores the exact reviewed patch bytes.
- `scripts/setup/apply_gr00t_v1_1_compat.sh` stores the exact reviewed installer bytes.
- `real/SONIC/vendor/psi_rtc_sonic_client.py` stores the exact upstream client bytes without modification.
- `real/SONIC/vendor/psi_rtc_sonic_client.provenance.json` records source and future-runtime metadata.
- `real/scripts/deploy_psi0-sonic-rtc-client.sh` becomes the exact fail-closed launcher defined below.
- `tests/test_sonic_prerequisite_artifacts.py` verifies identity, patch output, provenance, and refusal behavior.

No emergency-stop implementation, receipt parser, controller change, executable, model, checkpoint, generated output, virtual environment, or submodule-local untracked file belongs in this commit.

## Official Submodule Pin

`.gitmodules` must contain exactly one entry for `third_party/GR00T-WholeBodyControl` with:

```ini
[submodule "third_party/GR00T-WholeBodyControl"]
	path = third_party/GR00T-WholeBodyControl
	url = https://github.com/NVlabs/GR00T-WholeBodyControl.git
	branch = main
```

The parent repository index entry must have mode `160000` and object ID `c374bae5b9039cd0ee71377e654d11ce1bc69e1d`. A checkout with a dirty patched submodule is not acceptable as repository evidence; the patch is stored and tested in the parent repository, while the committed gitlink remains clean and exact.

## Compatibility Patch and Installer

The patch adds only the reviewed streamed-mode start propagation in `gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/include/input_interface/zmq_manager.hpp`. The installer:

1. resolves the Psi0 and submodule roots relative to its own path;
2. refuses an uninitialized submodule;
3. refuses any submodule revision other than `c374bae5b9039cd0ee71377e654d11ce1bc69e1d`;
4. reports success without changing files when the patch is already applied;
5. uses `git apply --check` before applying the patch; and
6. fails if the clean or reverse checks show overlapping, unreviewed changes.

The prerequisite integration preserves the reviewed patch and installer byte-for-byte. Improvements to their behavior require a separate review and new hashes.

## Vendored Client and Provenance

`real/SONIC/vendor/psi_rtc_sonic_client.py` is a byte-for-byte copy of the file in the PSI fork. It is not reformatted, renamed internally, patched, or given a local license header because any byte change would invalidate the reviewed digest.

The adjacent JSON provenance record contains these exact fields:

```json
{
  "artifact": "psi_rtc_sonic_client.py",
  "source_repository": "https://github.com/physical-superintelligence-lab/GR00T-WholeBodyControl.git",
  "source_commit": "d40376994ff094787591ab76a691c50b8e131786",
  "source_path": "psi_rtc_sonic_client.py",
  "sha256": "fe555caa6aa91bca350edf3fec71064532da4c7e3cb484ea2b2ab7ac0dc3726e",
  "license": "Apache-2.0",
  "license_source": "https://github.com/physical-superintelligence-lab/GR00T-WholeBodyControl/blob/d40376994ff094787591ab76a691c50b8e131786/LICENSE",
  "future_pythonpath": "third_party/GR00T-WholeBodyControl"
}
```

The client assumes `gear_sonic` is importable beside the script. Vendoring changes that filesystem relationship. A future, separately approved receipt-gated launcher must therefore prepend the pinned `third_party/GR00T-WholeBodyControl` directory to `PYTHONPATH` before invoking the unchanged vendored client. The prerequisite launcher does not set `PYTHONPATH` because it does not execute the client.

The source repository licenses scripts and source code under Apache License 2.0. The provenance record preserves that license identity and its source URL without modifying the reviewed client bytes.

## Unconditional Launcher Refusal

At the prerequisite stage, `real/scripts/deploy_psi0-sonic-rtc-client.sh` has exactly this behavior:

```bash
#!/usr/bin/env bash
set -euo pipefail

printf '%s\n' \
    '[psi0-sonic] REFUSED: emergency-stop/receipt gate not implemented; RTC publication is disabled.' >&2
exit 78
```

Exit status `78` denotes a configuration gate failure. The script performs only shell built-in `printf` and `exit` operations. It does not resolve a Python executable, change directory, set `PYTHONPATH`, import the vendored client, open ZMQ or WebSocket connections, send a start command, or publish an action.

The later emergency-stop implementation may replace this exact refusal only after its receipt schema, validator, controller safety path, and qualification tests are approved and implemented. The replacement must validate the receipt before resolving or executing Python.

## Verification Design

All tests are non-actuating. They must not start SONIC, connect to a robot, bind or connect a socket, or invoke the vendored client.

### Static artifact identity

`tests/test_sonic_prerequisite_artifacts.py` verifies:

- `.gitmodules` resolves the GR00T-WBC path to the exact official URL and contains no PSI-fork URL for that submodule;
- `git ls-files --stage third_party/GR00T-WholeBodyControl` returns mode `160000` and object ID `c374bae5b9039cd0ee71377e654d11ce1bc69e1d`;
- the patch, installer, and vendored client SHA-256 values equal the reviewed values;
- the provenance JSON has exactly the reviewed repository, commit, source path, digest, Apache-2.0 license, license source, and future `PYTHONPATH`; and
- the vendored client compiles with `py_compile` without importing or executing it.

### Exact clean-checkout patch output

The patch-output test requires the pinned submodule to be initialized. It creates a temporary archive from the submodule's committed `HEAD`, never from its working tree, and then:

1. asserts the submodule `HEAD` is the gitlink revision;
2. extracts only the target header from `git archive` into a temporary directory;
3. verifies the pristine header digest is `327723b44dcb3cc38c4ca65f9bcb2aebce06c062224f7d1a28738b8fc15e934b`;
4. runs `git apply --check` against the reviewed patch in that temporary directory;
5. applies the patch there;
6. verifies the resulting header digest is `d8d91661459765e3d634acdfec5f52a942346acaab99627ad5d65126f5cd6e00`; and
7. verifies `git apply --reverse --check` succeeds on that output.

This proves exact patch output from the clean pinned object without mutating the checked-out submodule. A separate installer test uses a disposable local clone of the initialized pinned submodule, runs the exact installer twice, verifies the first run applies the patch, verifies the second reports it already applied, and confirms the same patched-header digest. It performs no network access.

### Fail-closed launcher

The launcher test:

1. asserts the launcher text exactly equals the approved refusal script;
2. installs a fake executable named `python` in a temporary directory that creates a sentinel if called;
3. prepends that directory to `PATH` and points conventional Python override environment variables at the same sentinel;
4. invokes the launcher with arbitrary client-like arguments;
5. requires exit status `78` and the exact refusal message on stderr;
6. requires empty stdout; and
7. proves the sentinel was not created.

The exact-text assertion establishes that no alternate absolute Python path or socket-producing command exists beyond the fake executable's observation scope.

### Required verification commands

The artifact implementation plan must use the repository's Python 3.10 environment and include these non-actuating checks:

```bash
python -m pytest -q tests/test_sonic_prerequisite_artifacts.py
python -m py_compile real/SONIC/vendor/psi_rtc_sonic_client.py
sha256sum \
  patches/gr00t-wholebodycontrol/0001-zmq-manager-propagate-start.patch \
  scripts/setup/apply_gr00t_v1_1_compat.sh \
  real/SONIC/vendor/psi_rtc_sonic_client.py
git diff --check
git status --short
```

No test command launches the real client or controller.

## Commit and Review Boundary

The prerequisite artifact work is one separately reviewable integration commit whose tree contains only the paths listed in this design. Its PR must state that the launcher is intentionally unusable and that merging it does not clear any emergency-stop, simulation, deployment, or hardware gate.

After merge, reviewers verify that the artifact branch is an ancestor of `origin/main`, re-run the hash and patch-output tests from a clean recursive checkout, and record the prerequisite merge commit. Only that verified merge clears the repository prerequisite for writing the emergency-stop implementation plan. It does not authorize implementation execution or any runtime action.

## Acceptance Criteria

- The official submodule URL, mode, and gitlink are exact.
- The patch and installer bytes match their reviewed digests.
- Applying the exact patch to the clean pinned object produces the reviewed header digest.
- The vendored client bytes and provenance metadata match their reviewed source.
- The client source remains unmodified and is never imported or executed by prerequisite tests.
- The launcher is the exact unconditional refusal, exits nonzero, and cannot reach Python or sockets.
- Future `PYTHONPATH` handling is documented but not enabled.
- The diff contains no emergency-stop implementation, receipt validator, runtime launch, generated artifact, model, environment, simulator, host, or robot change.
- Artifact implementation remains blocked until this committed design document receives an independent approval verdict.
