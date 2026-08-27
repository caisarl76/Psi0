# Psi0 + SONIC Prerequisite Artifact Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Commit the exact reviewed GEAR-SONIC gitlink, compatibility patch and installer, vendored Psi0 RTC client provenance, and an unconditionally refusing supported launcher without executing the client or enabling publication through that launcher.

**Architecture:** The parent repository owns immutable artifact identities and a fail-closed shell entrypoint. Tests use only local Git objects and temporary fixtures: they prove the official gitlink, exact hashes, clean patch output, installer idempotence, and supported-launcher refusal while never importing or invoking the vendored client.

**Tech Stack:** Git submodules, Bash, Python 3.10 standard library, pytest, SHA-256, Git patch tooling.

---

## File Structure

- Modify `.gitmodules` to declare the official NVlabs GR00T-WBC submodule.
- Add `third_party/GR00T-WholeBodyControl` as mode-`160000` gitlink `c374bae5b9039cd0ee71377e654d11ce1bc69e1d`.
- Create `patches/gr00t-wholebodycontrol/0001-zmq-manager-propagate-start.patch` with the reviewed patch bytes.
- Create `scripts/setup/apply_gr00t_v1_1_compat.sh` with the reviewed version-pinned installer bytes.
- Create `real/SONIC/vendor/psi_rtc_sonic_client.py` from the immutable reviewed source.
- Create `real/SONIC/vendor/psi_rtc_sonic_client.provenance.json` with exact provenance and future import-path metadata.
- Replace `real/scripts/deploy_psi0-sonic-rtc-client.sh` with the exact unconditional refusal.
- Create `tests/test_sonic_prerequisite_artifacts.py` as the only artifact contract test module.

The approved specification requires one artifact implementation commit. Do not commit intermediate red or partially integrated states.

### Task 1: Add the failing artifact contract tests

**Files:**
- Create: `tests/test_sonic_prerequisite_artifacts.py`
- Reference: `docs/superpowers/specs/2026-08-27-psi0-sonic-prerequisite-integration-design.md`

- [ ] **Step 1: Write the complete failing test module**

Create `tests/test_sonic_prerequisite_artifacts.py` with:

```python
from __future__ import annotations

import hashlib
import io
import json
import os
import shutil
import stat
import subprocess
import sys
import tarfile
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
GR00T_PATH = REPO_ROOT / "third_party/GR00T-WholeBodyControl"
PATCH_PATH = (
    REPO_ROOT
    / "patches/gr00t-wholebodycontrol/0001-zmq-manager-propagate-start.patch"
)
INSTALLER_PATH = REPO_ROOT / "scripts/setup/apply_gr00t_v1_1_compat.sh"
CLIENT_PATH = REPO_ROOT / "real/SONIC/vendor/psi_rtc_sonic_client.py"
PROVENANCE_PATH = CLIENT_PATH.with_suffix(".provenance.json")
LAUNCHER_PATH = REPO_ROOT / "real/scripts/deploy_psi0-sonic-rtc-client.sh"

PIN = "c374bae5b9039cd0ee71377e654d11ce1bc69e1d"
PATCH_SHA256 = "34d20ee831999b08cdfc0e7215f6cbf8e8b0ac4ee0f5691afa994eb669229a06"
INSTALLER_SHA256 = "234ec9536138516f7157341e9081dfa508a3388816a1a7bc1d794a0b752ddf92"
CLIENT_SHA256 = "fe555caa6aa91bca350edf3fec71064532da4c7e3cb484ea2b2ab7ac0dc3726e"
PRISTINE_HEADER_SHA256 = (
    "327723b44dcb3cc38c4ca65f9bcb2aebce06c062224f7d1a28738b8fc15e934b"
)
PATCHED_HEADER_SHA256 = (
    "d8d91661459765e3d634acdfec5f52a942346acaab99627ad5d65126f5cd6e00"
)
TARGET_HEADER = Path(
    "gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/include/"
    "input_interface/zmq_manager.hpp"
)
REFUSAL_MESSAGE = (
    "[psi0-sonic] REFUSED: emergency-stop/receipt gate not implemented; "
    "RTC publication is disabled.\n"
)
REFUSAL_SCRIPT = """#!/usr/bin/env bash
set -euo pipefail

printf '%s\\n' \\
    '[psi0-sonic] REFUSED: emergency-stop/receipt gate not implemented; RTC publication is disabled.' >&2
exit 78
"""


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run(
    *args: str | Path,
    cwd: Path = REPO_ROOT,
    env: dict[str, str] | None = None,
    check: bool = True,
    text: bool = True,
) -> subprocess.CompletedProcess:
    return subprocess.run(
        [str(arg) for arg in args],
        cwd=cwd,
        env=env,
        check=check,
        capture_output=True,
        text=text,
    )


def assert_pristine_checkout(repo: Path) -> None:
    assert run("git", "status", "--porcelain=v1", "--untracked-files=all", cwd=repo).stdout == ""
    assert run("git", "diff", "--quiet", cwd=repo, check=False).returncode == 0
    assert run("git", "diff", "--cached", "--quiet", cwd=repo, check=False).returncode == 0
    assert run("git", "ls-files", "--others", "--exclude-standard", cwd=repo).stdout == ""


def assert_only_reviewed_patch(repo: Path, expected_patch: bytes) -> None:
    actual_diff = run(
        "git", "diff", "--binary", "--", TARGET_HEADER, cwd=repo, text=False
    ).stdout
    assert actual_diff == expected_patch
    assert run("git", "diff", "--name-only", cwd=repo).stdout.splitlines() == [
        TARGET_HEADER.as_posix()
    ]
    assert run("git", "diff", "--cached", "--quiet", cwd=repo, check=False).returncode == 0
    assert run("git", "ls-files", "--others", "--exclude-standard", cwd=repo).stdout == ""
    assert sha256(repo / TARGET_HEADER) == PATCHED_HEADER_SHA256


def test_official_gr00t_submodule_pin() -> None:
    config = REPO_ROOT / ".gitmodules"
    section = "submodule.third_party/GR00T-WholeBodyControl"
    assert run("git", "config", "-f", config, "--get", f"{section}.path").stdout.strip() == (
        "third_party/GR00T-WholeBodyControl"
    )
    assert run("git", "config", "-f", config, "--get", f"{section}.url").stdout.strip() == (
        "https://github.com/NVlabs/GR00T-WholeBodyControl.git"
    )
    assert run("git", "config", "-f", config, "--get", f"{section}.branch").stdout.strip() == "main"
    assert "physical-superintelligence-lab/GR00T-WholeBodyControl" not in config.read_text()

    stage = run(
        "git", "ls-files", "--stage", "third_party/GR00T-WholeBodyControl"
    ).stdout.strip()
    metadata, path = stage.split("\t", maxsplit=1)
    mode, object_id, stage_number = metadata.split()
    assert (mode, object_id, stage_number, path) == (
        "160000", PIN, "0", "third_party/GR00T-WholeBodyControl"
    )


def test_reviewed_patch_and_installer_hashes() -> None:
    assert sha256(PATCH_PATH) == PATCH_SHA256
    assert sha256(INSTALLER_PATH) == INSTALLER_SHA256


def test_vendored_client_identity_and_provenance(tmp_path: Path) -> None:
    assert sha256(CLIENT_PATH) == CLIENT_SHA256
    assert json.loads(PROVENANCE_PATH.read_text()) == {
        "artifact": "psi_rtc_sonic_client.py",
        "source_repository": "https://github.com/physical-superintelligence-lab/GR00T-WholeBodyControl.git",
        "source_commit": "d40376994ff094787591ab76a691c50b8e131786",
        "source_path": "psi_rtc_sonic_client.py",
        "sha256": CLIENT_SHA256,
        "license": "Apache-2.0",
        "license_source": "https://github.com/physical-superintelligence-lab/GR00T-WholeBodyControl/blob/d40376994ff094787591ab76a691c50b8e131786/LICENSE",
        "future_pythonpath": "third_party/GR00T-WholeBodyControl",
    }
    env = os.environ.copy()
    env["PYTHONPYCACHEPREFIX"] = str(tmp_path / "pycache")
    compiled = run(sys.executable, "-m", "py_compile", CLIENT_PATH, env=env)
    assert compiled.stdout == ""
    assert compiled.stderr == ""


def test_patch_output_from_clean_pinned_archive(tmp_path: Path) -> None:
    assert (GR00T_PATH / ".git").exists()
    assert run("git", "rev-parse", "HEAD", cwd=GR00T_PATH).stdout.strip() == PIN
    archive = run(
        "git", "archive", PIN, "--", TARGET_HEADER, cwd=GR00T_PATH, text=False
    ).stdout
    with tarfile.open(fileobj=io.BytesIO(archive), mode="r:") as tar:
        tar.extractall(tmp_path)
    target = tmp_path / TARGET_HEADER
    assert sha256(target) == PRISTINE_HEADER_SHA256
    run("git", "apply", "--check", PATCH_PATH, cwd=tmp_path)
    run("git", "apply", PATCH_PATH, cwd=tmp_path)
    assert sha256(target) == PATCHED_HEADER_SHA256
    run("git", "apply", "--reverse", "--check", PATCH_PATH, cwd=tmp_path)


def test_installer_isolated_exact_and_idempotent(tmp_path: Path) -> None:
    fixture = (tmp_path / "parent").resolve()
    fixture_patch = fixture / PATCH_PATH.relative_to(REPO_ROOT)
    fixture_installer = fixture / INSTALLER_PATH.relative_to(REPO_ROOT)
    fixture_gr00t = fixture / GR00T_PATH.relative_to(REPO_ROOT)
    fixture_patch.parent.mkdir(parents=True)
    fixture_installer.parent.mkdir(parents=True)
    fixture_gr00t.parent.mkdir(parents=True)
    shutil.copy2(PATCH_PATH, fixture_patch)
    shutil.copy2(INSTALLER_PATH, fixture_installer)
    assert sha256(fixture_patch) == PATCH_SHA256
    assert sha256(fixture_installer) == INSTALLER_SHA256
    for path in (fixture_patch, fixture_installer, fixture_gr00t):
        assert path.resolve().is_relative_to(fixture)

    run("git", "clone", "--shared", "--no-checkout", GR00T_PATH, fixture_gr00t)
    run("git", "checkout", "--detach", PIN, cwd=fixture_gr00t)
    assert run("git", "rev-parse", "HEAD", cwd=fixture_gr00t).stdout.strip() == PIN
    assert_pristine_checkout(fixture_gr00t)

    first = run("bash", fixture_installer, cwd=fixture)
    assert first.stdout == "[gr00t-v1.1] applied streamed-mode start fix\n"
    assert first.stderr == ""
    expected_patch = fixture_patch.read_bytes()
    assert_only_reviewed_patch(fixture_gr00t, expected_patch)
    first_diff = run("git", "diff", "--binary", cwd=fixture_gr00t, text=False).stdout

    second = run("bash", fixture_installer, cwd=fixture)
    assert second.stdout == "[gr00t-v1.1] streamed-mode start fix is already applied\n"
    assert second.stderr == ""
    assert_only_reviewed_patch(fixture_gr00t, expected_patch)
    assert run("git", "diff", "--binary", cwd=fixture_gr00t, text=False).stdout == first_diff


def test_supported_launcher_refuses_without_invoking_python(tmp_path: Path) -> None:
    assert LAUNCHER_PATH.read_text() == REFUSAL_SCRIPT
    sentinel = tmp_path / "python-invoked"
    fake_python = tmp_path / "python"
    fake_python.write_text(f"#!/usr/bin/env bash\ntouch '{sentinel}'\nexit 99\n")
    fake_python.chmod(fake_python.stat().st_mode | stat.S_IXUSR)
    env = os.environ.copy()
    env["PATH"] = f"{tmp_path}:{env['PATH']}"
    env["PYTHON"] = str(fake_python)
    env["PYTHON_EXECUTABLE"] = str(fake_python)
    env["PSI0_SONIC_CLIENT_PYTHON"] = str(fake_python)
    result = run(
        "bash", LAUNCHER_PATH, "--host", "127.0.0.1", "--port", "8014",
        cwd=REPO_ROOT, env=env, check=False,
    )
    assert result.returncode == 78
    assert result.stdout == ""
    assert result.stderr == REFUSAL_MESSAGE
    assert not sentinel.exists()
```

- [ ] **Step 2: Run the contract tests and verify the red state**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  /home/jihun/work/GR00T-lowlatency/.venv_teleop/bin/python -m pytest -q \
  tests/test_sonic_prerequisite_artifacts.py
```

Expected: collection succeeds and tests fail because the GR00T gitlink, reviewed artifacts, provenance, and refusing launcher are not yet present. Do not weaken assertions to make the red run pass.

### Task 2: Pin the official submodule and exact compatibility mechanism

**Files:**
- Modify: `.gitmodules`
- Add gitlink: `third_party/GR00T-WholeBodyControl`
- Create: `patches/gr00t-wholebodycontrol/0001-zmq-manager-propagate-start.patch`
- Create: `scripts/setup/apply_gr00t_v1_1_compat.sh`
- Test: `tests/test_sonic_prerequisite_artifacts.py`

- [ ] **Step 1: Add the official submodule and detach it at the reviewed pin**

Run:

```bash
git submodule add -b main \
  https://github.com/NVlabs/GR00T-WholeBodyControl.git \
  third_party/GR00T-WholeBodyControl
git -C third_party/GR00T-WholeBodyControl checkout --detach \
  c374bae5b9039cd0ee71377e654d11ce1bc69e1d
git add .gitmodules third_party/GR00T-WholeBodyControl
```

Expected: `.gitmodules` uses the official URL and `git ls-files --stage` reports mode `160000` with the exact object ID. Do not apply the patch in this checkout.

- [ ] **Step 2: Add the byte-identical reviewed patch**

Create `patches/gr00t-wholebodycontrol/0001-zmq-manager-propagate-start.patch` with exactly:

```diff
diff --git a/gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/include/input_interface/zmq_manager.hpp b/gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/include/input_interface/zmq_manager.hpp
index 6917e59..4267af2 100644
--- a/gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/include/input_interface/zmq_manager.hpp
+++ b/gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/include/input_interface/zmq_manager.hpp
@@ -371,6 +371,13 @@ class ZMQManager : public InputInterface {
       } else {
         // Streamed motion mode: delegate to pose interface
         if (pose_interface_) {
+          // Command messages are owned by ZMQManager, so propagate a start
+          // request before delegating. ZMQEndpointInterface only sees pose
+          // messages and cannot observe the manager's command topic itself.
+          if (start_control_ && !operator_state.start) {
+            operator_state.start = true;
+            reinitialize_heading = true;
+          }
           pose_interface_->handle_input(motion_reader, current_motion, current_frame,
                                        operator_state, reinitialize_heading,
                                        heading_state_buffer,
```

Run:

```bash
sha256sum patches/gr00t-wholebodycontrol/0001-zmq-manager-propagate-start.patch
```

Expected: `34d20ee831999b08cdfc0e7215f6cbf8e8b0ac4ee0f5691afa994eb669229a06`.

- [ ] **Step 3: Add the byte-identical reviewed installer**

Create `scripts/setup/apply_gr00t_v1_1_compat.sh` with exactly:

```bash
#!/usr/bin/env bash
set -euo pipefail

PSI0_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
GR00T_ROOT="${PSI0_ROOT}/third_party/GR00T-WholeBodyControl"
PATCH_PATH="${PSI0_ROOT}/patches/gr00t-wholebodycontrol/0001-zmq-manager-propagate-start.patch"
EXPECTED_REVISION="c374bae5b9039cd0ee71377e654d11ce1bc69e1d"

if [[ ! -d "${GR00T_ROOT}/.git" && ! -f "${GR00T_ROOT}/.git" ]]; then
    echo "[gr00t-v1.1] submodule is not initialized: ${GR00T_ROOT}" >&2
    exit 1
fi

ACTUAL_REVISION="$(git -C "${GR00T_ROOT}" rev-parse HEAD)"
if [[ "${ACTUAL_REVISION}" != "${EXPECTED_REVISION}" ]]; then
    echo "[gr00t-v1.1] expected official revision ${EXPECTED_REVISION}" >&2
    echo "[gr00t-v1.1] found ${ACTUAL_REVISION}; refusing to apply a version-specific patch" >&2
    exit 1
fi

(
    cd "${GR00T_ROOT}"
    if git apply --reverse --check "${PATCH_PATH}" >/dev/null 2>&1; then
        echo "[gr00t-v1.1] streamed-mode start fix is already applied"
    elif git apply --check "${PATCH_PATH}"; then
        git apply "${PATCH_PATH}"
        echo "[gr00t-v1.1] applied streamed-mode start fix"
    else
        echo "[gr00t-v1.1] patch does not apply cleanly; submodule may contain overlapping changes" >&2
        exit 1
    fi
)
```

Run:

```bash
chmod +x scripts/setup/apply_gr00t_v1_1_compat.sh
sha256sum scripts/setup/apply_gr00t_v1_1_compat.sh
```

Expected: `234ec9536138516f7157341e9081dfa508a3388816a1a7bc1d794a0b752ddf92`.

- [ ] **Step 4: Run the submodule, patch-output, and installer tests**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  /home/jihun/work/GR00T-lowlatency/.venv_teleop/bin/python -m pytest -q \
  tests/test_sonic_prerequisite_artifacts.py::test_official_gr00t_submodule_pin \
  tests/test_sonic_prerequisite_artifacts.py::test_reviewed_patch_and_installer_hashes \
  tests/test_sonic_prerequisite_artifacts.py::test_patch_output_from_clean_pinned_archive \
  tests/test_sonic_prerequisite_artifacts.py::test_installer_isolated_exact_and_idempotent
```

Expected: `4 passed`. Confirm separately that `git -C third_party/GR00T-WholeBodyControl status --short` is empty.

### Task 3: Vendor the exact client and keep the supported launcher closed

**Files:**
- Create: `real/SONIC/vendor/psi_rtc_sonic_client.py`
- Create: `real/SONIC/vendor/psi_rtc_sonic_client.provenance.json`
- Modify: `real/scripts/deploy_psi0-sonic-rtc-client.sh`
- Test: `tests/test_sonic_prerequisite_artifacts.py`

- [ ] **Step 1: Retrieve and verify the immutable client source outside the repository**

Run:

```bash
curl -fsSL \
  https://raw.githubusercontent.com/physical-superintelligence-lab/GR00T-WholeBodyControl/d40376994ff094787591ab76a691c50b8e131786/psi_rtc_sonic_client.py \
  -o /tmp/psi_rtc_sonic_client.d403769.py
sha256sum /tmp/psi_rtc_sonic_client.d403769.py
```

Expected: `fe555caa6aa91bca350edf3fec71064532da4c7e3cb484ea2b2ab7ac0dc3726e`. Stop if it differs. Add those exact bytes to `real/SONIC/vendor/psi_rtc_sonic_client.py` using `apply_patch`; do not format or edit the source.

- [ ] **Step 2: Add exact provenance metadata**

Create `real/SONIC/vendor/psi_rtc_sonic_client.provenance.json` with:

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

- [ ] **Step 3: Replace the supported launcher with the exact refusal**

Replace `real/scripts/deploy_psi0-sonic-rtc-client.sh` with:

```bash
#!/usr/bin/env bash
set -euo pipefail

printf '%s\n' \
    '[psi0-sonic] REFUSED: emergency-stop/receipt gate not implemented; RTC publication is disabled.' >&2
exit 78
```

Keep it executable. Do not leave a Python command, `PYTHONPATH`, socket command, or alternate execution branch in the file.

- [ ] **Step 4: Run the vendoring and refusal tests**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  /home/jihun/work/GR00T-lowlatency/.venv_teleop/bin/python -m pytest -q \
  tests/test_sonic_prerequisite_artifacts.py::test_vendored_client_identity_and_provenance \
  tests/test_sonic_prerequisite_artifacts.py::test_supported_launcher_refuses_without_invoking_python
```

Expected: `2 passed`. This proves only the supported launcher refuses; direct Python invocation remains possible, unsupported, and operationally prohibited.

### Task 4: Verify and create the single artifact implementation commit

**Files:**
- Stage only: `.gitmodules`
- Stage only: `third_party/GR00T-WholeBodyControl`
- Stage only: `patches/gr00t-wholebodycontrol/0001-zmq-manager-propagate-start.patch`
- Stage only: `scripts/setup/apply_gr00t_v1_1_compat.sh`
- Stage only: `real/SONIC/vendor/psi_rtc_sonic_client.py`
- Stage only: `real/SONIC/vendor/psi_rtc_sonic_client.provenance.json`
- Stage only: `real/scripts/deploy_psi0-sonic-rtc-client.sh`
- Stage only: `tests/test_sonic_prerequisite_artifacts.py`

- [ ] **Step 1: Run the full non-actuating contract suite**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  /home/jihun/work/GR00T-lowlatency/.venv_teleop/bin/python -m pytest -q \
  tests/test_sonic_prerequisite_artifacts.py
```

Expected: `6 passed` and no socket, controller, simulator, or robot process starts.

- [ ] **Step 2: Verify compile and exact hashes independently**

Run:

```bash
PYTHONPYCACHEPREFIX=/tmp/psi0-prereq-pycache \
  /home/jihun/work/GR00T-lowlatency/.venv_teleop/bin/python -m py_compile \
  real/SONIC/vendor/psi_rtc_sonic_client.py
sha256sum \
  patches/gr00t-wholebodycontrol/0001-zmq-manager-propagate-start.patch \
  scripts/setup/apply_gr00t_v1_1_compat.sh \
  real/SONIC/vendor/psi_rtc_sonic_client.py
```

Expected hashes, in order:

```text
34d20ee831999b08cdfc0e7215f6cbf8e8b0ac4ee0f5691afa994eb669229a06
234ec9536138516f7157341e9081dfa508a3388816a1a7bc1d794a0b752ddf92
fe555caa6aa91bca350edf3fec71064532da4c7e3cb484ea2b2ab7ac0dc3726e
```

- [ ] **Step 3: Verify repository and submodule cleanliness**

Run:

```bash
git -C third_party/GR00T-WholeBodyControl status --short --untracked-files=all
git diff --check
git diff --submodule=short --stat
git status --short
```

Expected: the submodule status is empty. Parent status contains only the eight approved artifact paths; there are no generated files.

- [ ] **Step 4: Stage exactly the artifact paths and inspect the index**

Run:

```bash
git add \
  .gitmodules \
  third_party/GR00T-WholeBodyControl \
  patches/gr00t-wholebodycontrol/0001-zmq-manager-propagate-start.patch \
  scripts/setup/apply_gr00t_v1_1_compat.sh \
  real/SONIC/vendor/psi_rtc_sonic_client.py \
  real/SONIC/vendor/psi_rtc_sonic_client.provenance.json \
  real/scripts/deploy_psi0-sonic-rtc-client.sh \
  tests/test_sonic_prerequisite_artifacts.py
git diff --cached --check
git diff --cached --name-status
git ls-files --stage third_party/GR00T-WholeBodyControl
```

Expected: exactly the eight approved paths are staged, and the gitlink line begins with `160000 c374bae5b9039cd0ee71377e654d11ce1bc69e1d 0`.

- [ ] **Step 5: Commit the one approved artifact integration**

Run:

```bash
git commit -m "chore(sonic): pin prerequisite deployment artifacts"
```

Expected: one commit containing exactly the eight approved artifact paths.

- [ ] **Step 6: Re-run verification from the committed tree**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  /home/jihun/work/GR00T-lowlatency/.venv_teleop/bin/python -m pytest -q \
  tests/test_sonic_prerequisite_artifacts.py
git show --check --stat --oneline HEAD
git diff --name-status HEAD^ HEAD
git status --short
```

Expected: `6 passed`; `git show --check` succeeds; the commit contains exactly the approved artifact paths; and the parent and submodule worktrees are clean.

Stop after this verification. Do not push, create a PR, merge, write the emergency-stop implementation plan, launch SONIC, run MuJoCo, change host settings, or operate a robot until separately authorized.
