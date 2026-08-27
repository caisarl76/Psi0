from __future__ import annotations

import hashlib
import io
import json
import os
import posixpath
import re
import shutil
import stat
import subprocess
import sys
import tarfile
from collections.abc import Mapping
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
GR00T_PATH = REPO_ROOT / "third_party/GR00T-WholeBodyControl"
PATCH_PATH = (
    REPO_ROOT / "patches/gr00t-wholebodycontrol/0001-zmq-manager-propagate-start.patch"
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
EXPECTED_ARCHIVE_DIRECTORIES = (
    "gear_sonic_deploy",
    "gear_sonic_deploy/src",
    "gear_sonic_deploy/src/g1",
    "gear_sonic_deploy/src/g1/g1_deploy_onnx_ref",
    "gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/include",
    "gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/include/input_interface",
)
REFUSAL_MESSAGE = (
    "[psi0-sonic] REFUSED: emergency-stop/receipt gate not implemented; "
    "RTC publication is disabled.\n"
)
REFUSAL_SCRIPT_BYTES = b"""#!/usr/bin/env bash
set -euo pipefail

printf '%s\\n' \\
    '[psi0-sonic] REFUSED: emergency-stop/receipt gate not implemented; RTC publication is disabled.' >&2
exit 78
"""

SAFE_PATH = "/usr/bin:/bin"
SUBPROCESS_TIMEOUT_SECONDS = 30
SHARED_CHECKOUT_TIMEOUT_SECONDS = 120
ALLOWED_ENV_OVERRIDES = frozenset(
    {"PATH", "PYTHON", "PYTHON_EXECUTABLE", "PSI0_SONIC_CLIENT_PYTHON"}
)
PROHIBITED_AMBIENT_ENVIRONMENT = frozenset(
    {
        "BASH_ENV",
        "ENV",
        "SHELLOPTS",
        "BASHOPTS",
        "CDPATH",
        "PYTHONPATH",
        "PYTHONHOME",
        "PYTHONSTARTUP",
        "PYTHONUSERBASE",
        "PYTHONINSPECT",
        "PYTHONEXECUTABLE",
        "GIT_DIR",
        "GIT_WORK_TREE",
        "GIT_INDEX_FILE",
        "GIT_OBJECT_DIRECTORY",
        "GIT_ALTERNATE_OBJECT_DIRECTORIES",
        "GIT_COMMON_DIR",
        "GIT_NAMESPACE",
        "GIT_EXTERNAL_DIFF",
        "GIT_DIFF_OPTS",
        "GIT_CONFIG",
        "GIT_CONFIG_LOCAL",
        "GIT_CONFIG_PARAMETERS",
        "GIT_EXEC_PATH",
        "GIT_TEMPLATE_DIR",
        "GIT_ATTR_SOURCE",
        "GIT_REPLACE_REF_BASE",
        "GIT_CEILING_DIRECTORIES",
        "GIT_DISCOVERY_ACROSS_FILESYSTEM",
        "GIT_SSH",
        "GIT_PROXY_COMMAND",
        "LD_PRELOAD",
        "LD_LIBRARY_PATH",
    }
)
SAFE_GIT_CONFIG = (
    ("core.hooksPath", "/dev/null"),
    ("core.attributesFile", "/dev/null"),
    ("core.fsmonitor", "false"),
    ("diff.external", ""),
    ("filter.lfs.clean", ""),
    ("filter.lfs.smudge", ""),
    ("filter.lfs.process", ""),
    ("filter.lfs.required", "false"),
    ("credential.helper", ""),
    ("protocol.file.allow", "always"),
)
CANONICAL_DIFF_OPTIONS = (
    "--binary",
    "--no-color",
    "--no-ext-diff",
    "--no-textconv",
    "--abbrev=7",
    "--no-renames",
    "--diff-algorithm=myers",
    "--indent-heuristic",
    "--unified=3",
)
READ_ONLY_WORKTREE_GIT_COMMANDS = frozenset(
    {"archive", "cat-file", "config", "ls-files", "rev-parse"}
)


def assert_regular_file(path: Path) -> os.stat_result:
    metadata = path.lstat()
    assert stat.S_ISREG(metadata.st_mode), f"expected a regular file: {path}"
    return metadata


def read_bytes(path: Path) -> bytes:
    assert_regular_file(path)
    return path.read_bytes()


def sha256(path: Path) -> str:
    return hashlib.sha256(read_bytes(path)).hexdigest()


def assert_executable_file(path: Path) -> None:
    metadata = assert_regular_file(path)
    assert metadata.st_mode & stat.S_IXUSR, f"expected owner-executable bit: {path}"


def assert_initialized_git_checkout(path: Path) -> None:
    checkout_metadata = path.lstat()
    assert stat.S_ISDIR(checkout_metadata.st_mode), (
        f"expected a checkout directory: {path}"
    )
    git_metadata = (path / ".git").lstat()
    assert stat.S_ISREG(git_metadata.st_mode) or stat.S_ISDIR(git_metadata.st_mode), (
        f"expected regular Git metadata, not a symlink: {path / '.git'}"
    )


def assert_below(root: Path, path: Path) -> None:
    assert path.resolve().is_relative_to(root.resolve()), (
        f"path escaped temporary fixture: {path}"
    )


def sanitized_environment(
    temp_root: Path, overrides: Mapping[str, str] | None = None
) -> dict[str, str]:
    root = temp_root.resolve()
    assert root.is_dir()
    environment = {
        "HOME": str(root / "home"),
        "PATH": SAFE_PATH,
        "LANG": "C",
        "LC_ALL": "C",
        "TZ": "UTC",
        "TMPDIR": str(root / "tmp"),
        "XDG_CACHE_HOME": str(root / "xdg-cache"),
        "XDG_CONFIG_HOME": str(root / "xdg-config"),
        "XDG_DATA_HOME": str(root / "xdg-data"),
        "PYTHONNOUSERSITE": "1",
        "PYTHONSAFEPATH": "1",
        "GIT_CONFIG_NOSYSTEM": "1",
        "GIT_CONFIG_SYSTEM": "/dev/null",
        "GIT_CONFIG_GLOBAL": "/dev/null",
        "GIT_ATTR_NOSYSTEM": "1",
        "GIT_TERMINAL_PROMPT": "0",
        "GIT_ASKPASS": "/bin/false",
        "GIT_SSH_COMMAND": "/bin/false",
        "GIT_ALLOW_PROTOCOL": "file",
        "GIT_PROTOCOL_FROM_USER": "0",
        "GIT_PAGER": "cat",
        "PAGER": "cat",
        "GIT_LFS_SKIP_SMUDGE": "1",
        "GIT_CONFIG_COUNT": str(len(SAFE_GIT_CONFIG)),
    }
    for index, (key, value) in enumerate(SAFE_GIT_CONFIG):
        environment[f"GIT_CONFIG_KEY_{index}"] = key
        environment[f"GIT_CONFIG_VALUE_{index}"] = value
    if overrides:
        unexpected = set(overrides) - ALLOWED_ENV_OVERRIDES
        assert not unexpected, f"unsafe environment overrides: {sorted(unexpected)}"
        for name in ("PYTHON", "PYTHON_EXECUTABLE", "PSI0_SONIC_CLIENT_PYTHON"):
            if name in overrides:
                assert_below(root, Path(overrides[name]))
        if "PATH" in overrides:
            path_entries = overrides["PATH"].split(":")
            safe_entries = SAFE_PATH.split(":")
            assert path_entries[-len(safe_entries) :] == safe_entries
            for entry in path_entries[: -len(safe_entries)]:
                assert_below(root, Path(entry))
        environment.update(overrides)
    assert PROHIBITED_AMBIENT_ENVIRONMENT.isdisjoint(environment)
    for directory_name in ("home", "tmp", "xdg-cache", "xdg-config", "xdg-data"):
        directory = root / directory_name
        assert_below(root, directory)
        directory.mkdir(exist_ok=True)
    return environment


def process_diagnostics(result: subprocess.CompletedProcess, cwd: Path) -> str:
    return (
        f"command: {result.args!r}\n"
        f"cwd: {cwd}\n"
        f"returncode: {result.returncode}\n"
        f"stdout: {result.stdout!r}\n"
        f"stderr: {result.stderr!r}"
    )


def assert_allowed_working_directory(
    command: list[str], cwd: Path, temp_root: Path
) -> None:
    resolved_cwd = cwd.resolve()
    if resolved_cwd.is_relative_to(temp_root.resolve()):
        return
    assert resolved_cwd.is_relative_to(REPO_ROOT.resolve()), (
        f"subprocess cwd escaped fixture and trusted checkout: {resolved_cwd}"
    )
    assert command[0] == "git" and command[1] in READ_ONLY_WORKTREE_GIT_COMMANDS, (
        f"non-read-only subprocess outside temporary fixture: {command!r}"
    )
    if command[1] == "config":
        assert "--get-all" in command or "--get-regexp" in command


def run(
    *args: str | Path,
    temp_root: Path,
    cwd: Path = REPO_ROOT,
    env_overrides: Mapping[str, str] | None = None,
    check: bool = True,
    text: bool = True,
    timeout_seconds: int = SUBPROCESS_TIMEOUT_SECONDS,
) -> subprocess.CompletedProcess:
    command = [str(arg) for arg in args]
    assert 0 < timeout_seconds <= SHARED_CHECKOUT_TIMEOUT_SECONDS
    assert_allowed_working_directory(command, cwd, temp_root)
    try:
        result = subprocess.run(
            command,
            cwd=cwd,
            env=sanitized_environment(temp_root, env_overrides),
            check=False,
            capture_output=True,
            text=text,
            timeout=timeout_seconds,
        )
    except subprocess.TimeoutExpired as error:
        raise AssertionError(
            f"command timed out after {timeout_seconds}s\n"
            f"command: {command!r}\n"
            f"cwd: {cwd}\n"
            f"stdout: {error.stdout!r}\n"
            f"stderr: {error.stderr!r}"
        ) from error
    if check and result.returncode != 0:
        raise AssertionError(process_diagnostics(result, cwd))
    return result


def git_run(
    *args: str | Path,
    temp_root: Path,
    cwd: Path = REPO_ROOT,
    check: bool = True,
    text: bool = True,
    timeout_seconds: int = SUBPROCESS_TIMEOUT_SECONDS,
) -> subprocess.CompletedProcess:
    return run(
        "git",
        *args,
        temp_root=temp_root,
        cwd=cwd,
        check=check,
        text=text,
        timeout_seconds=timeout_seconds,
    )


def assert_git_config_singleton(
    config: Path, key: str, expected_value: str, temp_root: Path
) -> None:
    result = git_run(
        "config", "-f", config, "--get-all", key, temp_root=temp_root, check=False
    )
    diagnostics = process_diagnostics(result, REPO_ROOT)
    assert result.returncode == 0, diagnostics
    assert result.stdout.splitlines() == [expected_value], diagnostics


def git_config_entries(
    config: Path, pattern: str, temp_root: Path
) -> list[tuple[str, str]]:
    result = git_run(
        "config",
        "-f",
        config,
        "--get-regexp",
        pattern,
        temp_root=temp_root,
    )
    return [tuple(line.split(maxsplit=1)) for line in result.stdout.splitlines()]


def assert_index_entry(
    path: Path,
    expected_mode: str,
    temp_root: Path,
    expected_object_id: str | None = None,
) -> None:
    relative_path = path.relative_to(REPO_ROOT).as_posix()
    result = git_run("ls-files", "--stage", "--", relative_path, temp_root=temp_root)
    rows = result.stdout.splitlines()
    assert len(rows) == 1, process_diagnostics(result, REPO_ROOT)
    metadata, indexed_path = rows[0].split("\t", maxsplit=1)
    mode, object_id, stage_number = metadata.split()
    diagnostics = process_diagnostics(result, REPO_ROOT)
    assert mode == expected_mode, diagnostics
    if expected_object_id is not None:
        assert object_id == expected_object_id, diagnostics
    assert stage_number == "0", diagnostics
    assert indexed_path == relative_path, diagnostics


def assert_pristine_checkout(repo: Path, temp_root: Path) -> None:
    status = git_run(
        "status",
        "--porcelain=v1",
        "--untracked-files=all",
        cwd=repo,
        temp_root=temp_root,
    )
    assert status.stdout == "", process_diagnostics(status, repo)
    unstaged = git_run(
        "diff",
        "--quiet",
        "--no-ext-diff",
        "--no-textconv",
        "--",
        cwd=repo,
        temp_root=temp_root,
        check=False,
    )
    assert unstaged.returncode == 0, process_diagnostics(unstaged, repo)
    staged = git_run(
        "diff",
        "--cached",
        "--quiet",
        "--no-ext-diff",
        "--no-textconv",
        "--",
        cwd=repo,
        temp_root=temp_root,
        check=False,
    )
    assert staged.returncode == 0, process_diagnostics(staged, repo)
    untracked = git_run(
        "ls-files",
        "--others",
        "--exclude-standard",
        cwd=repo,
        temp_root=temp_root,
    )
    assert untracked.stdout == "", process_diagnostics(untracked, repo)


def canonical_patch_bytes(
    repo: Path, patched_file: Path, comparison_root: Path, temp_root: Path
) -> bytes:
    assert_below(temp_root, comparison_root)
    pristine = git_run(
        "cat-file",
        "blob",
        f"{PIN}:{TARGET_HEADER.as_posix()}",
        cwd=repo,
        temp_root=temp_root,
        text=False,
    ).stdout
    assert hashlib.sha256(pristine).hexdigest() == PRISTINE_HEADER_SHA256
    patched = read_bytes(patched_file)

    pristine_path = comparison_root / "a" / TARGET_HEADER
    patched_path = comparison_root / "b" / TARGET_HEADER
    for path, contents in ((pristine_path, pristine), (patched_path, patched)):
        assert_below(temp_root, path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(contents)
        path.chmod(0o644)

    result = git_run(
        "diff",
        "--no-index",
        *CANONICAL_DIFF_OPTIONS,
        "--no-prefix",
        "--",
        pristine_path.relative_to(comparison_root),
        patched_path.relative_to(comparison_root),
        cwd=comparison_root,
        temp_root=temp_root,
        check=False,
        text=False,
    )
    assert result.returncode == 1, process_diagnostics(result, comparison_root)
    assert result.stderr == b"", process_diagnostics(result, comparison_root)
    return result.stdout


def repository_diff_bytes(repo: Path, temp_root: Path) -> bytes:
    result = git_run(
        "diff",
        *CANONICAL_DIFF_OPTIONS,
        "--src-prefix=a/",
        "--dst-prefix=b/",
        "--",
        cwd=repo,
        temp_root=temp_root,
        text=False,
    )
    assert result.stderr == b"", process_diagnostics(result, repo)
    return result.stdout


def assert_only_reviewed_patch(
    repo: Path,
    expected_patch: bytes,
    comparison_root: Path,
    temp_root: Path,
) -> bytes:
    actual_diff = repository_diff_bytes(repo, temp_root)
    assert actual_diff == expected_patch
    attribute_independent_diff = canonical_patch_bytes(
        repo, repo / TARGET_HEADER, comparison_root, temp_root
    )
    assert attribute_independent_diff == expected_patch
    changed = git_run(
        "diff",
        "--name-only",
        "--no-ext-diff",
        "--no-textconv",
        "--no-renames",
        "--",
        cwd=repo,
        temp_root=temp_root,
    )
    assert changed.stdout.splitlines() == [TARGET_HEADER.as_posix()], (
        process_diagnostics(changed, repo)
    )
    staged = git_run(
        "diff",
        "--cached",
        "--quiet",
        "--no-ext-diff",
        "--no-textconv",
        "--",
        cwd=repo,
        temp_root=temp_root,
        check=False,
    )
    assert staged.returncode == 0, process_diagnostics(staged, repo)
    untracked = git_run(
        "ls-files",
        "--others",
        "--exclude-standard",
        cwd=repo,
        temp_root=temp_root,
    )
    assert untracked.stdout == "", process_diagnostics(untracked, repo)
    assert sha256(repo / TARGET_HEADER) == PATCHED_HEADER_SHA256
    return actual_diff


def test_official_gr00t_submodule_pin(tmp_path: Path) -> None:
    config = REPO_ROOT / ".gitmodules"
    config_text = read_bytes(config).decode("utf-8")
    subsection = "third_party/GR00T-WholeBodyControl"
    section = f"submodule.{subsection}"
    target_path = "third_party/GR00T-WholeBodyControl"
    official_url = "https://github.com/NVlabs/GR00T-WholeBodyControl.git"
    psi_fork = "physical-superintelligence-lab/GR00T-WholeBodyControl"

    assert_git_config_singleton(config, f"{section}.path", target_path, tmp_path)
    assert_git_config_singleton(config, f"{section}.url", official_url, tmp_path)
    assert_git_config_singleton(config, f"{section}.branch", "main", tmp_path)

    target_entries = git_config_entries(config, rf"^{re.escape(section)}\.", tmp_path)
    assert len(target_entries) == 3
    assert {key for key, _ in target_entries} == {
        f"{section}.path",
        f"{section}.url",
        f"{section}.branch",
    }
    subsection_headers = re.findall(
        r'^\s*\[\s*submodule\s+"([^"]+)"\s*\]\s*(?:[#;].*)?$',
        config_text,
        flags=re.MULTILINE | re.IGNORECASE,
    )
    assert subsection_headers.count(subsection) == 1

    path_entries = git_config_entries(config, r"^submodule\..*\.path$", tmp_path)
    normalized_target_path = posixpath.normpath(target_path)
    assert [
        (key, value)
        for key, value in path_entries
        if posixpath.normpath(value) == normalized_target_path
    ] == [(f"{section}.path", target_path)]
    url_entries = git_config_entries(config, r"^submodule\..*\.url$", tmp_path)
    assert all(psi_fork not in value for _, value in url_entries)

    assert_index_entry(GR00T_PATH, "160000", tmp_path, expected_object_id=PIN)


def test_reviewed_patch_and_installer_hashes() -> None:
    assert sha256(PATCH_PATH) == PATCH_SHA256
    assert sha256(INSTALLER_PATH) == INSTALLER_SHA256
    assert_executable_file(INSTALLER_PATH)


def test_vendored_client_identity_and_provenance(tmp_path: Path) -> None:
    assert sha256(CLIENT_PATH) == CLIENT_SHA256
    assert json.loads(read_bytes(PROVENANCE_PATH).decode("utf-8")) == {
        "artifact": "psi_rtc_sonic_client.py",
        "source_repository": "https://github.com/physical-superintelligence-lab/GR00T-WholeBodyControl.git",
        "source_commit": "d40376994ff094787591ab76a691c50b8e131786",
        "source_path": "psi_rtc_sonic_client.py",
        "sha256": CLIENT_SHA256,
        "license": "Apache-2.0",
        "license_source": "https://github.com/physical-superintelligence-lab/GR00T-WholeBodyControl/blob/d40376994ff094787591ab76a691c50b8e131786/LICENSE",
        "future_pythonpath": "third_party/GR00T-WholeBodyControl",
    }

    pycache = tmp_path / "pycache"
    compiled = run(
        sys.executable,
        "-I",
        "-S",
        "-X",
        f"pycache_prefix={pycache}",
        "-m",
        "py_compile",
        CLIENT_PATH,
        cwd=tmp_path,
        temp_root=tmp_path,
    )
    assert compiled.stdout == "", process_diagnostics(compiled, tmp_path)
    assert compiled.stderr == "", process_diagnostics(compiled, tmp_path)
    compiled_files = list(pycache.rglob("*.pyc"))
    assert compiled_files
    assert all(path.resolve().is_relative_to(tmp_path) for path in compiled_files)


def test_patch_output_from_clean_pinned_archive(tmp_path: Path) -> None:
    assert_initialized_git_checkout(GR00T_PATH)
    revision = git_run("rev-parse", "HEAD", cwd=GR00T_PATH, temp_root=tmp_path)
    assert revision.stdout.strip() == PIN, process_diagnostics(revision, GR00T_PATH)
    archive = git_run(
        "archive",
        PIN,
        "--",
        TARGET_HEADER,
        cwd=GR00T_PATH,
        temp_root=tmp_path,
        text=False,
    ).stdout
    archive_root = tmp_path / "archive"
    target = archive_root / TARGET_HEADER
    assert_below(tmp_path, target)
    with tarfile.open(fileobj=io.BytesIO(archive), mode="r:") as tar:
        members = tar.getmembers()
        members_by_name = {member.name: member for member in members}
        expected_names = {*EXPECTED_ARCHIVE_DIRECTORIES, TARGET_HEADER.as_posix()}
        assert len(members_by_name) == len(members), "duplicate archive member names"
        assert set(members_by_name) == expected_names
        for directory_name in EXPECTED_ARCHIVE_DIRECTORIES:
            assert members_by_name[directory_name].isdir()
        target_member = members_by_name[TARGET_HEADER.as_posix()]
        assert target_member.isfile()
        extracted = tar.extractfile(target_member)
        assert extracted is not None
        pristine_contents = extracted.read()
        assert len(pristine_contents) == target_member.size
    target.parent.mkdir(parents=True)
    target.write_bytes(pristine_contents)
    assert sha256(target) == PRISTINE_HEADER_SHA256
    assert sha256(PATCH_PATH) == PATCH_SHA256
    git_run("apply", "--check", PATCH_PATH, cwd=archive_root, temp_root=tmp_path)
    git_run("apply", PATCH_PATH, cwd=archive_root, temp_root=tmp_path)
    assert sha256(target) == PATCHED_HEADER_SHA256
    git_run(
        "apply",
        "--reverse",
        "--check",
        PATCH_PATH,
        cwd=archive_root,
        temp_root=tmp_path,
    )


def test_installer_isolated_exact_and_idempotent(tmp_path: Path) -> None:
    fixture = (tmp_path / "parent").resolve()
    fixture.mkdir()
    fixture_patch = fixture / PATCH_PATH.relative_to(REPO_ROOT)
    fixture_installer = fixture / INSTALLER_PATH.relative_to(REPO_ROOT)
    fixture_gr00t = fixture / GR00T_PATH.relative_to(REPO_ROOT)
    fixture_patch.parent.mkdir(parents=True)
    fixture_installer.parent.mkdir(parents=True)
    fixture_gr00t.parent.mkdir(parents=True)
    for path in (fixture_patch, fixture_installer, fixture_gr00t):
        assert_below(fixture, path)

    assert sha256(PATCH_PATH) == PATCH_SHA256
    assert sha256(INSTALLER_PATH) == INSTALLER_SHA256
    assert_executable_file(INSTALLER_PATH)
    assert_initialized_git_checkout(GR00T_PATH)
    shutil.copy2(PATCH_PATH, fixture_patch)
    shutil.copy2(INSTALLER_PATH, fixture_installer)
    assert sha256(fixture_patch) == PATCH_SHA256
    assert sha256(fixture_installer) == INSTALLER_SHA256
    assert_executable_file(fixture_installer)

    git_run(
        "clone",
        "--shared",
        "--no-checkout",
        "--",
        GR00T_PATH,
        fixture_gr00t,
        cwd=fixture,
        temp_root=tmp_path,
        timeout_seconds=SHARED_CHECKOUT_TIMEOUT_SECONDS,
    )
    git_run(
        "checkout",
        "--detach",
        PIN,
        cwd=fixture_gr00t,
        temp_root=tmp_path,
        timeout_seconds=SHARED_CHECKOUT_TIMEOUT_SECONDS,
    )
    revision = git_run("rev-parse", "HEAD", cwd=fixture_gr00t, temp_root=tmp_path)
    assert revision.stdout.strip() == PIN, process_diagnostics(revision, fixture_gr00t)
    symbolic_ref = git_run(
        "symbolic-ref",
        "--quiet",
        "HEAD",
        cwd=fixture_gr00t,
        temp_root=tmp_path,
        check=False,
    )
    assert symbolic_ref.returncode == 1, process_diagnostics(
        symbolic_ref, fixture_gr00t
    )
    assert_pristine_checkout(fixture_gr00t, tmp_path)

    first = run(fixture_installer, cwd=fixture, temp_root=tmp_path)
    assert first.stdout == "[gr00t-v1.1] applied streamed-mode start fix\n", (
        process_diagnostics(first, fixture)
    )
    assert first.stderr == "", process_diagnostics(first, fixture)
    expected_patch = read_bytes(fixture_patch)
    first_diff = assert_only_reviewed_patch(
        fixture_gr00t,
        expected_patch,
        tmp_path / "canonical-diff",
        tmp_path,
    )
    first_status = git_run(
        "status",
        "--porcelain=v1",
        "--untracked-files=all",
        cwd=fixture_gr00t,
        temp_root=tmp_path,
    ).stdout
    first_header_hash = sha256(fixture_gr00t / TARGET_HEADER)

    second = run(fixture_installer, cwd=fixture, temp_root=tmp_path)
    assert second.stdout == (
        "[gr00t-v1.1] streamed-mode start fix is already applied\n"
    ), process_diagnostics(second, fixture)
    assert second.stderr == "", process_diagnostics(second, fixture)
    second_diff = assert_only_reviewed_patch(
        fixture_gr00t,
        expected_patch,
        tmp_path / "canonical-diff",
        tmp_path,
    )
    assert second_diff == first_diff
    second_status = git_run(
        "status",
        "--porcelain=v1",
        "--untracked-files=all",
        cwd=fixture_gr00t,
        temp_root=tmp_path,
    ).stdout
    assert second_status == first_status
    assert sha256(fixture_gr00t / TARGET_HEADER) == first_header_hash


def test_supported_launcher_refuses_without_invoking_python(tmp_path: Path) -> None:
    assert read_bytes(LAUNCHER_PATH) == REFUSAL_SCRIPT_BYTES
    assert_executable_file(LAUNCHER_PATH)

    fixture_launcher = tmp_path / LAUNCHER_PATH.name
    shutil.copy2(LAUNCHER_PATH, fixture_launcher)
    assert read_bytes(fixture_launcher) == REFUSAL_SCRIPT_BYTES
    assert_executable_file(fixture_launcher)

    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    sentinel = tmp_path / "python-invoked"
    fake_python = fake_bin / "python"
    fake_python.write_bytes(b"#!/usr/bin/env bash\n: > python-invoked\nexit 99\n")
    fake_python.chmod(0o755)
    assert_executable_file(fake_python)

    result = run(
        fixture_launcher,
        "--host",
        "example.invalid",
        "--port",
        "65535",
        cwd=tmp_path,
        temp_root=tmp_path,
        env_overrides={
            "PATH": f"{fake_bin}:{SAFE_PATH}",
            "PYTHON": str(fake_python),
            "PYTHON_EXECUTABLE": str(fake_python),
            "PSI0_SONIC_CLIENT_PYTHON": str(fake_python),
        },
        check=False,
    )
    assert result.returncode == 78, process_diagnostics(result, tmp_path)
    assert result.stdout == "", process_diagnostics(result, tmp_path)
    assert result.stderr == REFUSAL_MESSAGE, process_diagnostics(result, tmp_path)
    assert not sentinel.exists()
