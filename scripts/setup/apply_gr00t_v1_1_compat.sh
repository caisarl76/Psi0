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
