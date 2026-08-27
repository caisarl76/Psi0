#!/usr/bin/env bash
set -euo pipefail

printf '%s\n' \
    '[psi0-sonic] REFUSED: emergency-stop/receipt gate not implemented; RTC publication is disabled.' >&2
exit 78
