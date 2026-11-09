#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
# Debug profile enables numeric telemetry.
cargo v5 run --name competition-debug --slot 1 --target thumbv7a-vex-v5 --locked 2>&1 | tee "${DEBUG_LOG_FILE:-debug-run.log}"
