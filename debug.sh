#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
# Debug profile enables numeric telemetry. Release upload.sh stays unchanged.
cargo v5 build --locked
cargo v5 run --path "." \
  --file "${CARGO_TARGET_DIR:-$PWD/target}/armv7a-vex-v5/debug/override-competition" \
  --name "competition-debug" --slot 1 2>&1 | tee "${DEBUG_LOG_FILE:-debug-run.log}"
