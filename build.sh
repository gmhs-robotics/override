#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
cargo v5 build --target thumbv7a-vex-v5 --release --locked
