#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
cargo v5 upload --name override-competition --slot 1 --target thumbv7a-vex-v5 --release --locked
