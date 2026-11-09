#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
cargo-fmt --package override-competition -- --check
cargo test --locked --bin override-competition
cargo check --target thumbv7a-vex-v5 --locked
