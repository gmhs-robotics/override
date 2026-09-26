#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
./build.sh
cargo v5 upload --path "." \
  --file "$PWD/target/armv7a-vex-v5/release/override-competition" \
  --name "override-competition" --slot 1 --after none
