#!/usr/bin/env bash
#
# Run the replay binary against every committed fixture and byte-compare its
# stdout to the paired *.golden file.  Fails on any drift.
#
# This is the current regression gate for the Ada control software + FFI
# surface, runnable on any headless machine with the toolchain available.
#
# When Ada behaviour changes intentionally, regenerate goldens with
# `scripts/test-replay-fixtures.sh --regenerate`.
#
# Why a shell script instead of `cargo test`: every target in the simulator
# crate pulls in libMars_Rover.a through build.rs, so an integration test
# would need to supply its own FFI stubs to link.  A shell driver sidesteps
# this until the workspace split lands (at which point a proper cargo test
# can live in the Bevy-free core crate).

set -euo pipefail

MANIFEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE_DIR="$(cd "$MANIFEST_DIR/.." && pwd)"
FIXTURES_DIR="$MANIFEST_DIR/tests/fixtures"
REPLAY="$WORKSPACE_DIR/target/release/replay"

regenerate=0
if [[ "${1:-}" == "--regenerate" ]]; then
    regenerate=1
fi

if [[ ! -x "$REPLAY" ]]; then
    echo "error: $REPLAY not found; run 'cargo build --release -p mars-rover-replay' first" >&2
    exit 2
fi

cd "$MANIFEST_DIR"

fail=0
pass=0
mapfile -t csvs < <(find tests/fixtures -maxdepth 1 -name '*.csv' | sort)
if [[ ${#csvs[@]} -eq 0 ]]; then
    echo "error: no *.csv fixtures in tests/fixtures/" >&2
    exit 2
fi

for csv in "${csvs[@]}"; do
    name=$(basename "$csv" .csv)
    golden="${csv%.csv}.golden"

    if [[ $regenerate -eq 1 ]]; then
        "$REPLAY" "$csv" > "$golden" 2>/dev/null || true
        echo "regenerated: $name"
        continue
    fi

    if [[ ! -f "$golden" ]]; then
        echo "FAIL $name  (golden missing: $golden)"
        fail=$((fail + 1))
        continue
    fi

    actual=$("$REPLAY" "$csv" 2>/dev/null || true)
    expected=$(cat "$golden")

    if [[ "$actual" == "$expected" ]]; then
        echo "PASS $name"
        pass=$((pass + 1))
    else
        echo "FAIL $name  (stdout differs from $golden)"
        diff <(printf '%s\n' "$expected") <(printf '%s\n' "$actual") | head -20 | sed 's/^/  /'
        fail=$((fail + 1))
    fi
done

if [[ $regenerate -eq 1 ]]; then
    exit 0
fi

echo
echo "Summary: $pass pass, $fail fail"
exit $(( fail > 0 ? 1 : 0 ))
