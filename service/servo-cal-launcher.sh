#!/bin/sh
set -eu

ROOT=${SERVO_CAL_ROOT:-@SERVO_CAL_ROOT@}
CAPTURES=${SERVO_CAL_DIR:-"$HOME/printer_data/logs/servo_captures"}
PORT=${SERVO_CAL_PORT:-8085}
HOST=${SERVO_CAL_HOST:-0.0.0.0}
BINARY="$ROOT/target/snapshot/servo-cal"
BUILD_HEAD="$BINARY.head"

export PATH="$HOME/.bun/bin:$PATH"

while :; do
    head=$(git -C "$ROOT" rev-parse HEAD)
    if [ -x "$BINARY" ] && [ -f "$BUILD_HEAD" ] && [ "$(cat "$BUILD_HEAD")" = "$head" ]; then
        exec "$BINARY" serve --dir "$CAPTURES" --port "$PORT" --host "$HOST"
    fi

    if cargo build --profile snapshot -p servo-ident --manifest-path "$ROOT/Cargo.toml"; then
        printf '%s\n' "$head" > "$BUILD_HEAD"
        exec "$BINARY" serve --dir "$CAPTURES" --port "$PORT" --host "$HOST"
    fi

    echo "servo-cal: build failed; retrying in 2 seconds" >&2
    sleep 2
done
