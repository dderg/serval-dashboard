#!/bin/sh
set -eu

usage() {
    echo "usage: $0 [--klipper <dir>] [--no-service]" >&2
    exit 2
}

escape_sed() {
    printf '%s' "$1" | sed 's/[&|\\]/\\&/g'
}

ROOT=$(CDPATH='' cd -- "$(dirname -- "$0")" && pwd)
KLIPPER="$HOME/klipper"
INSTALL_SERVICE=1

while [ "$#" -gt 0 ]; do
    case "$1" in
        --klipper)
            [ "$#" -ge 2 ] || usage
            KLIPPER=$2
            shift 2
            ;;
        --no-service)
            INSTALL_SERVICE=0
            shift
            ;;
        --help)
            usage
            ;;
        *)
            usage
            ;;
    esac
done

EXTRAS="$ROOT/klippy_extras"
DESTINATION="$KLIPPER/klippy/extras"
[ -d "$EXTRAS" ] || {
    echo "servo-cal: missing plugin extras at $EXTRAS" >&2
    exit 1
}
[ -d "$DESTINATION" ] || {
    echo "servo-cal: Klipper extras directory does not exist: $DESTINATION" >&2
    exit 1
}

for entry in "$EXTRAS"/*; do
    [ -e "$entry" ] || [ -L "$entry" ] || {
        echo "servo-cal: no plugin extras found at $EXTRAS" >&2
        exit 1
    }
    case "$(basename "$entry")" in
    __pycache__ | *.pyc) continue ;;
    esac
    destination="$DESTINATION/$(basename "$entry")"
    if [ -L "$destination" ]; then
        rm "$destination"
    elif [ -d "$destination" ] && [ -z "$(find "$destination" -type f ! -name '*.pyc' | head -1)" ]; then
        rm -rf "$destination"
    elif [ -e "$destination" ]; then
        echo "servo-cal: refusing to replace non-symlink $destination" >&2
        exit 1
    fi
    ln -s "$entry" "$destination"
done

export PATH="$HOME/.bun/bin:$PATH"
if ! command -v cargo >/dev/null 2>&1; then
    echo "servo-cal: cargo is required; install Rust from https://rustup.rs/" >&2
    exit 1
fi
if ! command -v bun >/dev/null 2>&1; then
    echo "servo-cal: bun is required; install it from https://bun.sh/" >&2
    exit 1
fi

(
    cd "$ROOT"
    cargo build --profile snapshot -p servo-ident
)
git -C "$ROOT" rev-parse HEAD > "$ROOT/target/snapshot/servo-cal.head"

if [ "$INSTALL_SERVICE" -eq 1 ]; then
    case "$HOME" in
        /*) ;;
        *)
            echo "servo-cal: HOME must be an absolute path" >&2
            exit 1
            ;;
    esac
    SERVICE_HOME="$HOME/servo-cal"
    mkdir -p "$SERVICE_HOME"
    user=$(id -un)
    service_home_sed=$(escape_sed "$SERVICE_HOME")
    root_sed=$(escape_sed "$ROOT")
    user_sed=$(escape_sed "$user")
    sed -e "s|@SERVO_CAL_USER@|$user_sed|g" \
        -e "s|@SERVO_CAL_HOME@|$service_home_sed|g" \
        "$ROOT/service/servo-cal.service" > "$SERVICE_HOME/servo-cal.service"
    sed -e "s|@SERVO_CAL_ROOT@|$root_sed|g" \
        "$ROOT/service/servo-cal-launcher.sh" > "$SERVICE_HOME/servo-cal-launcher.sh"
    chmod +x "$SERVICE_HOME/servo-cal-launcher.sh"
    sudo systemctl enable "$SERVICE_HOME/servo-cal.service"
    sudo systemctl restart servo-cal
    systemctl status servo-cal --no-pager -n 5

    # servo capture retention: prune script + daily/boot timer
    CAPTURES_DIR="$HOME/printer_data/logs/servo_captures"
    captures_dir_sed=$(escape_sed "$CAPTURES_DIR")
    cp "$ROOT/scripts/servo-capture-prune" "$SERVICE_HOME/servo-capture-prune"
    chmod +x "$SERVICE_HOME/servo-capture-prune"
    sed -e "s|@SERVO_CAL_USER@|$user_sed|g" \
        -e "s|@SERVO_CAL_HOME@|$service_home_sed|g" \
        -e "s|@SERVO_CAPTURES_DIR@|$captures_dir_sed|g" \
        "$ROOT/service/servo-capture-prune.service" \
        > "$SERVICE_HOME/servo-capture-prune.service"
    cp "$ROOT/service/servo-capture-prune.timer" \
        "$SERVICE_HOME/servo-capture-prune.timer"
    sudo systemctl enable "$SERVICE_HOME/servo-capture-prune.service"
    sudo systemctl enable --now "$SERVICE_HOME/servo-capture-prune.timer"
    systemctl status servo-capture-prune.timer --no-pager -n 5
fi

cat <<EOF

Moonraker configuration hint (add this under [authorization]):
  cors_domains:
    http://<printer-host>:8085

Moonraker update-manager stanza:
[update_manager serval-dashboard]
type: git_repo
path: $ROOT
origin: https://github.com/dderg/serval-dashboard.git
install_script: install.sh
EOF
