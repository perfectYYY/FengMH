#!/usr/bin/env bash
set -e
SCRIPT_PATH="$(readlink -f "$0")"
SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$SCRIPT_PATH")" && pwd)"
exec python3 "$SCRIPT_DIR/mission_host.py" --config "$SCRIPT_DIR/mission_config.json" "$@"
