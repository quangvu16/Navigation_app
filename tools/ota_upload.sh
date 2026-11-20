#!/usr/bin/env bash
# ota_upload.sh - compile the sketch and push it over the air with
# ArduinoOTA (the LAN alternative to the app's URL-pull OTA, WFO1;
# docs/PROTOCOL.md §3.10).
#
# usage: tools/ota_upload.sh [sketch-dir] <host-or-ip> [fqbn]
#   sketch-dir  default: firmware/vacuum_phone_esp32 next to this repo root
#   host        mDNS name (hoverboard.local) or DHCP IP, on the SAME LAN
#   fqbn        default: esp32:esp32:esp32   (use ...:esp32s3 for the S3)
#
# OTA_PASSWORD is read from the sketch's secrets.h - the same password
# the board enforces. The robot must stand still: while it moves, espota
# times out on purpose (ota.h, safety rule 2).
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SKETCH="${1:-$ROOT/firmware/vacuum_phone_esp32}"
HOST="${2:-}"
FQBN="${3:-esp32:esp32:esp32}"

if [[ -z "$HOST" ]]; then
  echo "usage: $0 [sketch-dir] <host-or-ip> [fqbn]" >&2
  exit 2
fi
if [[ ! -f "$SKETCH/secrets.h" ]]; then
  echo "error: $SKETCH/secrets.h not found (copy secrets.h.example)" >&2
  exit 1
fi

# macOS grep has no -P; sed keeps this portable.
OTA_PASS="$(sed -n 's/.*#define OTA_PASSWORD[[:space:]]*"\([^"]*\)".*/\1/p' \
  "$SKETCH/secrets.h" | head -1)"
if [[ -z "$OTA_PASS" || "$OTA_PASS" == "choose-a-long-password" ]]; then
  echo "error: set a real OTA_PASSWORD in $SKETCH/secrets.h" >&2
  exit 1
fi

CLI="${ARDUINO_CLI:-$HOME/.cache/vacuum-nav/arduino/arduino-cli}"
CFG="${ARDUINO_CLI_CONFIG:-$HOME/Library/Arduino15/arduino-cli.yaml}"
[[ -x "$CLI" ]] || { echo "error: arduino-cli not at $CLI" >&2; exit 1; }

OUT="$(mktemp -d /tmp/vacuum-ota.XXXXXX)"
trap 'rm -rf "$OUT"' EXIT
echo "==> compiling $SKETCH ($FQBN)"
"$CLI" compile --config-file "$CFG" --fqbn "$FQBN" --output-dir "$OUT" "$SKETCH"

BIN="$(ls -1 "$OUT"/*.ino.bin 2>/dev/null | head -1)"
[[ -n "$BIN" ]] || { echo "error: no .ino.bin in $OUT" >&2; exit 1; }

ESPOTA="$(find "${ARDUINO15:-$HOME/Library/Arduino15}/packages/esp32" \
  -name espota.py 2>/dev/null | head -1)"
[[ -n "$ESPOTA" ]] || { echo "error: espota.py not found (ESP32 core?)" >&2; exit 1; }

echo "==> pushing $(basename "$BIN") to $HOST:3232"
python3 "$ESPOTA" -i "$HOST" -p 3232 -a "$OTA_PASS" -f "$BIN" --progress
echo "==> done (board reboots into the new image)"
