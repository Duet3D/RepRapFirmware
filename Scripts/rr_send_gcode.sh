#!/usr/bin/env bash
set -euo pipefail

DUET_HOST="${1:-}"
DUET_PASSWORD=""
GCODE=""

# Support both forms:
#   rr_send_gcode.sh <host> <gcode>
#   rr_send_gcode.sh <host> <password> <gcode>
if [[ "$#" -eq 2 ]]; then
    DUET_PASSWORD="reprap"
	GCODE="$2"
elif [[ "$#" -ge 3 ]]; then
	DUET_PASSWORD="$2"
	GCODE="$3"
fi

if [[ -z "$DUET_HOST" || -z "$GCODE" ]]; then
	echo "Usage: $0 <duet-host> [duet-password] <gcode>" >&2
	exit 2
fi

# VS Code task input may provide an empty password as "" or whitespace.
# Normalize these cases so password= is omitted unless a real value is present.
if [[ "$DUET_PASSWORD" == '""' ]]; then
	DUET_PASSWORD=""
fi
DUET_PASSWORD="${DUET_PASSWORD//[[:space:]]/}"

BASE_URL="http://$DUET_HOST"
COOKIE_FILE="$(mktemp)"
trap 'rm -f "$COOKIE_FILE"' EXIT

if [[ -n "$DUET_PASSWORD" ]]; then
	CONNECT_RESPONSE="$(curl --fail --silent --show-error --cookie-jar "$COOKIE_FILE" --url-query "password=$DUET_PASSWORD" "$BASE_URL/rr_connect")"
else
	CONNECT_RESPONSE="$(curl --fail --silent --show-error --cookie-jar "$COOKIE_FILE" "$BASE_URL/rr_connect")"
fi

if ! printf '%s' "$CONNECT_RESPONSE" | grep -Eq '"err"[[:space:]]*:[[:space:]]*0'; then
	echo "rr_connect failed: $CONNECT_RESPONSE" >&2
	exit 1
fi

echo "Sending G-code '$GCODE' to $DUET_HOST"
GCODE_RESPONSE="$(curl --fail --silent --show-error --cookie "$COOKIE_FILE" --url-query "gcode=$GCODE" "$BASE_URL/rr_gcode")"
echo "rr_gcode response: $GCODE_RESPONSE"

# This may fail if command triggered a reboot (e.g. M997), so keep it non-fatal.
curl --silent --show-error --cookie "$COOKIE_FILE" "$BASE_URL/rr_disconnect" >/dev/null || true
