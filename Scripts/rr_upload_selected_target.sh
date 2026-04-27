#!/usr/bin/env bash
set -euo pipefail

TARGET="${1:-}"
DUET_HOST="${2:-}"
DUET_PASSWORD="${3:-reprap}"

# VS Code task input may provide an empty password as "" or whitespace.
# Normalize these cases so password= is omitted unless a real value is present.
if [[ "$DUET_PASSWORD" == '""' ]]; then
	DUET_PASSWORD=""
fi
DUET_PASSWORD="${DUET_PASSWORD//[[:space:]]/}"

if [[ -z "$TARGET" || -z "$DUET_HOST" ]]; then
	echo "Usage: $0 <target> <duet-host> [duet-password]" >&2
	exit 2
fi

case "$TARGET" in
	Duet3_MB6HC)
		LOCAL_FILE="Duet3_MB6HC/Duet3Firmware_MB6HC.bin"
		REMOTE_FILE="/firmware/Duet3Firmware_MB6HC.bin"
		;;
	Duet3_MB6XD)
		LOCAL_FILE="Duet3_MB6XD/Duet3Firmware_MB6XD.bin"
		REMOTE_FILE="/firmware/Duet3Firmware_MB6XD.bin"
		;;
	Duet3Mini5plus)
		LOCAL_FILE="Duet3Mini5plus/Duet3Firmware_Mini5plus.bin"
		REMOTE_FILE="/firmware/Duet3Firmware_Mini5plus.bin"
		;;
	all)
		echo "Upload supports one board target at a time; please choose a specific target." >&2
		exit 1
		;;
	*)
		echo "Unsupported target: $TARGET" >&2
		exit 1
		;;
esac

if [[ ! -f "$LOCAL_FILE" ]]; then
	echo "Firmware file not found: $LOCAL_FILE" >&2
	echo "Build the selected target first." >&2
	exit 1
fi

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

echo "Uploading $LOCAL_FILE to $BASE_URL/rr_upload?name=$REMOTE_FILE"
UPLOAD_RESPONSE="$(curl --fail --silent --show-error --cookie "$COOKIE_FILE" --data-binary "@$LOCAL_FILE" "$BASE_URL/rr_upload?name=$REMOTE_FILE")"
if ! printf '%s' "$UPLOAD_RESPONSE" | grep -Eq '"err"[[:space:]]*:[[:space:]]*0'; then
	echo "rr_upload failed: $UPLOAD_RESPONSE" >&2
	exit 1
fi

curl --silent --show-error --cookie "$COOKIE_FILE" "$BASE_URL/rr_disconnect" >/dev/null || true

echo "Upload completed successfully."
