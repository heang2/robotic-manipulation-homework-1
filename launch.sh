#!/usr/bin/env bash
set -eu
export FFMPEG="${FFMPEG:-ffmpeg}"
export KIT_WINDOW=0
if test -n "${PRIVATE_XVFB:-}"; then
    export DISPLAY=":${DISPLAY_NUMBER:-91}"
    export XAUTHORITY="$PWD/display.auth"
    touch "$XAUTHORITY"; chmod 600 "$XAUTHORITY"
    xauth -f "$XAUTHORITY" add "$DISPLAY" MIT-MAGIC-COOKIE-1 "$(python3 -c 'import secrets;print(secrets.token_hex(16))')"
    "$PRIVATE_XVFB" "$DISPLAY" -screen 0 1600x1000x24 -nolisten tcp -auth "$XAUTHORITY" > display.log 2>&1 &
    server=$!
    trap 'kill "$server" 2>/dev/null || true' EXIT
    for i in $(seq 1 12); do
        if xdpyinfo -display "$DISPLAY" >/dev/null 2>&1; then export KIT_WINDOW=1 PRIVATE_DISPLAY=1; break; fi
        sleep .4
    done
fi
"${ISAAC_PYTHON:-$HOME/isaacsim/python.sh}" arm.py > trial.log 2>&1
grep -q TRIAL_COMPLETE trial.log

