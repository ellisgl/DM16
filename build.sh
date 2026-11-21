#!/usr/bin/env bash
# Build DM16 (build.tcl -> impl/pnr/DM16.fs) with every line of build.log timestamped.
# Retries when the Gowin license server doesn't answer.
#
#   ./build.sh            # GOWIN_HOME defaults to ~/Gowin_V1.9.12.04_linux
set -u
cd "$(dirname "$0")"

GOWIN_HOME=${GOWIN_HOME:-$HOME/Gowin_V1.9.12.04_linux}
LOG=build.log
TRIES=4

# gw_sh runs headless only with these (see the Gowin CLI notes)
export QT_QPA_PLATFORM=minimal
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libfreetype.so.6

stamp() {
    gawk '{ print strftime("%Y-%m-%d %H:%M:%S"), $0; fflush() }'
}

start=$(date +%s)
for try in $(seq 1 $TRIES); do
    {
        echo "=== build start (try $try of $TRIES)"
        "$GOWIN_HOME/IDE/bin/gw_sh" build.tcl 2>&1   # no stdbuf: it replaces LD_PRELOAD
        echo "=== gw_sh exit status $?"
    } | stamp > "$LOG"
    grep -q "License verification failed" "$LOG" || break
    [ "$try" -lt "$TRIES" ] && echo "License server not responding; retrying in 30 s" && sleep 30
done

elapsed=$(( $(date +%s) - start ))
printf '%s === build end, %dm%02ds total\n' "$(date '+%Y-%m-%d %H:%M:%S')" \
    $((elapsed / 60)) $((elapsed % 60)) >> "$LOG"
tail -2 "$LOG"
grep -q "gw_sh exit status 0" "$LOG" && [ -f impl/pnr/DM16.fs ]
