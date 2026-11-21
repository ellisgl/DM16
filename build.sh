#!/usr/bin/env bash
# Build DM16 with every line of the build log timestamped. Retries when the
# Gowin license server doesn't answer.
#
#   ./build.sh            # Tang Console 138K -> impl/pnr/DM16.fs, build.log
#   ./build.sh nano20k    # Tang Nano 20K     -> build/nano20k/impl/pnr/DM16.fs, build/nano20k/build.log
#   ./build.sh nano9k     # Tang Nano 9K      -> build/nano9k/impl/pnr/DM16.fs, build/nano9k/build.log
#
# GOWIN_HOME defaults to ~/Gowin_V1.9.12.04_linux.
set -u
cd "$(dirname "$0")"
ROOT=$(pwd)

BOARD=${1:-console138k}
case "$BOARD" in
    console138k) DIR=. ;;
    nano20k|nano9k) DIR=build/$BOARD ;;
    *) echo "unknown board '$BOARD' (console138k, nano20k or nano9k)"; exit 2 ;;
esac
mkdir -p "$DIR" && cd "$DIR"
export DM16_BOARD=$BOARD

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
        "$GOWIN_HOME/IDE/bin/gw_sh" "$ROOT/build.tcl" 2>&1   # no stdbuf: it replaces LD_PRELOAD
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
