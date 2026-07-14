#!/usr/bin/env bash
# Bootloader stress test for the Daisy keyboard.
#
# Cycles: app -> aster bootloader-jump -> mcuboot serial recovery (CDC ACM)
#         -> SMP probe (+ periodic full image upload) -> reset -> app.
#
# The post-enumeration delay before the first SMP command is varied each
# iteration (see --delays) to catch timing-dependent failures in the
# recovery USB bring-up (wait-for-DTR path, host open races, etc.).
#
# Usage: ./stress_bootloader.sh [options]
#   -n N              iterations (default 10)
#   --delays "..."    space-separated seconds, cycled per iteration
#                     (default "0 0.2 0.5 1 2 5")
#   --upload-every N  do a full image upload every Nth iteration, 0=never
#                     (default 3)
#   --image PATH      image for uploads (default build/app/zephyr/zmk.signed.bin)
#   --west-flash      west flash once before starting
#   --keep-going      don't stop on first failure
#   --chaos P         probability (0..1) that an upload gets interrupted by
#                     `nrfutil device reset` at a random point, to simulate a
#                     failed update. The bootloader must then either still have
#                     a valid app, or auto-enter recovery (BOOT_SERIAL_NO_APPLICATION)
#                     where a repair upload must succeed. (default 0)
#   --alt-image PATH  DIFFERENT image used for the interrupted chaos uploads
#                     (default build/zmk.signed.alt.bin). Must differ from
#                     --image: interrupting an upload of the identical binary
#                     rewrites the same bytes and can never invalidate slot0.
#                     Generate with: bump VERSION_TWEAK in app/VERSION, build,
#                     cp build/app/zephyr/zmk.signed.bin build/zmk.signed.alt.bin,
#                     revert, rebuild.
#   --jlink-sn SN     J-Link serial for nrfutil (default: autodetect)

set -u

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ASTER="${ASTER:-$HOME/clone/aster/target/debug/aster}"
MCUMGRCTL="${MCUMGRCTL:-$HOME/clone/mcumgr-toolkit/target/debug/mcumgrctl}"
IMAGE="$REPO_DIR/build/app/zephyr/zmk.signed.bin"
REC_GLOB='/dev/serial/by-id/*Daisy_Keyboard*Recovery*'
ITERATIONS=10
DELAYS=(0 0.2 0.5 1 2 5)
UPLOAD_EVERY=3
WEST_FLASH=0
KEEP_GOING=0
CHAOS=0
JLINK_SN=""
ALT_IMAGE="$REPO_DIR/build/zmk.signed.alt.bin"
SMP_TIMEOUT_MS=3000
ENUM_TIMEOUT=15   # seconds to wait for recovery/app (re-)enumeration
APP_TIMEOUT=20    # seconds to wait for app to answer over aster

while [ $# -gt 0 ]; do
    case "$1" in
        -n) ITERATIONS="$2"; shift 2 ;;
        --delays) read -r -a DELAYS <<< "$2"; shift 2 ;;
        --upload-every) UPLOAD_EVERY="$2"; shift 2 ;;
        --image) IMAGE="$2"; shift 2 ;;
        --west-flash) WEST_FLASH=1; shift ;;
        --keep-going) KEEP_GOING=1; shift ;;
        --chaos) CHAOS="$2"; shift 2 ;;
        --alt-image) ALT_IMAGE="$2"; shift 2 ;;
        --jlink-sn) JLINK_SN="$2"; shift 2 ;;
        *) echo "unknown option: $1" >&2; exit 2 ;;
    esac
done

PASS=0
FAIL=0
CHAOS_HITS=0
CHAOS_TO_RECOVERY=0
CHAOS_APP_SURVIVED=0
declare -a ENUM_TIMES=()
declare -a BOOT_TIMES=()
declare -a FAILURES=()

now() { date +%s.%N; }
elapsed() { awk -v a="$1" -v b="$2" 'BEGIN { printf "%.2f", b - a }'; }
log() { printf '[%s] %s\n' "$(date +%H:%M:%S)" "$*"; }

die_or_continue() {
    FAILURES+=("iter $iter: $*")
    FAIL=$((FAIL + 1))
    log "FAIL: $*"
    [ "$KEEP_GOING" = 1 ] && return 0
    summary
    exit 1
}

# Wait until the glob matches, echo first match. Empty output on timeout.
wait_for_dev() {
    local glob="$1" timeout="$2" t0 m
    t0=$(now)
    while :; do
        # shellcheck disable=SC2086
        m=$(compgen -G $glob | head -1)
        [ -n "$m" ] && { echo "$m"; return 0; }
        awk -v a="$t0" -v b="$(now)" -v t="$timeout" 'BEGIN { exit !(b - a > t) }' && return 1
        sleep 0.05
    done
}

wait_dev_gone() {
    local glob="$1" timeout="$2" t0
    t0=$(now)
    while compgen -G $glob > /dev/null; do
        awk -v a="$t0" -v b="$(now)" -v t="$timeout" 'BEGIN { exit !(b - a > t) }' && return 1
        sleep 0.05
    done
    return 0
}

wait_app_ready() {
    local timeout="$1" t0 ver
    t0=$(now)
    while :; do
        ver=$("$ASTER" v1 --firmware-version 2>/dev/null) && { echo "$ver"; return 0; }
        awk -v a="$t0" -v b="$(now)" -v t="$timeout" 'BEGIN { exit !(b - a > t) }' && return 1
        sleep 0.3
    done
}

# Interrupt an upload with a J-Link reset at a random point, then verify the
# device ends up either still running a valid app or in recovery, where a
# repair upload must succeed. Runs the rest of the iteration itself.
chaos_upload() {
    local t up_pid outcome rec ver t_reset t_boot in_flight
    # Transfer takes ~3.5-4s; aim inside it so the write is really cut short.
    t=$(awk -v r=$RANDOM 'BEGIN { printf "%.1f", 0.3 + r / 32767 * 3.0 }')
    CHAOS_HITS=$((CHAOS_HITS + 1))
    log "CHAOS: interrupting upload of $(basename "$ALT_IMAGE") with nrfutil reset after ${t}s"

    "$MCUMGRCTL" -q -t "$SMP_TIMEOUT_MS" -s "$rec_dev" image upload "$ALT_IMAGE" >/dev/null 2>&1 &
    up_pid=$!
    sleep "$t"
    in_flight=yes
    kill -0 "$up_pid" 2>/dev/null || in_flight=no
    nrfutil device reset --serial-number "$JLINK_SN" >/dev/null 2>&1 \
        || { kill "$up_pid" 2>/dev/null; die_or_continue "nrfutil device reset failed"; return 1; }
    kill "$up_pid" 2>/dev/null
    wait "$up_pid" 2>/dev/null
    log "CHAOS: upload still in flight at reset: $in_flight"

    # Outcome: either the app image survived and boots, or the image is
    # invalid and mcuboot must auto-enter serial recovery.
    outcome=""
    local t0
    t0=$(now)
    while [ -z "$outcome" ]; do
        # shellcheck disable=SC2086
        rec=$(compgen -G $REC_GLOB | head -1)
        [ -n "$rec" ] && outcome=recovery && break
        "$ASTER" v1 --firmware-version >/dev/null 2>&1 && outcome=app && break
        awk -v a="$t0" -v b="$(now)" -v t="$ENUM_TIMEOUT" 'BEGIN { exit !(b - a > t) }' \
            && { die_or_continue "neither app nor recovery came up after interrupt"; return 1; }
        sleep 0.3
    done

    if [ "$outcome" = app ]; then
        CHAOS_APP_SURVIVED=$((CHAOS_APP_SURVIVED + 1))
        log "CHAOS: app image survived the interrupt, app booted"
        return 0
    fi

    CHAOS_TO_RECOVERY=$((CHAOS_TO_RECOVERY + 1))
    log "CHAOS: image invalidated -> auto-entered recovery ($rec), repairing"
    "$MCUMGRCTL" -q -t "$SMP_TIMEOUT_MS" -s "$rec" image upload "$IMAGE" >/dev/null 2>&1 \
        || { die_or_continue "repair upload failed"; return 1; }
    t_reset=$(now)
    "$MCUMGRCTL" -t "$SMP_TIMEOUT_MS" -s "$rec" os system-reset >/dev/null 2>&1 \
        || { die_or_continue "reset after repair failed"; return 1; }
    wait_dev_gone "$REC_GLOB" "$ENUM_TIMEOUT" \
        || { die_or_continue "recovery CDC still present after repair reset"; return 1; }
    ver=$(wait_app_ready "$APP_TIMEOUT") \
        || { die_or_continue "app did not boot after repair"; return 1; }
    t_boot=$(elapsed "$t_reset" "$(now)")
    BOOT_TIMES+=("$t_boot")
    log "CHAOS: repaired, app back (fw $ver) after ${t_boot}s"
    return 0
}

summary() {
    echo
    echo "=== Summary: $PASS passed, $FAIL failed (of $ITERATIONS planned) ==="
    if [ "$CHAOS_HITS" -gt 0 ]; then
        echo "chaos: $CHAOS_HITS interrupted uploads -> $CHAOS_TO_RECOVERY recovered via bootloader, $CHAOS_APP_SURVIVED app survived"
    fi
    if [ ${#ENUM_TIMES[@]} -gt 0 ]; then
        printf '%s\n' "${ENUM_TIMES[@]}" | awk \
            '{ s += $1; if ($1 > mx || NR == 1) mx = $1; if ($1 < mn || NR == 1) mn = $1 }
             END { printf "jump -> recovery CDC: min %.2fs avg %.2fs max %.2fs\n", mn, s/NR, mx }'
    fi
    if [ ${#BOOT_TIMES[@]} -gt 0 ]; then
        printf '%s\n' "${BOOT_TIMES[@]}" | awk \
            '{ s += $1; if ($1 > mx || NR == 1) mx = $1; if ($1 < mn || NR == 1) mn = $1 }
             END { printf "reset -> app answers: min %.2fs avg %.2fs max %.2fs\n", mn, s/NR, mx }'
    fi
    for f in "${FAILURES[@]:-}"; do [ -n "$f" ] && echo "  $f"; done
}
trap summary INT

[ -x "$ASTER" ] || { echo "aster not found at $ASTER" >&2; exit 2; }
[ -x "$MCUMGRCTL" ] || { echo "mcumgrctl not found at $MCUMGRCTL" >&2; exit 2; }
[ "$UPLOAD_EVERY" != 0 ] && [ ! -f "$IMAGE" ] && { echo "image not found: $IMAGE" >&2; exit 2; }

if awk -v p="$CHAOS" 'BEGIN { exit !(p > 0) }'; then
    command -v nrfutil >/dev/null || { echo "--chaos needs nrfutil" >&2; exit 2; }
    [ -f "$ALT_IMAGE" ] || { echo "--chaos needs alt image ($ALT_IMAGE), see --alt-image help" >&2; exit 2; }
    cmp -s "$ALT_IMAGE" "$IMAGE" && { echo "--alt-image must differ from --image" >&2; exit 2; }
    if [ -z "$JLINK_SN" ]; then
        JLINK_SN=$(nrfutil device list 2>/dev/null | awk '/^[0-9]+$/ { sn = $1 } /J-Link/ { print sn; exit }')
        [ -n "$JLINK_SN" ] || { echo "--chaos: no J-Link found, pass --jlink-sn" >&2; exit 2; }
        log "chaos enabled (p=$CHAOS), J-Link SN $JLINK_SN"
    fi
fi

if [ "$WEST_FLASH" = 1 ]; then
    log "west flash (initial full flash)"
    (cd "$REPO_DIR" && west flash) || { echo "west flash failed" >&2; exit 1; }
fi

for ((iter = 1; iter <= ITERATIONS; iter++)); do
    delay=${DELAYS[$(( (iter - 1) % ${#DELAYS[@]} ))]}
    do_upload=0
    [ "$UPLOAD_EVERY" != 0 ] && [ $((iter % UPLOAD_EVERY)) = 0 ] && do_upload=1
    log "--- iteration $iter/$ITERATIONS (post-enum delay ${delay}s, upload=$do_upload) ---"

    # 1. App must be alive. If a previous run died in recovery, reset out of it.
    # shellcheck disable=SC2086
    rec=$(compgen -G $REC_GLOB | head -1)
    if [ -n "$rec" ]; then
        log "device stuck in recovery ($rec), resetting"
        "$MCUMGRCTL" -t "$SMP_TIMEOUT_MS" -s "$rec" os system-reset >/dev/null 2>&1
        wait_dev_gone "$REC_GLOB" "$ENUM_TIMEOUT" || true
    fi
    ver=$(wait_app_ready "$APP_TIMEOUT") \
        || { die_or_continue "app not answering over aster"; continue; }
    log "app alive, fw $ver"

    # 2. Jump to bootloader.
    t_jump=$(now)
    "$ASTER" v1 --bootloader-jump >/dev/null 2>&1 \
        || { die_or_continue "bootloader-jump command failed"; continue; }

    # 3. Recovery CDC ACM must enumerate.
    rec=$(wait_for_dev "$REC_GLOB" "$ENUM_TIMEOUT") \
        || { die_or_continue "recovery CDC did not enumerate within ${ENUM_TIMEOUT}s"; continue; }
    t_enum=$(elapsed "$t_jump" "$(now)")
    ENUM_TIMES+=("$t_enum")
    log "recovery at $rec after ${t_enum}s"

    # 4. Variable delay before first SMP traffic — the timing under test.
    sleep "$delay"

    # 5. SMP must answer.
    state=$("$MCUMGRCTL" -t "$SMP_TIMEOUT_MS" -s "$rec" image get-state 2>/dev/null) \
        || { die_or_continue "image get-state failed on $rec"; continue; }
    grep -q hash <<< "$state" \
        || { die_or_continue "image get-state gave no hash"; continue; }

    # 6. Periodic full image upload: stresses the flash write path.
    if [ "$do_upload" = 1 ]; then
        if awk -v r=$RANDOM -v p="$CHAOS" 'BEGIN { exit !(r / 32767 < p) }'; then
            # chaos_upload finishes the iteration itself (device ends in app)
            rec_dev="$rec"
            if chaos_upload; then
                PASS=$((PASS + 1))
            fi
            continue
        fi
        log "uploading $(basename "$IMAGE")"
        "$MCUMGRCTL" -q -t "$SMP_TIMEOUT_MS" -s "$rec" image upload "$IMAGE" >/dev/null 2>&1 \
            || { die_or_continue "image upload failed"; continue; }
    fi

    # 7. Reset out of recovery.
    t_reset=$(now)
    "$MCUMGRCTL" -t "$SMP_TIMEOUT_MS" -s "$rec" os system-reset >/dev/null 2>&1 \
        || { die_or_continue "os system-reset failed"; continue; }

    # 8. Recovery device must vanish and the app must come back.
    wait_dev_gone "$REC_GLOB" "$ENUM_TIMEOUT" \
        || { die_or_continue "recovery CDC still present after reset"; continue; }
    ver=$(wait_app_ready "$APP_TIMEOUT") \
        || { die_or_continue "app did not come back after reset"; continue; }
    t_boot=$(elapsed "$t_reset" "$(now)")
    BOOT_TIMES+=("$t_boot")
    log "app back (fw $ver) after ${t_boot}s"

    PASS=$((PASS + 1))
done

summary
[ "$FAIL" = 0 ]
