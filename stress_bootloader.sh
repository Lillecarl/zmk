#!/usr/bin/env bash
# Bootloader stress test for the Daisy keyboard (default) or dongle.
#
# Cycles: app -> bootloader jump -> mcuboot serial recovery (CDC ACM)
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
#   --rec-glob GLOB   recovery CDC ACM device glob
#                     (default '/dev/serial/by-id/*Daisy_Keyboard*Recovery*')
#   --jump-cmd CMD    shell command that makes the app reboot into recovery
#                     (default: aster v1 --bootloader-jump)
#   --app-check-cmd CMD
#                     shell command that succeeds iff the app is alive; its
#                     stdout is logged as the firmware version
#                     (default: aster v1 --firmware-version)
#   --safe-check      after each app boot, hammer the factory protocol with a
#                     randomized sweep of safe commands (read-only queries
#                     plus transient LED/backlight setters) and fail the
#                     iteration if the firmware rejects any. Each pick is
#                     uniformly random with replacement, so order and
#                     per-command frequency differ every sweep. Catches a
#                     stale image whose factory protocol handles only INFO
#                     (the plain app check can't tell it from a current
#                     build) and exercises the HID request path right after
#                     re-enumeration. LEDs will blink during the sweep.
#                     Keyboard-only: the sweep always drives aster v1.
#   --safe-cmds "..." space-separated aster v1 flags to pick from, implies
#                     --safe-check. "@P" in an entry is replaced with a fresh
#                     random percent (0-100) on every pick. The default is
#                     every command that's harmless under repetition: all
#                     read-only queries and the RAM-only LED/caps/backlight
#                     setters. --factory-reset is deliberately excluded: a
#                     no-op stub in aster today, but it will wipe persisted
#                     state once its firmware lands.
#   --safe-count N    commands per sweep (default 25), implies --safe-check
#
# The Daisy dongle (hid-remapper) reuses this script by overriding the three
# device-specific hooks; see daisy.md in hid-remapper-private for the exact
# invocation.

set -u

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ASTER="${ASTER:-$HOME/clone/aster/target/debug/aster}"
MCUMGRCTL="${MCUMGRCTL:-$(command -v mcumgrctl || echo "$HOME/clone/mcumgr-toolkit/target/debug/mcumgrctl")}"
IMAGE="$REPO_DIR/build/app/zephyr/zmk.signed.bin"
REC_GLOB='/dev/serial/by-id/*Daisy_Keyboard*Recovery*'
ITERATIONS=10
DELAYS=(0 0.2 0.5 1 2 5)
UPLOAD_EVERY=3
WEST_FLASH=0
KEEP_GOING=0
CHAOS=0
JLINK_SN=""
JUMP_CMD=""
APP_CHECK_CMD=""
ALT_IMAGE="$REPO_DIR/build/zmk.signed.alt.bin"
SAFE_CHECK=0
SAFE_CMDS="--info --proto-version --firmware-version --serial --lang-id
           --battery-temp --battery-voltage --battery-current
           --pairing-button-state --protocol-switch-state
           --caps-led-on --caps-led-off --backlight-pwm=@P
           --led-0-pwm=@P --led-1-pwm=@P --led-2-pwm=@P --led-3-pwm=@P
           --led-red-pwm=@P --led-green-pwm=@P --led-blue-pwm=@P"
SAFE_COUNT=25
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
        --rec-glob) REC_GLOB="$2"; shift 2 ;;
        --jump-cmd) JUMP_CMD="$2"; shift 2 ;;
        --app-check-cmd) APP_CHECK_CMD="$2"; shift 2 ;;
        --safe-check) SAFE_CHECK=1; shift ;;
        --safe-cmds) SAFE_CMDS="$2"; SAFE_CHECK=1; shift 2 ;;
        --safe-count) SAFE_COUNT="$2"; SAFE_CHECK=1; shift 2 ;;
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

# Succeeds iff the app is alive; prints its firmware version.
app_version() {
    if [ -n "$APP_CHECK_CMD" ]; then
        bash -c "$APP_CHECK_CMD" 2>/dev/null
    else
        "$ASTER" v1 --firmware-version 2>/dev/null
    fi
}

# Randomized sweep of safe factory commands (--safe-check): SAFE_COUNT picks
# from SAFE_CMDS, uniformly random with replacement, so order and per-command
# frequency differ every sweep; "@P" in a pick becomes a fresh random percent.
# Fails the iteration on the first command the firmware rejects — catches a
# stale image whose factory protocol is a subset of the current one (e.g.
# answers INFO but not battery reads), which app_version alone can't
# distinguish from a current build.
safe_cmds_check() {
    [ "$SAFE_CHECK" = 1 ] || return 0
    local cmds i tmpl cmd out mix=""
    # shellcheck disable=SC2206 # word splitting is the parse (multi-line list)
    cmds=($SAFE_CMDS)
    declare -A counts=()
    for ((i = 0; i < SAFE_COUNT; i++)); do
        tmpl=${cmds[RANDOM % ${#cmds[@]}]}
        counts[$tmpl]=$((${counts[$tmpl]:-0} + 1))
        cmd=${tmpl//@P/$((RANDOM % 101))}
        out=$("$ASTER" v1 "$cmd" 2>&1) \
            || { die_or_continue "safe command $cmd rejected: $out"; return 1; }
    done
    for tmpl in "${!counts[@]}"; do mix+="$tmpl x${counts[$tmpl]} "; done
    log "safe sweep ok: $SAFE_COUNT commands (${mix% })"
    return 0
}

jump_to_bootloader() {
    if [ -n "$JUMP_CMD" ]; then
        bash -c "$JUMP_CMD" >/dev/null 2>&1
    else
        "$ASTER" v1 --bootloader-jump >/dev/null 2>&1
    fi
}

wait_app_ready() {
    local timeout="$1" t0 ver
    t0=$(now)
    while :; do
        ver=$(app_version) && { echo "$ver"; return 0; }
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
        app_version >/dev/null && outcome=app && break
        awk -v a="$t0" -v b="$(now)" -v t="$ENUM_TIMEOUT" 'BEGIN { exit !(b - a > t) }' \
            && { die_or_continue "neither app nor recovery came up after interrupt"; return 1; }
        sleep 0.3
    done

    if [ "$outcome" = app ]; then
        CHAOS_APP_SURVIVED=$((CHAOS_APP_SURVIVED + 1))
        log "CHAOS: app image survived the interrupt, app booted"
        safe_cmds_check || return 1
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
    safe_cmds_check || return 1
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

if [ -z "$JUMP_CMD" ] || [ -z "$APP_CHECK_CMD" ] || [ "$SAFE_CHECK" = 1 ]; then
    [ -x "$ASTER" ] || { echo "aster not found at $ASTER" >&2; exit 2; }
fi
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
    safe_cmds_check || continue

    # 2. Jump to bootloader.
    t_jump=$(now)
    jump_to_bootloader \
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
    state=$("$MCUMGRCTL" -t "$SMP_TIMEOUT_MS" --retries 0 -s "$rec" image get-state 2>/dev/null) \
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
    safe_cmds_check || continue

    PASS=$((PASS + 1))
done

summary
[ "$FAIL" = 0 ]
