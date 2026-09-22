#!/usr/bin/env bash
# Upload a C++ Unity suite and return the result reported by the Teensy.
set -euo pipefail

firmware=${1:?Usage: run_teensy_test.sh firmware.hex}
tycmd=${TYCMD:-tycmd}
test_timeout=${TEST_TIMEOUT:-30}
if [[ ! $test_timeout =~ ^[1-9][0-9]*$ ]]; then
    echo "TEST_TIMEOUT must be a positive integer (seconds)." >&2
    exit 1
fi

board_args=()
if [[ -n ${TEST_BOARD:-} ]]; then
    board_args=(--board "$TEST_BOARD")
else
    devices=$("$tycmd" list)
    count=$(printf '%s\n' "$devices" | awk 'NF { count++ } END { print count+0 }')
    if [[ $count != 1 ]]; then
        echo "Connect one Teensy 4.1, or select one with TEST_BOARD=tag. Detected $count boards." >&2
        exit 1
    fi
fi

printf '\n===== %s (Teensy) =====\n' "$(basename "$(dirname "$firmware")")"
# also supports empty arrays on macOS Bash 3.2.
"$tycmd" upload ${board_args[@]+"${board_args[@]}"} "$firmware"

test_tmp=$(mktemp -d)
monitor_pid=
cleanup() {
    if [[ -n $monitor_pid ]]; then
        kill "$monitor_pid" 2>/dev/null || true
        wait "$monitor_pid" 2>/dev/null || true
    fi
    rm -rf "$test_tmp"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

# Open both ends so a failed monitor cannot block opening the FIFO.
mkfifo "$test_tmp/output"
exec 3<> "$test_tmp/output"
printf r > "$test_tmp/start"
"$tycmd" monitor ${board_args[@]+"${board_args[@]}"} --raw --silent --timeout-eof -1 \
    < "$test_tmp/start" >&3 2>&1 &
monitor_pid=$!

deadline=$((SECONDS + test_timeout))
pending=
result_pattern='^TEENSY_TEST_RESULT ([0-9]+) ([0-9]+)$'
while (( SECONDS < deadline )); do
    line=
    # Read one character at a time: Bash 3.2 discards partial lines on timeout.
    if IFS= read -r -n 1 -t 1 line <&3; then
        if [[ -n $line ]]; then
            pending+=$line
            continue
        fi
        printf '%s\n' "$pending"
        if [[ ${pending%$'\r'} =~ $result_pattern ]]; then
            tests=${BASH_REMATCH[1]}
            failures=${BASH_REMATCH[2]}
            if (( 10#$tests > 0 && 10#$failures == 0 )); then
                exit 0
            fi
            exit 1
        fi
        pending=
    else
        if ! kill -0 "$monitor_pid" 2>/dev/null; then
            echo "Teensy monitor exited before reporting test results." >&2
            exit 1
        fi
    fi
done

echo "Teensy test timed out after $test_timeout seconds." >&2
exit 1
