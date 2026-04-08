#!/usr/bin/env bash

set -euo pipefail

JSON_ESCAPED=

json_escape() {
    JSON_ESCAPED="${1//\\/\\\\}"
    JSON_ESCAPED="${JSON_ESCAPED//\"/\\\"}"
    JSON_ESCAPED="${JSON_ESCAPED//$'\n'/\\n}"
    JSON_ESCAPED="${JSON_ESCAPED//$'\r'/\\r}"
    JSON_ESCAPED="${JSON_ESCAPED//$'\t'/\\t}"
}

get_system_includes() {
    local compiler="$1"
    local language="$2"

    "$compiler" -E -x "$language" - -v < /dev/null 2>&1 \
        | awk '
            /#include <...> search starts here:/ { in_list=1; next }
            /End of search list./ { exit }
            in_list {
                sub(/^[[:space:]]+/, "", $0)
                if (length($0) > 0) {
                    print $0
                }
            }
        '
}

: "${CURDIR:?CURDIR is required}"
: "${BUILD_DIR:?BUILD_DIR is required}"
: "${COMPILER_CPP:?COMPILER_CPP is required}"
: "${COMPILER_C:?COMPILER_C is required}"
: "${TARGET_TRIPLE:=arm-none-eabi}"
: "${CPPFLAGS:?CPPFLAGS is required}"
: "${CXXFLAGS:?CXXFLAGS is required}"
: "${CFLAGS:?CFLAGS is required}"
: "${SRC_FILES:?SRC_FILES is required}"

echo "[Generating compile_commands.json]"

if [[ ! -x "$COMPILER_CPP" ]]; then
    echo "Error: C++ compiler not found at $COMPILER_CPP" >&2
    exit 1
fi

if [[ ! -x "$COMPILER_C" ]]; then
    echo "Error: C compiler not found at $COMPILER_C" >&2
    exit 1
fi

tmp_db="$(mktemp "${TMPDIR:-/tmp}/compile_commands.json.XXXXXX")"
cleanup() {
    rm -f "$tmp_db"
}
trap cleanup EXIT

cxx_sys_includes=()
while IFS= read -r line; do
    cxx_sys_includes+=("$line")
done < <(get_system_includes "$COMPILER_CPP" c++)

c_sys_includes=()
while IFS= read -r line; do
    c_sys_includes+=("$line")
done < <(get_system_includes "$COMPILER_C" c)

read -r -a cppflags <<< "$CPPFLAGS"
read -r -a cxxflags <<< "$CXXFLAGS"
read -r -a cflags <<< "$CFLAGS"

json_escape "$CURDIR"
escaped_curdir="$JSON_ESCAPED"

cxx_base_args=("$COMPILER_CPP" --target="$TARGET_TRIPLE" "${cppflags[@]}" "${cxxflags[@]}")
for include_dir in "${cxx_sys_includes[@]}"; do
    cxx_base_args+=(-isystem "$include_dir")
done

c_base_args=("$COMPILER_C" --target="$TARGET_TRIPLE" "${cppflags[@]}" "${cflags[@]}")
for include_dir in "${c_sys_includes[@]}"; do
    c_base_args+=(-isystem "$include_dir")
done

first=1
entry_count=0

exec 3> "$tmp_db"
printf '[\n' >&3

for src in $SRC_FILES; do
    case "$src" in
        *.cc|*.cpp|*.cxx)
            args=("${cxx_base_args[@]}")
            ;;
        *.c)
            args=("${c_base_args[@]}")
            ;;
        *)
            continue
            ;;
    esac

    obj="$BUILD_DIR/$src.o"
    args+=(-c "$src" -o "$obj")

    if [[ $first -eq 0 ]]; then
        printf ',\n' >&3
    fi
    first=0

    json_escape "$src"
    escaped_src="$JSON_ESCAPED"
    json_escape "$obj"
    escaped_obj="$JSON_ESCAPED"

    printf '  {\n' >&3
    printf '    "directory": "%s",\n' "$escaped_curdir" >&3
    printf '    "file": "%s",\n' "$escaped_src" >&3
    printf '    "output": "%s",\n' "$escaped_obj" >&3
    printf '    "arguments": [' >&3
    for i in "${!args[@]}"; do
        if [[ $i -gt 0 ]]; then
            printf ', ' >&3
        fi
        json_escape "${args[$i]}"
        printf '"%s"' "$JSON_ESCAPED" >&3
    done
    printf ']\n' >&3
    printf '  }' >&3

    entry_count=$((entry_count + 1))
done

printf '\n]\n' >&3
exec 3>&-
mv "$tmp_db" compile_commands.json
trap - EXIT

echo "[compile_commands.json generated with $entry_count entries]"
