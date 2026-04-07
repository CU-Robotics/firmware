#!/usr/bin/env bash

set -euo pipefail

json_escape() {
    local s="${1//\\/\\\\}"
    s="${s//\"/\\\"}"
    s="${s//$'\n'/\\n}"
    s="${s//$'\r'/\\r}"
    s="${s//$'\t'/\\t}"
    printf '%s' "$s"
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

echo "[compile-db] Generating compile_commands.json"

if [[ ! -x "$COMPILER_CPP" ]]; then
    echo "Error: C++ compiler not found at $COMPILER_CPP" >&2
    exit 1
fi

if [[ ! -x "$COMPILER_C" ]]; then
    echo "Error: C compiler not found at $COMPILER_C" >&2
    exit 1
fi

tmp_db="$(mktemp compile_commands.json.XXXXXX)"
cleanup() {
    rm -f "$tmp_db"
}
trap cleanup EXIT

mapfile -t cxx_sys_includes < <(get_system_includes "$COMPILER_CPP" c++)
mapfile -t c_sys_includes < <(get_system_includes "$COMPILER_C" c)
read -r -a cppflags <<< "$CPPFLAGS"
read -r -a cxxflags <<< "$CXXFLAGS"
read -r -a cflags <<< "$CFLAGS"

first=1
entry_count=0

exec 3> "$tmp_db"
printf '[\n' >&3

for src in $SRC_FILES; do
    case "$src" in
        *.cc|*.cpp|*.cxx)
            compiler="$COMPILER_CPP"
            flags=(--target="$TARGET_TRIPLE" "${cppflags[@]}" "${cxxflags[@]}")
            sys_includes=("${cxx_sys_includes[@]}")
            ;;
        *.c)
            compiler="$COMPILER_C"
            flags=(--target="$TARGET_TRIPLE" "${cppflags[@]}" "${cflags[@]}")
            sys_includes=("${c_sys_includes[@]}")
            ;;
        *)
            continue
            ;;
    esac

    obj="$BUILD_DIR/$src.o"
    args=("$compiler" "${flags[@]}")
    for include_dir in "${sys_includes[@]}"; do
        args+=(-isystem "$include_dir")
    done
    args+=(-c "$src" -o "$obj")

    if [[ $first -eq 0 ]]; then
        printf ',\n' >&3
    fi
    first=0

    printf '  {\n' >&3
    printf '    "directory": "%s",\n' "$(json_escape "$CURDIR")" >&3
    printf '    "file": "%s",\n' "$(json_escape "$src")" >&3
    printf '    "output": "%s",\n' "$(json_escape "$obj")" >&3
    printf ',\n' >&3
    printf '    "arguments": [\n' >&3
    for i in "${!args[@]}"; do
        if [[ $i -gt 0 ]]; then
            printf ',\n' >&3
        fi
        printf '      "%s"' "$(json_escape "${args[$i]}")" >&3
    done
    printf '\n    ]\n' >&3
    printf '  }' >&3

    entry_count=$((entry_count + 1))
done

printf '\n]\n' >&3
exec 3>&-
mv "$tmp_db" compile_commands.json
trap - EXIT

echo "[compile-db] Done: compile_commands.json generated with $entry_count entries"
