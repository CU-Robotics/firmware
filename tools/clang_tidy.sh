#!/usr/bin/env bash
# Runs clang-tidy over the given files using compile_commands.json and the
# checks in .clang-tidy. Used by `make tidy` and `make tidy-diff`.
#
# Only files under src/ are checked. A header is checked through the .cpp
# with the same name, if there is one. Findings are printed but never fail
# the run, unless TIDY_STRICT=1.
#
# Environment:
#   CLANG_TIDY   clang-tidy binary (default: tools/clang-tidy/bin/clang-tidy,
#                which tools/install_clang_tidy.sh installs)
#   JOBS         parallel jobs (default: core count)
#   TIDY_FORMAT  set to "github" to also print GitHub Actions annotations
#   TIDY_ONLY_LISTED  set to 1 to report only findings in the files passed in,
#                not in other headers they include (used by `make tidy-diff`)

set -uo pipefail

CLANG_TIDY="${CLANG_TIDY:-tools/clang-tidy/bin/clang-tidy}"

if ! command -v "$CLANG_TIDY" >/dev/null 2>&1; then
    echo "clang-tidy not found at $CLANG_TIDY. Run tools/install_clang_tidy.sh or set CLANG_TIDY." >&2
    exit 1
fi

if [[ ! -f compile_commands.json ]]; then
    echo "compile_commands.json is missing. Run 'make clangd' first." >&2
    exit 1
fi

JOBS="${JOBS:-$(sysctl -n hw.ncpu 2>/dev/null || nproc 2>/dev/null || echo 1)}"

# Pick the translation units to check.
files=()
for f in "$@"; do
    case "$f" in
        src/*.cpp|src/*.c)
            [[ -f "$f" ]] && files+=("$f")
            ;;
        src/*.hpp|src/*.h)
            for ext in cpp c; do
                [[ -f "${f%.*}.$ext" ]] && files+=("${f%.*}.$ext")
            done
            ;;
    esac
done

if [[ ${#files[@]} -eq 0 ]]; then
    echo "TIDY     no source files under src/ to check"
    exit 0
fi

unique=()
while IFS= read -r f; do unique+=("$f"); done < <(printf '%s\n' "${files[@]}" | sort -u)
files=("${unique[@]}")
echo "TIDY     checking ${#files[@]} file(s) with $("$CLANG_TIDY" --version | grep -o 'version [0-9.]*')"

# The compile commands are for arm-none-eabi-gcc. These extra flags let clang
# parse them: ignore GCC-only flags and warnings, don't turn the project's
# -Werror into errors here, and match the int32_t definitions that
# arm-none-eabi-gcc uses (same fix as in .clangd).
extra_args=(
    --extra-arg=-Wno-unknown-warning-option
    --extra-arg=-Wno-unused-command-line-argument
    --extra-arg=-Wno-error
    --extra-arg=-U__INT32_TYPE__
    "--extra-arg=-D__INT32_TYPE__=long int"
    --extra-arg=-U__UINT32_TYPE__
    "--extra-arg=-D__UINT32_TYPE__=long unsigned int"
)

out="$(mktemp)"
trap 'rm -f "$out"' EXIT

printf '%s\0' "${files[@]}" \
    | xargs -0 -n 1 -P "$JOBS" "$CLANG_TIDY" -p . --quiet "${extra_args[@]}" \
    2>/dev/null > "$out"
status=$?

# Keep findings in our own code only: clang can't fully parse some vendored
# libraries (GCC extensions), and those errors aren't ours to fix. Each header
# finding is reported once per file that includes it, so drop repeats too.
root="$(pwd)/"
listed=""
if [[ "${TIDY_ONLY_LISTED:-0}" == "1" ]]; then
    listed="$(printf '%s\n' "$@")"
fi
findings="$(awk -v root="$root" -v listed="$listed" '
    BEGIN {
        n = split(listed, names, "\n")
        for (i = 1; i <= n; i++) if (names[i] != "") allowed[names[i]] = 1
    }
    /^[^ ].*:[0-9]+:[0-9]+: (warning|error):/ {
        path = $0
        if (index(path, root) == 1) path = substr(path, length(root) + 1)
        file = path
        sub(/:[0-9]+:[0-9]+: .*/, "", file)
        keep = (path ~ /^src\//) && (listed == "" || file in allowed) && !seen[path]++
        if (keep) print path
        next
    }
    keep
' "$out")"
echo "$findings"

count="$(grep -cE '^[^ ].*:[0-9]+:[0-9]+: (warning|error):' <<< "$findings")"
echo "TIDY     $count finding(s)"

if [[ "${TIDY_FORMAT:-}" == "github" ]]; then
    grep -E '^[^ ].*:[0-9]+:[0-9]+: (warning|error):' <<< "$findings" \
        | sed -E 's/^([^:]+):([0-9]+):([0-9]+): (warning|error): (.*)$/::warning file=\1,line=\2,col=\3::\5/'
fi

if [[ "${TIDY_STRICT:-0}" == "1" && ( "$count" -gt 0 || $status -ne 0 ) ]]; then
    exit 1
fi
exit 0
