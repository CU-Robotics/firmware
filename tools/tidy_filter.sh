#!/usr/bin/env bash
# Cleans up clang-tidy output for `make tidy` and `make tidy-diff`.
# Reads clang-tidy output on stdin and always exits 0.
#
# - Drops findings outside src/. clang can't parse a GCC extension in the
#   vendored FlexCAN_T4 library, so it reports errors there.
# - Drops repeats: a header's findings appear once per .cpp that includes it.
# - With TIDY_ONLY_FILES (space-separated paths), keeps only findings in
#   those files.
# - With TIDY_FORMAT=github, also prints GitHub Actions annotations.

awk -v root="$(pwd)/" -v only="${TIDY_ONLY_FILES:-}" -v github="${TIDY_FORMAT:-}" '
    BEGIN {
        n = split(only, names, " ")
        for (i = 1; i <= n; i++) allowed[names[i]] = 1
    }
    /^[^ ].*:[0-9]+:[0-9]+: (warning|error):/ {
        line = $0
        if (index(line, root) == 1) line = substr(line, length(root) + 1)
        file = line
        sub(/:[0-9]+:[0-9]+: .*/, "", file)
        keep = file ~ /^src\// && (n == 0 || file in allowed) && !seen[line]++
        if (keep) {
            print line
            count++
            if (github == "github") {
                split(line, parts, ":")
                msg = line
                sub(/^[^:]+:[0-9]+:[0-9]+: (warning|error): /, "", msg)
                notes[count] = "::warning file=" parts[1] ",line=" parts[2] ",col=" parts[3] "::" msg
            }
        }
        next
    }
    keep
    END {
        printf "TIDY     %d finding(s)\n", count
        for (i = 1; i <= count; i++) if (i in notes) print notes[i]
    }
'
exit 0
