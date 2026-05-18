#!/usr/bin/env bash
# One-click source line counter with a small desktop result window.

set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SHOW_GUI=1

for arg in "$@"; do
    case "$arg" in
        --no-gui) SHOW_GUI=0 ;;
        -h|--help)
            cat <<'USAGE'
Usage:
  ./count_lines.sh            Count project text files and show a small window
  ./count_lines.sh --no-gui   Print the report only
USAGE
            exit 0
            ;;
        *)
            echo "Unknown option: $arg" >&2
            echo "Try: ./count_lines.sh --help" >&2
            exit 2
            ;;
    esac
done

command -v find >/dev/null 2>&1 || { echo "Missing required command: find" >&2; exit 1; }
command -v file >/dev/null 2>&1 || { echo "Missing required command: file" >&2; exit 1; }
command -v awk >/dev/null 2>&1 || { echo "Missing required command: awk" >&2; exit 1; }

cd "$PROJECT_ROOT"

TMP_FILES="$(mktemp)"
TMP_COUNTS="$(mktemp)"
cleanup() {
    rm -f "$TMP_FILES" "$TMP_COUNTS"
}
trap cleanup EXIT

find . \
    \( \
        -path './.git' -o \
        -path './.venv' -o \
        -path './__pycache__' -o \
        -path './.pytest_cache' -o \
        -path './.mypy_cache' -o \
        -path './.ruff_cache' -o \
        -path './.cache' -o \
        -path './build' -o \
        -path './dist' -o \
        -path './results' -o \
        -path './htmlcov' -o \
        -path './node_modules' -o \
        -path '*/__pycache__' \
    \) -prune -o \
    -type f \
    ! -name '*.pyc' \
    ! -name '*.pyo' \
    ! -name '*.so' \
    ! -name '*.dll' \
    ! -name '*.dylib' \
    ! -name '*.a' \
    ! -name '*.o' \
    ! -name '*.exe' \
    ! -name '*.png' \
    ! -name '*.jpg' \
    ! -name '*.jpeg' \
    ! -name '*.gif' \
    ! -name '*.webp' \
    ! -name '*.ico' \
    ! -name '*.pdf' \
    ! -name '*.zip' \
    ! -name '*.gz' \
    ! -name '*.tar' \
    ! -name '*.tgz' \
    ! -name '*.7z' \
    ! -name '*.STL' \
    ! -name '*.stl' \
    ! -name '*.npy' \
    ! -name '*.npz' \
    ! -name '*.mat' \
    ! -name '*.sav' \
    ! -name '*.parquet' \
    ! -name '*.orc' \
    ! -name '*.wav' \
    -print0 |
while IFS= read -r -d '' path; do
    if file --brief --mime "$path" | grep -Eq 'charset=(us-ascii|utf-8|utf-16|iso-8859|unknown-8bit)|text/'; then
        printf '%s\0' "$path"
    fi
done > "$TMP_FILES"

while IFS= read -r -d '' path; do
    awk -v path="$path" '
        BEGIN { total = 0; blank = 0 }
        { total++; if ($0 ~ /^[[:space:]]*$/) blank++ }
        END { printf "%s\t%d\t%d\n", path, total, blank }
    ' "$path"
done < "$TMP_FILES" > "$TMP_COUNTS"

REPORT="$(
awk -F '\t' '
    function extname(path, base, n, parts) {
        base = path
        sub(/^.*\//, "", base)
        if (base !~ /\./) {
            return "[no_ext]"
        }
        n = split(base, parts, ".")
        return "." parts[n]
    }
    {
        ext = extname($1)
        files++
        lines += $2
        blanks += $3
        ext_files[ext]++
        ext_lines[ext] += $2
    }
    END {
        nonblank = lines - blanks
        printf "AM-D02 项目代码量统计\n"
        printf "=====================\n\n"
        printf "项目路径: %s\n", root
        printf "统计口径: 文本文件；已排除 .git、.venv、__pycache__、results、构建/缓存目录和常见二进制文件。\n\n"
        printf "总文件数: %d\n", files
        printf "总行数:   %d\n", lines
        printf "非空行:   %d\n", nonblank
        printf "空行:     %d\n\n", blanks
        printf "按扩展名统计（行数降序）:\n"
        printf "%-12s %8s %10s\n", "扩展名", "文件数", "行数"
        printf "%-12s %8s %10s\n", "------------", "--------", "----------"
        for (ext in ext_lines) {
            printf "%s\t%d\t%d\n", ext, ext_files[ext], ext_lines[ext] | "sort -t\"\t\" -k3,3nr -k1,1"
        }
        close("sort -t\"\t\" -k3,3nr -k1,1")
    }
' root="$PROJECT_ROOT" "$TMP_COUNTS" |
awk -F '\t' '
    NF == 3 && $1 ~ /^(\.|\\[no_ext\\])$/ { next }
    NF == 3 { printf "%-12s %8d %10d\n", $1, $2, $3; next }
    { print }
'
)"

printf '%s\n' "$REPORT"

if [[ "$SHOW_GUI" -eq 1 ]] && command -v zenity >/dev/null 2>&1 && [[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]]; then
    printf '%s\n' "$REPORT" |
        zenity --text-info \
            --title="代码量统计" \
            --width=760 \
            --height=560 \
            --ok-label="关闭" \
        >/dev/null 2>&1 || true
fi
