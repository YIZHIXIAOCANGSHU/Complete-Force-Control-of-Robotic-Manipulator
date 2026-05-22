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
command -v grep >/dev/null 2>&1 || { echo "Missing required command: grep" >&2; exit 1; }
command -v mktemp >/dev/null 2>&1 || { echo "Missing required command: mktemp" >&2; exit 1; }

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
awk -F '\t' -v root="$PROJECT_ROOT" '
    function print_table_header(title, width, line) {
        line = ""
        while (length(line) < width) {
            line = line "-"
        }
        printf "%-" width "s %8s %10s %10s %10s\n", title, "文件数", "总行数", "非空行", "空行"
        printf "%-" width "s %8s %10s %10s %10s\n", line, "--------", "----------", "----------", "----------"
    }
    function print_table_row(name, files, lines, blanks, width) {
        printf "%-" width "s %8d %10d %10d %10d\n", name, files, lines, lines - blanks, blanks
    }
    function extname(path, base, n, parts) {
        base = path
        sub(/^.*\//, "", base)
        if (base !~ /\./) {
            return "[no_ext]"
        }
        n = split(base, parts, ".")
        return "." parts[n]
    }
    function py_category(path, rel, n, parts) {
        if (path ~ /^\.\/src\/robot_control\//) {
            rel = path
            sub(/^\.\/src\/robot_control\//, "", rel)
            n = split(rel, parts, "/")
            if (n == 1) {
                return "src/robot_control"
            }
            if (parts[1] == "hardware" && n >= 3) {
                return "src/robot_control/hardware/" parts[2]
            }
            if (parts[1] == "hardware") {
                return "src/robot_control/hardware"
            }
            if (parts[1] == "modes" && n >= 3) {
                return "src/robot_control/modes/" parts[2]
            }
            if (parts[1] == "modes") {
                return "src/robot_control/modes"
            }
            if (parts[1] == "shared" && n >= 3) {
                return "src/robot_control/shared/" parts[2]
            }
            if (parts[1] == "shared") {
                return "src/robot_control/shared"
            }
            return "src/robot_control/" parts[1]
        }
        if (path ~ /^\.\/tests\//) {
            return "tests"
        }
        rel = path
        sub(/^\.\//, "", rel)
        if (rel !~ /\//) {
            return "[repo_root]"
        }
        sub(/\/[^\/]+$/, "", rel)
        return rel
    }
    function sort_keys_by_lines(values, keys,    key, count, i, j, current) {
        delete keys
        count = 0
        for (key in values) {
            keys[++count] = key
        }
        for (i = 2; i <= count; i++) {
            current = keys[i]
            j = i - 1
            while (j >= 1 && (values[keys[j]] < values[current] || (values[keys[j]] == values[current] && keys[j] > current))) {
                keys[j + 1] = keys[j]
                j--
            }
            keys[j + 1] = current
        }
        return count
    }
    {
        ext = extname($1)
        files++
        lines += $2
        blanks += $3
        ext_files[ext]++
        ext_lines[ext] += $2
        ext_blanks[ext] += $3
        if (ext == ".py") {
            category = py_category($1)
            py_files[category]++
            py_lines[category] += $2
            py_blanks[category] += $3
        }
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

        printf "按 Python 文件分类统计（行数降序）:\n"
        print_table_header("分类", 40)
        py_count = sort_keys_by_lines(py_lines, py_keys)
        for (i = 1; i <= py_count; i++) {
            category = py_keys[i]
            print_table_row(category, py_files[category], py_lines[category], py_blanks[category], 40)
        }
        printf "\n"

        printf "按扩展名统计（行数降序）:\n"
        print_table_header("扩展名", 12)
        ext_count = sort_keys_by_lines(ext_lines, ext_keys)
        for (i = 1; i <= ext_count; i++) {
            ext = ext_keys[i]
            print_table_row(ext, ext_files[ext], ext_lines[ext], ext_blanks[ext], 12)
        }
    }
' "$TMP_COUNTS"
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
