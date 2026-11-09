#!/usr/bin/env bash
set -euo pipefail

# Run criterion benchmarks, append results to CSV history, and generate benchmarks.md
# with three-point tracking: baseline, previous, latest.
#
# Usage:
#   ./scripts/bench-history.sh              # defaults to bench-history.csv
#   ./scripts/bench-history.sh results.csv  # custom output file

HISTORY_FILE="${1:-bench-history.csv}"
BENCHMARKS_MD="benchmarks.md"
TIMESTAMP=$(date -u +"%Y-%m-%dT%H:%M:%SZ")
COMMIT=$(git rev-parse --short HEAD 2>/dev/null || echo "unknown")
BRANCH=$(git branch --show-current 2>/dev/null || echo "unknown")

# Create header if file doesn't exist
if [ ! -f "$HISTORY_FILE" ]; then
    echo "timestamp,commit,branch,benchmark,estimate_ns" > "$HISTORY_FILE"
fi

echo "╔══════════════════════════════════════════╗"
echo "║       impetus benchmark suite            ║"
echo "╠══════════════════════════════════════════╣"
echo "║  commit: $COMMIT                          ║"
echo "║  branch: $BRANCH                            ║"
echo "║  date:   $TIMESTAMP   ║"
echo "╚══════════════════════════════════════════╝"
echo ""

# Run benchmarks and capture output, stripping ANSI escape codes
BENCH_OUTPUT=$(cargo bench --bench simulation 2>&1 | sed 's/\x1b\[[0-9;]*m//g')

# Show full output
echo "$BENCH_OUTPUT"
echo ""

# Collect results for CSV and markdown
declare -a BENCH_NAMES=()
declare -a BENCH_NS=()
declare -a BENCH_DISPLAY=()

PREV_LINE=""
while IFS= read -r line; do
    if [[ "$line" == *"time:"*"["* ]]; then
        BENCH_NAME=$(echo "$line" | sed -E 's/[[:space:]]*time:.*//' | xargs)
        if [ -z "$BENCH_NAME" ]; then
            BENCH_NAME=$(echo "$PREV_LINE" | xargs)
        fi

        VALS=$(echo "$line" | sed -E 's/.*\[(.+)\]/\1/')
        MEDIAN=$(echo "$VALS" | awk '{print $3}')
        UNIT=$(echo "$VALS" | awk '{print $4}')

        # Normalize to nanoseconds
        case "$UNIT" in
            ps)  NS=$(echo "$MEDIAN" | awk '{printf "%.4f", $1 / 1000}') ;;
            ns)  NS="$MEDIAN" ;;
            µs|us)  NS=$(echo "$MEDIAN" | awk '{printf "%.4f", $1 * 1000}') ;;
            ms)  NS=$(echo "$MEDIAN" | awk '{printf "%.4f", $1 * 1000000}') ;;
            s)   NS=$(echo "$MEDIAN" | awk '{printf "%.4f", $1 * 1000000000}') ;;
            *)   NS="$MEDIAN" ;;
        esac

        # Human-readable display value
        DISPLAY="${MEDIAN} ${UNIT}"

        echo "${TIMESTAMP},${COMMIT},${BRANCH},${BENCH_NAME},${NS}" >> "$HISTORY_FILE"
        BENCH_NAMES+=("$BENCH_NAME")
        BENCH_NS+=("$NS")
        BENCH_DISPLAY+=("$DISPLAY")
    fi
    PREV_LINE="$line"
done <<< "$BENCH_OUTPUT"

COUNT=${#BENCH_NAMES[@]}

# ---------------------------------------------------------------------------
# Three-point tracking: baseline (first run), previous, latest
# ---------------------------------------------------------------------------

# Get distinct timestamps from CSV (skip header)
mapfile -t ALL_TIMESTAMPS < <(tail -n +2 "$HISTORY_FILE" | cut -d',' -f1 | sort -u)
NUM_RUNS=${#ALL_TIMESTAMPS[@]}

BASELINE_TS="${ALL_TIMESTAMPS[0]}"
if [ "$NUM_RUNS" -ge 3 ]; then
    PREV_TS="${ALL_TIMESTAMPS[$((NUM_RUNS - 2))]}"
elif [ "$NUM_RUNS" -ge 2 ]; then
    PREV_TS="${ALL_TIMESTAMPS[$((NUM_RUNS - 2))]}"
else
    PREV_TS=""
fi
LATEST_TS="${ALL_TIMESTAMPS[$((NUM_RUNS - 1))]}"

# Helper: look up a benchmark value from a specific timestamp
lookup_ns() {
    local ts="$1" bench="$2"
    grep "^${ts}," "$HISTORY_FILE" | grep ",${bench}," | tail -1 | cut -d',' -f5
}

# Helper: format ns value for display
format_ns() {
    local ns="$1"
    if [ -z "$ns" ]; then
        echo "—"
        return
    fi
    echo "$ns" | awk '{
        v = $1 + 0;
        if (v < 1)        printf "%.2f ps", v * 1000;
        else if (v < 1000) printf "%.2f ns", v;
        else if (v < 1000000) printf "%.2f µs", v / 1000;
        else               printf "%.2f ms", v / 1000000;
    }'
}

# Get baseline and latest commit info
BASELINE_COMMIT=$(grep "^${BASELINE_TS}," "$HISTORY_FILE" | head -1 | cut -d',' -f2)
LATEST_COMMIT=$(grep "^${LATEST_TS}," "$HISTORY_FILE" | head -1 | cut -d',' -f2)

# Generate benchmarks.md with three-point tracking
{
    echo "# Benchmarks"
    echo ""
    echo "Three-point tracking: **baseline** (first run) / **previous** / **latest**"
    echo ""
    echo "| Point | Date | Commit |"
    echo "|-------|------|--------|"
    echo "| Baseline | ${BASELINE_TS} | \`${BASELINE_COMMIT}\` |"
    if [ -n "$PREV_TS" ] && [ "$PREV_TS" != "$BASELINE_TS" ] && [ "$PREV_TS" != "$LATEST_TS" ]; then
        PREV_COMMIT=$(grep "^${PREV_TS}," "$HISTORY_FILE" | head -1 | cut -d',' -f2)
        echo "| Previous | ${PREV_TS} | \`${PREV_COMMIT}\` |"
    fi
    echo "| Latest | ${LATEST_TS} | \`${LATEST_COMMIT}\` |"
    echo ""

    CURRENT_GROUP=""
    for i in $(seq 0 $((COUNT - 1))); do
        NAME="${BENCH_NAMES[$i]}"
        GROUP=$(echo "$NAME" | cut -d'/' -f1)
        BENCH=$(echo "$NAME" | cut -d'/' -f2)

        if [ "$GROUP" != "$CURRENT_GROUP" ]; then
            if [ -n "$CURRENT_GROUP" ]; then
                echo ""
            fi
            echo "## ${GROUP}"
            echo ""
            if [ -n "$PREV_TS" ] && [ "$PREV_TS" != "$BASELINE_TS" ] && [ "$PREV_TS" != "$LATEST_TS" ]; then
                echo "| Benchmark | Baseline | Previous | Latest |"
                echo "|-----------|----------|----------|--------|"
            else
                echo "| Benchmark | Baseline | Latest |"
                echo "|-----------|----------|--------|"
            fi
            CURRENT_GROUP="$GROUP"
        fi

        BASE_VAL=$(format_ns "$(lookup_ns "$BASELINE_TS" "$NAME")")
        LATEST_VAL=$(format_ns "$(lookup_ns "$LATEST_TS" "$NAME")")

        if [ -n "$PREV_TS" ] && [ "$PREV_TS" != "$BASELINE_TS" ] && [ "$PREV_TS" != "$LATEST_TS" ]; then
            PREV_VAL=$(format_ns "$(lookup_ns "$PREV_TS" "$NAME")")
            echo "| \`${BENCH}\` | ${BASE_VAL} | ${PREV_VAL} | ${LATEST_VAL} |"
        else
            echo "| \`${BENCH}\` | ${BASE_VAL} | ${LATEST_VAL} |"
        fi
    done
    echo ""
    echo "---"
    echo ""
    echo "Generated by \`./scripts/bench-history.sh\`. Full history in \`bench-history.csv\`."
} > "$BENCHMARKS_MD"

echo "════════════════════════════════════════════"
echo "  ${COUNT} benchmarks recorded (${NUM_RUNS} runs in history)"
echo "  CSV:      ${HISTORY_FILE}"
echo "  Markdown: ${BENCHMARKS_MD}"
echo "════════════════════════════════════════════"
