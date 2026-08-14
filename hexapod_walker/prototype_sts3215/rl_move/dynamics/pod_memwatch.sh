#!/bin/sh
# pod_memwatch.sh — container-OOM guard (train-10 was OOMKilled 08-14
# ~09:40 UTC at the 96Gi pod limit ~3h into the chained holdwalk cohort,
# taking the WHOLE pod and every overlay-fs artifact with it: pods mount
# no persistent volume, so a cgroup OOM kill loses everything).
#
# Every 60s: append container memory.current + the largest-RSS process
# to logs/memwatch.log. Above THRESH_GIB (default 85), kill -9 the
# single largest python process — one job dies loudly, the pod (and all
# other jobs/artifacts) survive.
#
#   nohup sh rl_move/dynamics/pod_memwatch.sh > /dev/null 2>&1 &
cd "$(dirname "$0")/../.."
LOG=rl_move/dynamics/logs/memwatch.log
THRESH_GIB=${THRESH_GIB:-85}
THRESH_B=$((THRESH_GIB * 1024 * 1024 * 1024))
mkdir -p "$(dirname "$LOG")"
echo "== memwatch start $(date -u +%FT%TZ) thresh=${THRESH_GIB}GiB" >> "$LOG"
while :; do
    CUR=$(cat /sys/fs/cgroup/memory.current 2>/dev/null || echo 0)
    TOP=$(ps -eo rss,pid,args --sort=-rss --no-headers 2>/dev/null \
          | head -1 | cut -c1-160)
    echo "$(date -u +%FT%TZ) total=$((CUR / 1048576))MiB top: $TOP" >> "$LOG"
    if [ "$CUR" -gt "$THRESH_B" ]; then
        PID=$(ps -eo rss,pid,comm --sort=-rss --no-headers \
              | awk '$3 ~ /python/ {print $2; exit}')
        if [ -n "$PID" ]; then
            echo "$(date -u +%FT%TZ) MEMWATCH_KILL pid=$PID cmdline: $(tr '\0' ' ' < /proc/$PID/cmdline 2>/dev/null | cut -c1-200)" >> "$LOG"
            kill -9 "$PID"
        fi
    fi
    sleep 60
done
