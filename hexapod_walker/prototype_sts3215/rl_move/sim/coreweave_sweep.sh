#!/bin/bash
# Fan out parallel PPO sim-training runs as pods on the CoreWeave cluster.
#
# Each run gets its own pod (48 CPUs — two fit per 128-core node beside
# other work), its own checkpoint prefix, and logs to the same W&B
# project. Usage, from prototype_sts3215/:
#
#   rl_move/sim/coreweave_sweep.sh "s1:--seed 1" "s2:--seed 2" \
#       "lowdr:--seed 0 --dr-scale 0.1"
#
# Spec format: <name>:<extra train_ppo_sim args>. Common base args below;
# override steps with STEPS=1000000 in the environment. Pods are named
# hexapod-sweep-<name> and left running after the run (kubectl delete
# them when done; `sleep infinity` keeps them exec-able for eval).
set -euo pipefail

KC="${KUBECONFIG_CW:-$HOME/.kube/coreweave.yaml}"
STEPS="${STEPS:-200000}"
N_ENVS="${N_ENVS:-48}"
# Pod sizing, overridable when the cluster is tight (CPU_REQ=24 fits in
# leftover node capacity at ~half throughput).
CPU_REQ="${CPU_REQ:-48}"
CPU_LIM="${CPU_LIM:-56}"
# NB: must stay ONE line — an embedded newline splits the remote nohup
# command in half and training silently never starts (2026-08-07).
BASE_ARGS="--steps $STEPS --n-envs $N_ENVS --subproc --dr-scale 0.2 --eval-every 100000 --video-every 100000"
PROTO_DIR="$(cd "$(dirname "$0")/../.." && pwd)"
SETUP="$PROTO_DIR/rl_move/sim/coreweave_pod_setup.sh"

[ $# -ge 1 ] || { echo "usage: $0 '<name>:<extra args>' ..."; exit 1; }

echo "[sweep] packing code from $PROTO_DIR"
tar -C "$PROTO_DIR" --exclude './logs' --exclude '*.stl' \
    --exclude '__pycache__' --exclude 'node_modules' \
    --exclude './rl_move/sim/coreweave.env' \
    --exclude './rl_move/sim/policies' --exclude '.DS_Store' \
    -czf /tmp/sweep_code.tgz .

launch_one() {
    local name="$1" extra="$2" pod="hexapod-sweep-$1"
    kubectl --kubeconfig="$KC" apply -f - <<EOF
apiVersion: v1
kind: Pod
metadata:
  name: $pod
  namespace: default
  labels: {app: hexapod-sweep}
spec:
  restartPolicy: Never
  containers:
    - name: train
      image: python:3.11-slim
      command: ["sleep", "infinity"]
      workingDir: /workspace
      env: [{name: MUJOCO_GL, value: osmesa}]
      resources:
        requests: {cpu: "$CPU_REQ", memory: 32Gi}
        limits: {cpu: "$CPU_LIM", memory: 64Gi}
EOF
    kubectl --kubeconfig="$KC" wait --for=condition=Ready "pod/$pod" --timeout=300s
    kubectl --kubeconfig="$KC" cp /tmp/sweep_code.tgz "$pod:/tmp/code.tgz"
    kubectl --kubeconfig="$KC" cp "$SETUP" "$pod:/tmp/setup.sh"
    kubectl --kubeconfig="$KC" exec "$pod" -- bash /tmp/setup.sh > /dev/null
    kubectl --kubeconfig="$KC" exec "$pod" -- bash -c \
        "cd /workspace/prototype_sts3215 && nohup python -m rl_move.sim.train_ppo_sim \
         $BASE_ARGS $extra --run-name cw-$name --out-name ppo_goal_cw_$name \
         > /tmp/train.log 2>&1 & echo '[sweep] $name: training started'"
}

pids=()
for spec in "$@"; do
    name="${spec%%:*}"
    extra="${spec#*:}"
    ( launch_one "$name" "$extra" > "/tmp/sweep_$name.launch.log" 2>&1 \
        && echo "[sweep] $name up" || echo "[sweep] $name FAILED (see /tmp/sweep_$name.launch.log)" ) &
    pids+=($!)
done
wait "${pids[@]}"
echo "[sweep] all launched. Watch: https://wandb.ai/l2k2/hexapod-balance"
echo "[sweep] logs: kubectl --kubeconfig=$KC exec hexapod-sweep-<name> -- tail /tmp/train.log"
