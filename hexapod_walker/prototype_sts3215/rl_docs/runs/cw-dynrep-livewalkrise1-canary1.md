# cw-dynrep-livewalkrise1-canary1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-17T21:40:26+00:00

**pod**: hexapod-mjx-train-5

**steps**: 150000

**git_sha**: 8cc61c3de6309e7c9fb806a57b074047cab00ce5

**wandb_id**: m10szmx6

**hypothesis**: Prove the new live world-model machinery end to end before spending real compute: the walking-and-rising robot collects its own command-rich experience (all heading directions, stops and direction changes, commanded yaw; rises from flat/bridge/crouch and post-lower bank poses), an online dynamics transformer keeps learning from it on the GPU with stratified 75/25 walk/rise replay + 25% v5 rehearsal, and the critic-facing feature snapshot starts as exact frozen D and may only change at gated step boundaries. This canary judges the MECHANISM (quotas, bins, gates, actor isolation, checkpoints), never learning outcomes. Operator order fb_20260817T210422_9df9c7 arm A.

**gate**: MECHANISM-HEALTH CANARY ONLY: (1) CUDA - store tensors/predictor batches/online Adam state on CUDA (bank on-pod PASS); (2) actor isolation - pred/actor_kl_from_predictor exactly 0 throughout; (3) quotas - past warmup, batch fresh/rehearsal ~0.75/0.25 and fresh walk/rise ~0.75/0.25 (+/-10%); (4) coverage - data/added nonzero for cmd_change, start_stop, yaw_left+right, back/lat/diag, rise flat_bridge+crouch+post_lower; (5) snapshot discipline - version changes only at 50k boundaries, every attempt logs all six gate values, >=1 attempt exercised (accept OR reject both legal here); (6) W&B pred/*, critic/*, data/* advancing; (7) checkpoints <100MB, round-trip OK, online predictor saved separately. PASS -> launch cw-dynrep-livewalkrise1 (10M, boundary 1M, same cfg) immediately; FAIL -> fix + re-canary, no continuation.

**note**: Script-owned (pod_livewalkrise.sh, manifest livewalkrise_manifest.jsonl on-pod). Operator order fb_20260817T210422_9df9c7 arm A, mechanism canary before the 10M continuation. Code 8cc61c3d (main): live_replay.py stratified CUDA store + predictive_critic live mode (boundary-gated versioned snapshot). Tests: test_dynrep_live_replay.py 10/10 on-pod CUDA (incl. the CUDA store/optimizer proof), all prior dynrep banks 28/28, real-env F smoke on train-5 PASS (obs 16x73 yaw-cmd, md5 verified, pretrained heldout ref 2.286 exact). Canary boundary=50k exists ONLY to exercise the gated accept/reject machinery; the continuation uses the ordered 1M boundary.

