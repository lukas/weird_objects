# cw-walk-hist8

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-08T23:01:14+00:00

**pod**: hexapod-sweep-s3

**steps**: 4000000

**parent**: ppo_goal_cw_stance_dr10.zip (md5 da1d912a)

**wandb_id**: 4chv0m83

**hypothesis**: Walk failures are a CAPABILITY gap: a memoryless MLP cannot coordinate swing timing / do implicit system ID, so PPO converges to static exploits (shuffle, tripod park). 300ms env-side obs history (8 frames @ 25Hz, obs.history_frames=8, newest-first, obs 576) supplies the temporal state a gait needs. One-variable swap vs control cw-walk-phase-stance2 (same init/settings/walk cfg; phase package OUT, history IN). If-true: six-foot alternating contacts appear AND survive optimization; sto walk >=4/6 gait-valid @ vel_err <=0.035 @ DR 0.2. If-false: same tripod park (one tripod duty ~1.0, other <=0.3) -> refutes capability-alone, promotes rung 2 (time-averaged per-leg load pricing on top of history). Strongest alternative: park is optimal regardless of capability because unpriced. Probe probe-walk-hist8 PASSED (mechanical; canary baseline = parent on-pod). No asym-critic (refused with transplant). Snapshot e85a290.

**gate**: sto walk >=4/6 gait-valid @ vel_err <=0.035 on 0.02-0.06 @ DR 0.2 AND video shows all six feet cycling contact/swing AND sto rise >=4/6 retained (canaries armed from stance parent)

**verdict**: AUTO-STOPPED 1.24M/4M (canary 'lower' 3-streak) — by design. Harness: walk 0/12 gait-valid (tripod park legs 1/3/5 duty ~0.9+), rise/lower/hold 0/12 posture-strict, std 1.0->1.31. Control phase-stance2 survived 4M: 576-wide fresh-Adam first layer drifts ~3x faster. Capability question INCONCLUSIVE (under-dosed); retention risk of wide transplant at std 1.0 CONFIRMED. ckpt md5 eddfc2d0. Next: history as one-variable arm on the walk-only step0 baseline (no retention tension).

