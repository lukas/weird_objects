# probe-walk-step0

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-08T23:15:48+00:00

**pod**: hexapod-sweep-lower

**steps**: 150000

**parent**: none (from scratch — operator-directed exception to warm-start default)

**hypothesis**: MECHANICAL+DIRECTIONAL PROBE (audit sec6) for operator queue item 0: step-event reward package (k_step_event=1.0, k_drag_loaded=10.0, k_park_duty=1.0) on walk-ONLY joint_walk, from scratch, DR 0, audited exploration (std 1.0 via log_std_init 0, ent 0.01, target_kl 0.02). If-true: all three reward parts appear in W&B-off trainer logs within audited bands (park ~-0.6/tick when parked during command, step events fire, drag <=0), no traceback, healthy fps. If-false: fix code before the 4M cw-walk-step0 launch next cycle. Directional bonus signal (no claim): step-event count trending up by 150k, as the phase probe showed six-leg cycling by 96k. Snapshot 9ccb3c8.

**gate**: mechanical only: healthy to 150k, reward_step_event/reward_drag/reward_park_duty present and in audited bands, no traceback

**verdict**: PROBE PASS mechanical (cycle 13): 159,744 steps, 0 tracebacks, parts in audited bands via local rollout of the ckpt (park_duty mean -0.306/tick floor -0.600 exact, drag -0.0004, step_event present). ZERO step events at 150k det — no directional signal, no claim. cw-walk-step0 4M is next cycle's first launch.

