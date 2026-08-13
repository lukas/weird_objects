# cw-gait-sched1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-13T01:08:42+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-gait-slowfirst1

**wandb_id**: rxptwtzd

**hypothesis**: Teach a from-scratch, no-imitation walking network to lift its feet by letting it learn to MOVE first and only then making foot-dragging expensive: the anti-skate stance charge ramps in gradually DURING the run (free until 0.5M steps, linearly up to the full audit price k=8000 by 1.5M) instead of being on from step 0 (cw-gait-dragstance1: froze into a park) or dropped all-at-once on an already-formed skater (cw-gait-anneal1: charge absorbed, never resolved). This is GAIT.md P3 lever 2 in its last untried form, using the new in-run coefficient scheduler (sched.* cfg keys, snapshot fae79ea, tests test_coef_sched.py green). Stack is byte-identical to cw-gait-dragstance1 (same charge endpoint, same income, same speed band 0.05-0.06) with the schedule as the ONE variable. NB: eval-harness reward panels are unreliable for the scheduled key (eval envs re-run the sched clock from ~0), judge on measured behavior metrics (slip_per_m, gait_valid, fwd travel) only.

**gate**: 2M DR0 det: (a) mobility retained: fwd travel med >=0.3 m (NOT the dragstance1/rsi1/slowfirst1 frozen fingerprint), AND (b) the ramped charge reshapes rather than being absorbed: slip/m med <=3.0 (anneal1's fixed-charge skate sat at 4.3-5.1) OR gait_valid >=1/6. If PASS: the scheduler is the first lever ever to move from-scratch gait; next rung = longer ramp at 20M hardening. If FAIL (freeze OR unresolved-charge skate again): every form of P3 lever 2 incl. in-run annealing is CLOSED and the recommendation to the operator is to close the from-scratch gait line.

