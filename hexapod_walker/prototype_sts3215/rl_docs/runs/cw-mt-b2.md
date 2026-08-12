# cw-mt-b2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-12T20:30:26+00:00

**pod**: hexapod-mjx-train-1

**steps**: 20000000

**parent**: cw-mt-b1

**wandb_id**: 4acq029q

**hardware_ready**: False

**hypothesis**: Teach one fresh policy to walk, stop, and turn on command all at once, and see whether the gait that emerges is steerable — this arm re-runs the narrow generalist (speeds 0-0.06, occasional +/-0.15 rad/s yaw commands, 40% stop segments) at the 20M budget the recipe actually needs, after the 2M wave proved under-budget for every arm. Prediction-if-true: forward walking emerges AND zero-command segments hold still AND yaw probes respond sign-correctly both ways. Prediction-if-false(acquisition): forward emerges but yaw response never arrives — the narrow-local-optimum story survives even for a fresh generalist. Strongest alternative: everything works numerically but the gait is the same paddle as the sequential champion — judged on video per MULTITASK.md.

**gate**: Video-first at 20M: (1) zero-command segments hold still (no march-in-place); (2) forward det prog med >= 0.5x cw-mt-a2's; (3) yaw probes at +-0.15 rad/s turn the correct way BOTH directions. PASS = all three. FAIL(budget) if nothing walks at 20M; FAIL(acquisition) if forward emerges but yaw never responds. Labels binding per MULTITASK.md.

**verdict**: FAIL(acquisition-shortfall) per MULTITASK.md labels: a real six-leg gait finally emerges at 20M (gait_valid 6/6 det+sto, both DR0 and own-DR0.2 — unlike wave-1's 0/6 paddle) but clause 2 fails on the numbers now that the control lands: det prog med 0.51 (gate)/0.53 (own-cfg) vs the required >=0.5x cw-mt-a2's 1.23/1.30 (need >=0.615/0.65) — short on both. Clause 3 (extra eval_yaw probe, not auto-staged) also fails: turn |wz_err| med 0.137 (gate<=0.10), hold |wz| med 0.050 (gate<=0.05, borderline), and 9 falls across the 10 scripted scenarios — yaw partially responds (arc-left/right differ from hold) but is unreliable, not clean bothdirections tracking. Clause 1 (stop segments hold still) looks fine on video+the isolated stop-hold probe (0 falls, low wz). Quality caveat for the record: leg index 3 sits at duty 0.11-0.35 in EVERY one of 24 episodes (gate+owncfg x det+sto), well below the other five legs' 0.15-0.85 spread and repeatedly close to the 0.10 auto-sacrifice cutoff -- gait_valid never trips but this is not a clean symmetric six-leg gait; a2 (no yaw/stop diversity, same seed/recipe) shows no such leg and a healthier 0.26-0.72 spread, so the added command diversity looks like the proximate cause, not a generic recipe artifact.

