# cw-stand-crouchrise2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-11T17:31:58+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-stand-holdbc1-hard1

**wandb_id**: 1gg41uez

**hypothesis**: DISCOVERY (2M, warm from cw-stand-holdbc1-hard1, NOT from crouchrise1 — its hold cheat may be baked in): crouchrise1 proved the crouch-60% start mix fixes the lineage's last defect (det crouch 5/5, RSI-off 8/8 vs hard1 0/8) but its second delta — goal-mix skew rise 0.45->0.6 / lower 0.45->0.3 — starved lower and let the flag-leg hold cheat back in (det hold feet 1+4 duty 0.07/0.01; valid_plant 7/12). crouchrise2 = ONE variable vs crouchrise1: restore hard1's exact goal mix (hold=0.1,rise=0.45,lower=0.45), keep goal.rise_flat_frac=0.10 / rise_partial_frac=0.30. Same holdbc1 stack, seed 13.

**gate**: PASS if det crouch-start rise valid >= 3/4 with zero tilt falls AND hold retention at hard1 level: hold det+sto valid_plant >= 10/12 AND det-hold per-foot contact duty >= 0.8 on ALL SIX feet (explicit — crouchrise1's flag-leg hold passed valid_plant=True; duty is the telemetry that caught it) AND det flat/bridge rise not worse than hard1, no flag-leg/tripod cheat on video. FAIL if crouch <= 2/4 or hold/duty regress. PASS -> replaces hard1 as the stance deploy candidate; the robot export MUST ship WITH its goal-ramp profile (08-10 stale-push lesson) and the stand lineage closes.

**verdict**: FAIL (clean isolation, lever CLOSED): crouch fix retained — gate det crouch rise 5/5 + bridge 1/1, zero falls, Imax 2.64A — but the det-hold flag-leg persists with hard1 exact goal mix restored: feet 1+4 contact duty 0.05/0.03 (crouchrise1: 0.07/0.01 — identical fingerprint; hard1: all six 0.90-0.99), hold det+sto valid_plant 9/12 (<10/12), hold/sto 3/6 vs hard1 5/6. Since the ONLY delta vs hard1 is the crouch-60% start mix, the start mix itself causes the hold cheat. Mechanism suspect: the BC anchor reference (rise_ref_belly2plant.npz) is CLOCK-indexed from episode start — crouch-start episodes see belly-phase reference poses against near-plant states, actively teaching lifted-leg postures in plant-adjacent states, which bleeds into hold. Two-miss rule: start-mix-without-side-effects lever CLOSES (crouchrise1 + crouchrise2). Next lever is CODE and needs SPECIFICATION first: state/height-aligned BC anchor for non-flat rise starts (or anchor gated off on crouch starts). hard1 REMAINS the stance deploy candidate; its crouch-start weakness (RSI-off 0/8) stands as a known limitation. Do not deploy crouchrise2, do not warm from it. Gate artifacts: train-0 logs/ckpt_eval/cw_stand_crouchrise2_gate.

