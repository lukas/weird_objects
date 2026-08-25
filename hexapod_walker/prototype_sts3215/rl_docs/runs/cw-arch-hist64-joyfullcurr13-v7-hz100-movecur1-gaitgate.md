# cw-arch-hist64-joyfullcurr13-v7-hz100-movecur1-gaitgate

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-25T05:03:10+00:00

**pod**: hexapod-mjx-train-6

**steps**: 2000000

**parent**: cw-arch-hist64-joyfullcurr13-v7-hz100-gaitgate-scratch1

**hypothesis**: Plain English: combined-lever sibling of movecur1 -- does adding the direct current-dwell charge (reward.k_walk_move_current=2.0) ON TOP of the min-across-legs anti-sacrifice gate (reward.walk_gait_gate=1.0, left ON exactly as gaitgate-scratch1 trained it) fix what neither lever fixed alone? gaitgate-scratch1 (gate alone) made the held-out fall rate WORSE (39/48 vs ungated 12/48) via a gameable satisfy-then-lock exploit; if the current charge removes the INCENTIVE to lock at all (by making sustained near-trip current itself expensive, not just ungated leg-cycling), the gate's min-across-legs requirement might then land on genuinely cycling legs instead of a token-swing dodge. Same V7/100Hz/hist64/mesh recipe, byte-identical to gaitgate-scratch1 except this one added lever.

**gate**: PASS: training reward improves AND walk_move_current_max_a stays mostly under ~2.2A AND eval/walk/survived_frac shows real nonzero stretches AND (bonus, not required for PASS) per-leg duty_cycle in an early frame-strip spot-check does not show the {1,3,5}-vs-{0,2,4} lock pattern. PARTIAL: over_current frequency drops vs gaitgate-scratch1 but survived_frac stays mostly 0. FAIL: indistinguishable from gaitgate-scratch1 at matched steps -- the combination doesn't help beyond the charge alone (compare against movecur1's own read), or the gate actively interferes with the charge's incentive.

