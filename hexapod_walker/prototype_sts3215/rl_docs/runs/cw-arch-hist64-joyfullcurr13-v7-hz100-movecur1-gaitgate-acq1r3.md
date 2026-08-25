# cw-arch-hist64-joyfullcurr13-v7-hz100-movecur1-gaitgate-acq1r3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-25T06:24:31+00:00

**pod**: hexapod-mjx-train-10

**steps**: 38000000

**parent**: cw-arch-hist64-joyfullcurr13-v7-hz100-movecur1-gaitgate

**wandb_id**: htgxjxjo

**hypothesis**: Plain English: combined-lever sibling continuation -- movecur1-gaitgate (walk_gait_gate=1.0 + k_walk_move_current=2.0) is ALSO statistically indistinguishable from its uncharged/ungated matched-step reference at 2M (same near-identical over_current/height/reward trajectory shape as the plain movecur1 read), so this 2M canary can't yet tell whether the gate+charge combo helps once the policy starts really walking. Continuing to the same 40M budget the acq1/tf64-mesh-acq1 references got, both levers held on, to see whether combining them prevents BOTH known failure modes (gaitgate-scratch1's gameable satisfy-then-lock AND acq1's raw over-current lock) once cmd_prog actually moves past ~0.

**gate**: PASS: by ~38-40M, held-out joygate falls stay low (matching or beating the 12/48 ungated-scratch1 baseline, NOT gaitgate-scratch1's 39/48) AND over_current/Imax stay suppressed AND frontier/promotions move past b0. PARTIAL: over_current suppressed but frontier stuck at b0 (dig-in territory, not blind extension) or falls improve but gait_valid stays 0. FAIL: reproduces gaitgate-scratch1's WORSE-than-ungated joygate signature (39/48-class falls) or acq1's raw over_current lock -- the combination doesn't rescue either lever; drop the gate for this lineage and treat k_walk_move_current alone (movecur1-acq1r2) as the live arm.

**verdict**: Result: FAIL exactly per the pre-registered FAIL branch -- combining walk_gait_gate=1.0 with k_walk_move_current=2.0 rescues NEITHER failure mode at full 38M budget; it reproduces BOTH. Evidence: held-out joygate 31/48 falls (dr0 panel 23/24!) vs the 12/48 ungated-scratch1 baseline, dir_err med 52.7deg (allow 40), gait_valid_frac 0.0 => joygate FAIL; DR-0 gate panel 23/24 over_current terms, gait_valid 0/24, prog med 0.23-0.34, slip up to 7.3, persistent sacrificed legs [1,3] (the locked-leg tripod signature of the uncharged acq1 refs); contact sheet shows the sprawled body-drag, not six-leg walking. Reward rose (Q4 165) but that optimum IS the OC-lock -- the gate mechanism was the realignment attempt and it failed. Why: the gait gate withholds income the policy can satisfy-then-lock, and the move-current charge alone never prices the locked tripod out; together they still leave the drag basin as the local optimum. What's next (pre-registered): drop walk_gait_gate for this hist64 lineage; k_walk_move_current alone (movecur1-acq1r2, concurrent cycle) is the live arm, with tf64-mesh-movecur1-acq1 still training for the arch axis. No new launch from this verdict.

