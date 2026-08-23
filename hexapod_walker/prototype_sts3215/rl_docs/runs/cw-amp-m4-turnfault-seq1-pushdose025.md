# cw-amp-m4-turnfault-seq1-pushdose025

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-23T04:36:57+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1

**hypothesis**: Plain English: pushcont1 (push active 100% of training episodes, fault-first order) softened but did not fix push-driven turn erosion (tip err 0.27/0.30 vs fault-only parent's 0.18/0.17 vs every push-FIRST order's 0.38-0.49) -- closing the composition-ORDER search. This tests whether the erosion is DOSE-sensitive: if every training episode must pay to survive/recover a push, that competes with turn-tracking income for the same reward budget every single step; training with push active only 25% of episodes (same base 10-25N force, same eval-time push probability of 1.0 for the M5 push-section grade) should leave more turn-tracking budget intact. Single lever vs pushcont1: dr.ext_push_prob 1.0 -> 0.25. Re-inits from the SAME pre-cheat turnfault-seq1 checkpoint pushcont1 itself used (not pushcont1's own weights), per the 08-22 init-basin rule.

**gate**: Own-cfg DR-0 gate: gait_valid >=9/12 (fault-only parent's own floor, cleared at 11/12). Hand-run eval_yaw (hazards zeroed): tip-left AND tip-right err. PASS-clean = both <=0.20-0.25 (matches fault-only parent, order+dose together solve it). PARTIAL = between 0.20-0.25 and pushcont1's own 0.27/0.30 (dose helps, doesn't fully solve). FLAT = ~0.27-0.30 unchanged across doses (dose is not the lever, income-competition is structural regardless of exposure fraction -- escalate to the deferred hold/forward repricing build). Also run eval_amp_m5 push section (extra_cfg forces ext_push_prob=1.0 at eval time regardless of training dose) to confirm push-recovery competence survives the lower training dose.

