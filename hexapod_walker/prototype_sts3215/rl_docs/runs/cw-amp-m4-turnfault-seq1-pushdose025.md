# cw-amp-m4-turnfault-seq1-pushdose025

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-23T04:36:57+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1

**wandb_id**: 22j88kw5

**hypothesis**: Plain English: pushcont1 (push active 100% of training episodes, fault-first order) softened but did not fix push-driven turn erosion (tip err 0.27/0.30 vs fault-only parent's 0.18/0.17 vs every push-FIRST order's 0.38-0.49) -- closing the composition-ORDER search. This tests whether the erosion is DOSE-sensitive: if every training episode must pay to survive/recover a push, that competes with turn-tracking income for the same reward budget every single step; training with push active only 25% of episodes (same base 10-25N force, same eval-time push probability of 1.0 for the M5 push-section grade) should leave more turn-tracking budget intact. Single lever vs pushcont1: dr.ext_push_prob 1.0 -> 0.25. Re-inits from the SAME pre-cheat turnfault-seq1 checkpoint pushcont1 itself used (not pushcont1's own weights), per the 08-22 init-basin rule.

**gate**: Own-cfg DR-0 gate: gait_valid >=9/12 (fault-only parent's own floor, cleared at 11/12). Hand-run eval_yaw (hazards zeroed): tip-left AND tip-right err. PASS-clean = both <=0.20-0.25 (matches fault-only parent, order+dose together solve it). PARTIAL = between 0.20-0.25 and pushcont1's own 0.27/0.30 (dose helps, doesn't fully solve). FLAT = ~0.27-0.30 unchanged across doses (dose is not the lever, income-competition is structural regardless of exposure fraction -- escalate to the deferred hold/forward repricing build). Also run eval_amp_m5 push section (extra_cfg forces ext_push_prob=1.0 at eval time regardless of training dose) to confirm push-recovery competence survives the lower training dose.

**verdict**: Training-time push-probability dose sweep closes FLAT: lowering dr.ext_push_prob during training (1.0->0.25, 0.5, 0.75, all vs pushcont1's own 0.27/0.30 at dose=1.0) does NOT recover fault-only-parent turn tracking. This arm (dose=0.25): eval_yaw tip-left/right err 0.2642/0.2690 -- a small, non-dose-monotonic improvement over pushcont1 (0.27/0.30) shared by all three doses (see pushdose075 0.261/0.255, pushdose05b 0.239/0.257), all clustered in a narrow 0.24-0.27 band regardless of exposure fraction from 25% to 75%. Every dose still fails the <=0.20 m5 bar and the <=0.20-0.25 preserved band (05b's tip-left 0.2394 grazes the edge but tip-right 0.2571 misses). Mechanism-safety floor clears (gait_valid 11/12, 0 terms, 1 legit carried-fault leg [4]), video clean six-leg cycling, no new pathology. CONCLUSION: dose is not the lever -- even 25% push exposure triggers nearly the same turn-tracking cost as 100%, so the erosion is driven by the mere PRESENCE of push in the training distribution (a basin/pricing effect), not by cumulative exposure fraction. This is the pre-registered FLAT branch. Escalates the deferred hold/forward-income repricing build (q_20260823T0240Z item b) from 'parked' to 'next real M4/M5 lever' -- composition-order AND dose are now both closed.

