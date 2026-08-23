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

**verdict**: Push-training-dose is NOT the lever that saves M4 turn-tracking. Cutting training push exposure from pushcont1's 1.0 to 0.25 (same base 10-25N force, same fault-first init) barely moves tip error: hand-run eval_yaw (hazards zeroed) reads tip-left/right 0.2642/0.2690 vs pushcont1's own 0.2727/0.3029 -- inside single-eval noise of 'unchanged', nowhere near the pre-registered PASS-clean bar (both <=0.20-0.25) and matching the gate's own pre-registered FLAT branch, not PARTIAL. Own-cfg DR-0 walk floor holds (gait_valid 11/12 det+sto, matches parent's 11/12 exactly, 0 terminations, one legit carried leg [4], clean six-leg video). Worse: the paired eval_amp_m5 push section (eval-time forced ext_push_prob=1.0, per the gate's own 'confirm push-recovery survives the lower dose' instruction) shows push-recovery competence did NOT survive -- sto terminations under forced push jumped 2 (pushcont1) -> 5 (this run), busting the section's own cap of <=3, even though gait_valid stayed fine (12/12). So the dose cut buys no turn-tracking headroom and actively costs push robustness on the one axis it was supposed to protect -- the income competition between yaw-tracking and push-survival is structural, not proportional to training exposure fraction. Training reward rose all run (39/101/173/203 per quarter) but this is a pre-registered behavioral grid, not an undertrained lineage -- the FLAT branch's own prescription is to escalate to the deferred hold/forward repricing build, not to relaunch more dose points on this substrate. Evidence: logs/ckpt_eval/cw_amp_m4_turnfault_seq1_pushdose025_{gate,m5}/, cf. cw_amp_m4_turnfault_seq1_{,pushcont1_}m5/m5_verdict.json.

