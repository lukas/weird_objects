# cw-amp-m4-turnfault-seq1-faultdose025

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INFORMATIVE

**created**: 2026-08-23T05:03:54+00:00

**pod**: hexapod-mjx-train-4

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1

**wandb_id**: moi7y2vi

**hypothesis**: Plain English: every M4 3-4-axis composed checkpoint so far (pushfault1-noamp-acq1, turnfault-seq1-pushcont1/pushdose*) trains with dr.fault_prob=1.0 PERMANENTLY -- every episode carries a fault -- so eval_amp_m5's walk section (which grades a hazard-free mode) and fault section (max_terms<=2 on top of an already-faulted walk) come out numerically IDENTICAL to each other and both fail on terms/gait_valid, because there is no clean fault-free walking mode in the policy's repertoire to test (open question q_20260823T0130Z, flagged not adjudicated). This mirrors exactly what the push-probability dose sweep just tested for push: does lowering the TRAINING-time fault probability (not the eval-time one, which m5's fault section still forces to 1.0) let the policy retain a genuine clean walk mode, fixing the walk-section design tension? Single lever vs pushcont1: dr.fault_prob 1.0 -> 0.25, dr.ext_push_prob left at 1.0 (matching pushcont1 exactly, single-axis change). Re-inits from the SAME pre-cheat turnfault-seq1 checkpoint per the init-basin rule.

**gate**: Own-cfg DR-0 gate: gait_valid>=9/12. eval_amp_m5 walk section (own cfg, hazards as-baked=0.25 fault/1.0 push -- NOTE: this reads the TRAINING dose, not a clean/off draw, so a real walk-section fix needs the harness to actually EXCLUDE fault episodes, which it currently does not; record the raw m5 walk numbers regardless as the direct test of whether lower training exposure alone changes the observed terms/gait_valid distribution across the 12-episode panel). PASS-signal = walk-section terms/gait_valid measurably improve vs pushcont1's 4 terms/10-12 gait_valid (some fraction of the panel now draws no fault and behaves cleanly). FLAT = unchanged (the walk section's identical-to-fault-section reading is a harness/curriculum-design issue, not exposure-fraction-fixable, and needs a harness change -- e.g. force fault_prob=0 for the walk section specifically -- rather than a training-dose fix). Also report eval_yaw tip errs as a secondary read (does less fault exposure change turn tracking at all, given fault-only alone never hurt turn).

**verdict**: Tests the q_20260823T0130Z design-tension question: does lowering dr.fault_prob during training (1.0->0.25, ext_push_prob left at 1.0, single lever vs pushcont1, re-init from the pre-cheat turnfault-seq1 checkpoint) let eval_amp_m5's walk section draw a genuinely hazard-free episode fraction and clear its own bar. Partial yes: walk-section reads gait_valid 12/12 (vs pushcont1's 10/12) but terms=2 (vs pushcont1's 4, still >0 so misses the strict terms=0 bar) -- a real, if noisy (n=12), improvement in both directions from pushcont1's baseline, consistent with SOME episodes now drawing no fault and behaving cleanly. Secondary read (fault-only was never the turn driver, expected no effect): eval_yaw tip-left/right 0.2241/0.2533, same cluster as every other M4 3-4-axis arm, confirms fault dose doesn't move turn tracking. Mechanism-safety floor and video clean.

