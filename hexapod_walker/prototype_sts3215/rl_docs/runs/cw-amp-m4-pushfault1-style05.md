# cw-amp-m4-pushfault1-style05

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T02:16:32+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m4-faultobs2-headingsfull-style05

**wandb_id**: g0byttti

**hypothesis**: Plain English: style05 twin of cw-amp-m4-pushfault1-noamp-r2 -- does the AMP style channel help or hurt when push-recovery and fault-compensation are BOTH active in the same episode (the hardest off-distribution stack tested yet)? Every prior M2 axis found style neutral, and fault-alone was also neutral (faultobs2-headingsfull-style05 PASS-neutral vs noamp); push-alone was neutral too. This is the first joint two-off-distribution-axis test of that neutrality claim. Single lever vs cw-amp-m4-faultobs2-headingsfull-style05 (the PASSed fault-sighted+style full-heading checkpoint): add dr.ext_push_prob=1.0, same obs-pad-transplant=0 fix as the noamp twin (its own -r1 attempt LAUNCH_CRASHED on the stale =18 value). Prediction-if-true (style still neutral): paired against the noamp twin, gait_valid/terminations/prog/slip land within the same noise band as every prior style-vs-noamp comparison. Prediction-if-false: style actively vetoes the compensating-limp-while-recovering motion (worse gait_valid or prog than noamp) -- the first real negative style finding, on the hardest stack yet.

**gate**: Discovery mechanism-safety bar (2M, DR-0, own cfg, dr.fault_prob=1.0 AND dr.ext_push_prob=1.0 both active): gait_valid >=9/12 det+sto, no crouch, video confirms compensation not statue. Joint read (same seed=0, paired episodes) against cw-amp-m4-pushfault1-noamp-r2's own numbers on prog_ratio/slip/gait_valid -- WASH if within 6-episode noise (style stays neutral, 8th-ish confirmation), STYLE-HELPS or STYLE-HURTS if the delta is decisive, matching the paired-comparison method faultobs2's own sighted-vs-blind trio used.

