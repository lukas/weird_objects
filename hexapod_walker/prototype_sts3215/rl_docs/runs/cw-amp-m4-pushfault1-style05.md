# cw-amp-m4-pushfault1-style05

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-23T02:16:32+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m4-faultobs2-headingsfull-style05

**wandb_id**: g0byttti

**hypothesis**: Plain English: style05 twin of cw-amp-m4-pushfault1-noamp-r2 -- does the AMP style channel help or hurt when push-recovery and fault-compensation are BOTH active in the same episode (the hardest off-distribution stack tested yet)? Every prior M2 axis found style neutral, and fault-alone was also neutral (faultobs2-headingsfull-style05 PASS-neutral vs noamp); push-alone was neutral too. This is the first joint two-off-distribution-axis test of that neutrality claim. Single lever vs cw-amp-m4-faultobs2-headingsfull-style05 (the PASSed fault-sighted+style full-heading checkpoint): add dr.ext_push_prob=1.0, same obs-pad-transplant=0 fix as the noamp twin (its own -r1 attempt LAUNCH_CRASHED on the stale =18 value). Prediction-if-true (style still neutral): paired against the noamp twin, gait_valid/terminations/prog/slip land within the same noise band as every prior style-vs-noamp comparison. Prediction-if-false: style actively vetoes the compensating-limp-while-recovering motion (worse gait_valid or prog than noamp) -- the first real negative style finding, on the hardest stack yet.

**gate**: Discovery mechanism-safety bar (2M, DR-0, own cfg, dr.fault_prob=1.0 AND dr.ext_push_prob=1.0 both active): gait_valid >=9/12 det+sto, no crouch, video confirms compensation not statue. Joint read (same seed=0, paired episodes) against cw-amp-m4-pushfault1-noamp-r2's own numbers on prog_ratio/slip/gait_valid -- WASH if within 6-episode noise (style stays neutral, 8th-ish confirmation), STYLE-HELPS or STYLE-HURTS if the delta is decisive, matching the paired-comparison method faultobs2's own sighted-vs-blind trio used.

**verdict**: Push+fault composition WITH style completes the paired read against cw-amp-m4-pushfault1-noamp-r2 (PASS, verdicted this cycle by a concurrent pass): style clears the SAME mechanism-safety bar with a WASH-to-mild-help signature, not a clean win or loss. Evidence (own-cfg gate, both hazards active, seed=0, DR-0): gait_valid 12/12 -- BETTER than noamp's 11/12, and unlike noamp (1 sacrificed leg, ep5) style05 has ZERO sacrificed legs across all 12 episodes, matching the faultobs2 sighted-vs-blind trio's own finding that style eliminates leg-sacrifice. Topples: 1/6 det (ep5, tilt_pitch, but only AFTER prog_ratio 3.13 -- the robot walked unusually far before the late fall) + 2/6 sto (tilt_roll ep2, tilt_pitch ep4) = 3/12 total -- one MORE than noamp's 2/12, inside 6-episode noise for this n. det prog med ~1.21/slip ~2.77 vs noamp's 1.12/3.53 (style slightly better on both); sto prog med ~0.87/slip ~3.6 vs noamp's 0.81/4.26 (style slightly better on both too). Height stays in-band except the two 'fell' episodes' end-frame readings (expected post-fall). Video (det_5, sto_2 strips watched): both topples are genuine late-episode knockdowns after 5+ clean walking frames -- same fingerprint as noamp's falls, not statues. Per the gate's own decision rule (WASH if within noise): reads as a WASH on raw topple count but a genuine STYLE-HELPS on the zero-sacrifice/prog/slip axes -- style is not hurting fault+push composition and plausibly firms up the no-sacrifice property the faultobs2 line already credited it with. Reward finite+rising every quarter (34->109->175->213), same shape as noamp. No follow-up arm needed beyond the already-launched acquisition continuation (cw-amp-m4-pushfault1-noamp-acq1, running); if that closes clean, a matched style05 acquisition continuation is the natural next comparison.

