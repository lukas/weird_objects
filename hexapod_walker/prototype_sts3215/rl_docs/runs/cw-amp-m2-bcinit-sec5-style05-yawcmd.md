# cw-amp-m2-bcinit-sec5-style05-yawcmd

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-22T22:07:06+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m2-bcinit-sec5-style05-headingsfull

**wandb_id**: 506whm12

**hypothesis**: Plain English: the AMP walker follows translation commands in every direction -- can it now learn to TURN on command (yaw-rate channel, +/-0.3 rad/s), something every reward-tuning attempt on the OLD walking-champion substrate failed at because that policy carried a baked-in left-yaw drift? This substrate is different in two ways that make the question worth re-asking: the BC-tripod init has no known chirality drift, and the AMP motion library (teacher_v2) contains REAL turn-in-place clips, so the style channel rewards genuine turning motion directly. Continues from the headingsfull checkpoint (obs 73->74 via --obs-pad-transplant 1, the same proven tail-append mechanism as yawcmd1/quad-turn1-r1; fresh disc), adding the FULL landed bank-verified turn pricing set (k_walk_yaw=1 + walk_yaw_kernel_gate + k_yaw_prog + k_yaw_still + walk_kernel_yaw_gate + the 08-11 hold_prog_gate/yaw_still_avg_s fixes -- NOT the known-failed kernel-only subset of yawcmd1/yawgate1; OMNI semantics bank verified this stack prices honest turning above drift, full bank currently green). Prediction-if-true: turn-segment yaw tracking emerges (|wz| sign follows wz_ref, wz_err visibly below the command-invariant ~0.24 fingerprint) with gait_valid >=5/6 det+sto and translation dir_err not degrading >15deg vs headingsfull. Prediction-if-false: wz stays command-invariant despite clean pricing on a drift-free substrate -- turning is structurally hard (gait-timing conflict), and the amp track needs a dedicated turn curriculum stage or mirror-symmetry regularizer, not more pricing. Strongest alternative: turning emerges but translation erodes (kernel competition on the minimal sec5 diet).

**gate**: Discovery continuation (2M, DR-0). INFORMATIVE-PASS = gait_valid >=5/6 det+sto at own cfg, no sacrificed legs, height_err 18-31mm band, translation dir_err within 15deg of headingsfull (det ~33/sto ~50), AND yaw response is command-SIGNED (achieved wz correlates with wz_ref sign on turn segments; not the flat command-invariant drift fingerprint). FAIL-collapse = statue/drag/sacrificed legs. FAIL-ignore (informative) = gait clean but wz command-invariant -- closes cheap-pricing turning on this substrate, next lever is a turn-in-place curriculum stage.

