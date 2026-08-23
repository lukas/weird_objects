# cw-amp-m3-pushcur1-noamp-repeat2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-23T00:49:34+00:00

**pod**: hexapod-mjx-train-3

**steps**: 6000000

**parent**: cw-amp-m3-pushacq1-noamp

**wandb_id**: dkgr0fpb

**hypothesis**: Plain English: the jump from one shove to three per episode gave the walker nothing to learn from (repeat3-r1: reward and tilt-terms flat for the whole 6M) — this tests whether staging the COUNT axis one rung at a time unlocks it, training at dr.ext_push_repeat_max=2 (up to TWO 10-25N shoves, gaps 1-3s) from the clean single-shove champion. Everything else identical to repeat3-r1. Prediction-if-true: tilt terms START high and FALL (unlike repeat3-r1's flat-from-Q1 line) and the gate at repeat_max=2 holds topples <=1/6 det + <=2/6 sto — count is learnable in rungs, and repeat3 becomes stage 2 from this checkpoint. Prediction-if-false: terms flat here too — the second shove during recovery is a qualitatively different problem (mid-recovery balance) that dose staging cannot reach, naming a recovery-state curriculum or longer episodes as the real M3 lever. Strongest alternative: repeat2 is already almost free from the single-shove prior (terms start AND stay low, gate clean) — then the plateau at 3 is about shove DENSITY vs episode length, not count per se.

**gate**: Hardening stage 1 on the count axis (6M, DR-0, 10-25N, repeat_max=2, from pushacq1-noamp ckpt). PASS = DR-0 own-cfg gate gait_valid >=5/6 det+sto, zero sacrificed legs, det prog med >=0.9, topples <=1/6 det AND <=2/6 sto, video shows surviving BOTH shoves in-episode. On PASS the pre-registered follow-up is repeat_max=3 from THIS checkpoint. INFORMATIVE-plateau = topples above bar with training terms flat over last 2M => count axis not unlockable by staging, names recovery-state mechanism for M3. FAIL = collapse/statue/NaN.

**verdict**: Count rung 2 is clean: repeat_max=2 (two 10-25N shoves/ep) from pushacq1-noamp lands 0/6 det + 2/6 sto topples (bar <=1/6 det, <=2/6 sto), gait_valid 12/12, zero sacrificed, det prog med 1.12. Strips watched: det_4 absorbs a 22deg shove and returns to upright six-leg cycling. CAVEAT — the pre-named strongest-alternative fired: tilt terms started low AND stayed low (pitch 13->8.5, roll 7->5/window vs repeat3-r1's flat 15-17), i.e. rung 2 was largely FREE from the single-shove prior, so this run does not yet prove count is stageable — rung 3 is the real test. Next: pre-registered repeat_max=3 from THIS checkpoint (pushcur2-noamp-repeat3); if terms are flat there too, the plateau at 3 shoves is mid-recovery balance/shove density, not count per se — recovery-state mechanism becomes the named lever.

