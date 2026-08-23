# cw-arch-hist16-dep1-c1-joyfullcurr6

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-23T22:21:18+00:00

**pod**: hexapod-mjx-train-0

**steps**: 40000000

**parent**: cw-arch-hist16-dep1-c1

**hypothesis**: Plain English: teach the 16-frame deployment-contract walker (hist16-dep1-c1) to follow joystick commands in EVERY direction - sideways, backwards, full circle - via a heading-band curriculum (operator order fb_20260823T220651_5c66e3). The lineage only ever trained front-cone (+-45 deg) commands. New WALKCURR_BUCKETS_V6 ladder: bridge at the source operating point (0.05-0.06 m/s straight), then front45 20/60s -> side90 20/60s -> rear135 40s -> rear180 60s -> full-circle 60s -> retain under DR 0.2/0.5. Reward adds the bank-proven direct loaded-slip charge (k_loadslip_excess=0.8, ok 1.2, max 3.0) and smoothed course-direction charge (k_walk_course=1.0, tau 0.75s) so training prices exactly what the full-circle drive eval measures; optional k_walk_cmd_track omitted to keep the coupled-change count down. If-true: precert B0 passes and the frontier promotes past side90 with real rear-heading command following (cmd_prog>=0.65, slip<=2.0, 0 falls at cert). If-false: precert fails (broken transplant) or frontier stalls at side90/rear while reward rises - that is a reward/eval misalignment audit per the 08-21 ruling, not a same-recipe seed relaunch. Strongest alternative: the gait is already heading-symmetric and rear rungs pass trivially (would show as fast full-ladder promotion, still a win).

**gate**: walkcurr V6 precert B0 prog>=0.50; frontier past side90_60s via det certs (cmd_prog>=0.65, slip/m<=2.0, 0 falls); FINAL: eval_drive full-circle --heading-max-deg 180 DR0.5 random 60s flips: 0 falls, rear/side directions actually followed (low dir err/wrong-way), slip not exploding vs parent; DR0 + own-DR walk gates 6/6 gait_valid 0 term; video all six feet cycling, no flag leg

**refused_reason**: hexapod-mjx-train-0 code marker 75863b8119a452cd35b4fde2447e427e99a3b6b0 != local HEAD e27969b4c41214eeff9077c2ca2673ac436f6497 and the delta is not benign-orchestrator-only. Sync first: snapshot.sh --sync hexapod-mjx-train-0 (and snapshot/commit before that if the tree is dirty).

