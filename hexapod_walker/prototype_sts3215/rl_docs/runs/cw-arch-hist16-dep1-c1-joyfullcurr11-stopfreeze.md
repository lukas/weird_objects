# cw-arch-hist16-dep1-c1-joyfullcurr11-stopfreeze

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: KILLED

**created**: 2026-08-24T08:58:25+00:00

**pod**: hexapod-mjx-train-0

**steps**: 40000000

**parent**: cw-arch-hist16-dep1-c1-joyfullcurr9-stopcur2

**wandb_id**: z1ul6hi9

**hypothesis**: Plain English: this cycle's stopfreeze-probe diagnostic proved that FORCING a physical hold on stop commands (discard the policy's own action past a 0.4s grace, re-issue its last safe command) passes the V6 b1 stop cert outright on an unchanged checkpoint -- stop_speed_m_s 0.0133 vs the 0.015 bar, more than halving the 0.027-0.048 plateau every reward-pricing arm in this ladder (joyfullcurr6/7/8/9/10-chg2/chg4, 7 arms) has been stuck in. That was an offline ~15s dry-run read on ONE bucket though; walk_stop_freeze_s defaults OFF so the curriculum's own periodic in-training cert never saw it. This run wires goal.walk_stop_freeze_s=0.4 into an ACTUAL training continuation warm-started from the current best checkpoint (stopcur2: 1/48 held-out joygate falls, over_current solved) so the live b1 cert -- which gates promotion to buckets b2-b9 (side90/rear/full-circle, the actual point of the operator's full-circle order, never practiced by any arm in this ladder) -- can finally clear using the SAME mechanism the diagnostic just proved.

**gate**: If walkcurr/frontier leaves b1 (promotes to b2+) at any point: the structural hold unlocks the ladder for the first time -- read the resulting buckets' held-out joygate falls/slip/dir_err AND video-check for any NEW pathology the freeze itself could introduce (e.g. holding a mid-swing foot placement, a stumble right as a stop is commanded) before calling ladder progress a clean pass. If frontier stays pinned at b1 through the full budget despite the diagnostic's pre_b1_pass=1: the live in-training cert differs from the offline probe in some way (DR/command-seed variance during training rounds, or a training-time interaction with the override) -- dig into the per-round walkcurr/b1_front45_20s/stop_speed_m_s trace before any further stop lever. Either way report whether over_current falls stay solved (stopcur2's win) -- the freeze must not reopen that axis.

**verdict**: DUPLICATE, killed before any real training progress: a concurrent decision cycle independently launched cw-arch-hist16-dep1-c1-joyfullcurr11-freeze40 ~2 min later off the SAME parent checkpoint (stopcur2), SAME seed (0), and the SAME cfg (goal.walk_stop_freeze_s=0.4) -- identical hypothesis, zero differentiation. Killed this one (created first but let freeze40 stand as the surviving arm, already past INTENT) rather than spend two GPUs on a config with no informational delta. No verdict on the underlying hypothesis -- see joyfullcurr11-freeze40 for the real answer.

