# cw-amp-m2-turnclone-yawcmd0

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: LAUNCH_CRASH

**created**: 2026-08-23T00:03:20+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-amp-m2-bcinit-sec5-style05-yawcmd

**hypothesis**: Plain English: the zero-dose control of the turnclone batch -- no dedicated turn-in-place episodes, just the original yawcmd command mix (arc turns while translating + occasional stress_mix zero-speed segments). Does a BC init that was actually taught to turn (see tip50/tip90-turnclone siblings) do better than the original yawcmd run (which parked on turn-in-place, tip err 0.2995/0.3008 == |wz_ref|, while carrying a command-invariant left drift during translation) even WITHOUT dedicated tip exposure? Single lever vs cw-amp-m2-bcinit-sec5-style05-yawcmd: --init-from swapped from the plain (never-taught-to-turn) headingsfull-derived checkpoint to the turn-clone, plus goal.walk_phase_run_on_yaw=1 (harmless here since this arm has no turn_in_place_frac, but keeps the batch's phase-clock behavior uniform for any stress_mix zero-speed segment it does sample).

**gate**: Discovery continuation (2M, DR-0). Manual eval_yaw on the run's own pod: PASS/PARTIAL/FAIL-washout per the tip50/tip90-turnclone siblings' bar, judged against this arm's own frac=0 command mix (mostly arc-turns-while-translating, not pure tip).

**verdict**: Launch-config bug, zero training happened (wandb _runtime=0, no logged steps, empty summary/history). Root cause (confirmed via wandb output.log): the respec chain launched this arm from cw-amp-m2-bcinit-sec5-style05-yawcmd (which itself uses --obs-pad-transplant 1, since its OWN parent chain widened obs 73->74 for the yaw-cmd dim) but only overrode --init-from to the turn-clone checkpoint (ppo_goal_cw_bcgait_turnclone_fullprof_phase1.zip, which already has the wider 75-dim obs baked in) without also zeroing --obs-pad-transplant like its tip50/tip90 siblings correctly did (--obs-pad-transplant 0). Result: obs-pad-transplant asked to widen by 1 dim, found the checkpoint and env already agree (75==75), and train_ppo_mjx hard-failed immediately: "--obs-pad-transplant 1 but obs widened by 0 (75 -> 75); check cfg-sets". Not a research result -- no eval_yaw/DR-0 read is possible or meaningful for this entry. Already caught and corrected by the same session that produced this launch (before this triage cycle started): relaunched as cw-amp-m2-turnclone-yawcmd0-r2 with --obs-pad-transplant 0, currently training on hexapod-mjx-train-0 (another cycle's line now -- not touched here). No further action needed on this exact ledger entry; the zero-dose joint-read question stays open pending -r2's finish.

**failed_reason**: run never appeared as 'running' in W&B within 240s

