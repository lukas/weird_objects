# cw-amp-m2-turnclone-yawcmd0-r2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-23T00:08:42+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m2-bcinit-sec5-style05-yawcmd

**wandb_id**: e6ksf9j2

**hypothesis**: Retry of cw-amp-m2-turnclone-yawcmd0 (REFUSED-at-runtime: crashed at step 0, '--obs-pad-transplant 1 but obs widened by 0 (75 -> 75)' -- the turn-clone checkpoint already carries the wz_ref obs column natively (built with goal.walk_yaw_cmd=1 from the start), so unlike the original yawcmd run's own 73->74 pad-transplant off the yaw-blind headingsfull checkpoint, THIS init needs zero padding). Single lever now correctly isolated: --obs-pad-transplant 0 (no-op plumbing fix) + --init-from swapped to the turn-clone. Same plain English question as the crashed attempt: does a BC init that was actually taught to turn do better than the original yawcmd run (parked, tip err 0.2995/0.3008 == |wz_ref|, command-invariant left drift while translating) even WITHOUT dedicated tip exposure (frac=0, arc-turns-while-translating + occasional stress_mix zero-speed segments only)?

**gate**: Discovery continuation (2M, DR-0). Manual eval_yaw on the run's own pod: PASS/PARTIAL/FAIL-washout per the tip50/tip90-turnclone siblings' bar, judged against this arm's own frac=0 command mix.

**verdict**: PASS -- the zero-dose control confirms dedicated turn-in-place exposure was NOT necessary once the init is turn-taught. Retry of the crashed cw-amp-m2-turnclone-yawcmd0 (obs-pad-transplant fixed 1->0). Manual eval_yaw (own cfg, no turn_in_place_frac -- pure yawcmd command mix, arc-turns-while-translating + occasional stress_mix zero-speed segments): tip-left err 0.1525, tip-right err 0.1614 (turn med 0.1548), hold |wz| 0.0042, 0 falls -- both directions clear the <=0.20 bar with correct sign, matching the tip50/90-turnclone siblings despite having ZERO dedicated tip episodes during RL. DR-0 gate: ALL 6/6 det episodes clean translation (no tip contamination at frac=0) -- gait_valid 6/6, zero sacrificed legs, slip/m 1.90-2.77 (better than the original yawcmd parent's ~2.8), progress_ratio 0.80-1.13 (strong, some episodes ahead of nominal pace), forward_dist up to 1.07m/15s. This is the cleanest read of the three turnclone arms: real command-signed rotation AND excellent translation simultaneously, with the smallest single-lever delta from a FAILed control (cw-amp-m2-bcinit-sec5-style05-yawcmd, tip err 0.2995/0.3008, dir-drifting while translating) -- only --init-from + --obs-pad-transplant=0 differ. Confirms the BC-turn-clone fix generalizes beyond the tip-exposure curriculum: the turn-taught prior alone, exercised only through ordinary arc-turn/stop commands, already produces real turning.

