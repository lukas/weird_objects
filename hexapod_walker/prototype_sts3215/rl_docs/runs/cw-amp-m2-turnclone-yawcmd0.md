# cw-amp-m2-turnclone-yawcmd0

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-23T00:03:20+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-amp-m2-bcinit-sec5-style05-yawcmd

**hypothesis**: Plain English: the zero-dose control of the turnclone batch -- no dedicated turn-in-place episodes, just the original yawcmd command mix (arc turns while translating + occasional stress_mix zero-speed segments). Does a BC init that was actually taught to turn (see tip50/tip90-turnclone siblings) do better than the original yawcmd run (which parked on turn-in-place, tip err 0.2995/0.3008 == |wz_ref|, while carrying a command-invariant left drift during translation) even WITHOUT dedicated tip exposure? Single lever vs cw-amp-m2-bcinit-sec5-style05-yawcmd: --init-from swapped from the plain (never-taught-to-turn) headingsfull-derived checkpoint to the turn-clone, plus goal.walk_phase_run_on_yaw=1 (harmless here since this arm has no turn_in_place_frac, but keeps the batch's phase-clock behavior uniform for any stress_mix zero-speed segment it does sample).

**gate**: Discovery continuation (2M, DR-0). Manual eval_yaw on the run's own pod: PASS/PARTIAL/FAIL-washout per the tip50/tip90-turnclone siblings' bar, judged against this arm's own frac=0 command mix (mostly arc-turns-while-translating, not pure tip).

