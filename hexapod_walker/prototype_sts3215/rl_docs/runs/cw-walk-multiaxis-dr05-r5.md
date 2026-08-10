# cw-walk-multiaxis-dr05-r5

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T01:57:34+00:00

**pod**: hexapod-mjx-train-0

**steps**: 14000000

**parent**: cw-walk-multiaxis-dr05-r4

**wandb_id**: roy8wsxi

**hardware_ready**: False

**hypothesis**: Retry (4th consecutive infra loss, r1-r4 all 0 steps). Same spec unchanged.

**gate**: Own-cfg (4-axis stack + DR0.5) det+sto 6/6: gait_valid 6/6, 0 term, prog med>=0.75; DR0 nominal retention det 6/6 gv, slip/m<=1.24, prog>=0.9; frames watched det.

**verdict**: FAIL -- duplicates/confirms the already-FAILED cw-walk-multiaxis-dr05 (this run, r5, is the same 4-axis-stack-at-DR0.5 recipe, retried after 4 consecutive infra deaths; same seed=0, warm-started along the same lineage). Own-cfg (DR0.5+mass/latency/deadband/comshift stack) exposure panel is clean: gv 12/12, 0 term, det med fwd 1.29m (>=1.1 gate), no craters at all. But DR0 nominal retention misses hard: det med slip/m 1.58 (vs <=1.24 cap, ~27% over) driven by 2/6 fixed-draw crater episodes (fwd 0.63-0.70m, slip 2.8-3.3); sto stays clean (1.14<=1.24). Frames on the crater episodes show the same mechanically-clean march-in-place pattern seen across the walk lineage all night (six legs cycling, level body, no falls, no flag leg) -- not a visible new defect, but the retention cap is still missed on the letter, same as the original multiaxis-dr05's own verdict ("axis-stacking ceiling for this step budget is 4 at DR0; adding generic DR0.5 on top does not hold"). No requeue -- this closes the question again at this budget; more steps or fewer simultaneous axes would be the next lever, not pursued.

