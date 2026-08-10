# cw-walk-lowgait-dr035-comshift-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-10T04:52:04+00:00

**pod**: hexapod-mjx-train-6

**steps**: 18000000

**parent**: cw-walk-lowgait-dr035-comshift-r1

**wandb_id**: p0u272fe

**hardware_ready**: False

**hypothesis**: Seed-1 twin of lowgait-dr035-comshift-r1 (ruling-7 seed robustness): the crouch-50mm+DR0.35 x CoM-offset compose PASS is not seed-0 luck. If-true: own-cfg + DR0 retention medians in r1's band, no new failure class. If-false: seed-1 shows flag-leg/falls or median erosion — compose is seed-fragile.

**gate**: Own-cfg (DR0.35+dr.com_offset_m=0.03) det+sto 6/6 @15s: gait_valid 12/12, 0 term, mean end-height err<=10mm, slip/m med<=1.6; DR0 no-offset retention det 6/6 gv, mean height err<=8mm, slip/m med<=1.15; frames watched det

**verdict**: PASS -- seed-1 twin of lowgait-dr035-comshift-r1 (seed0) reproduces cleanly, confirming the crouch(-50mm,DR0.35) x off-center-CoM(0.03m) compose is a recipe, not seed luck. Own-cfg (DR0.35+comshift) det+sto gv 12/12, 0 term, height err mean 4.1/2.4mm (<=10mm gate), slip/m med 1.11 det/1.30 sto (<=1.6 gate) -- matches r1 seed0 own band (height err 4.8/4.1mm, slip 1.02/1.36) closely, slightly better on height. TRUE DR0 no-offset retention (re-run after catching my own methodology slip -- evalcmds default command still carries the training com_offset override, so a genuine retention pass needs it dropped by hand): det gv 6/6, 0 term, height err mean 2.4mm (<=8mm gate), slip/m med 1.01 (<=1.15 gate, clean, NO march-in-place crater this time); sto gv 6/6, height err mean 2.3mm, slip/m med 1.28, one mild dip (sto/4 prog 0.43/slip 3.15, not catastrophic). Separately, WITH the comshift override still active at dr-scale=0 (i.e. own-cfg minus only the generic background DR), det/4 DOES crater (prog 0.17/slip 13.0) -- matching r1 seed0s own with-offset crater at the same index almost exactly. Read together this is fully consistent with c79s original root-cause finding on r1 (a fresh-draw panel across many seeds showed comshift training just relocates which rare draw lands on the lineages known paddling-attractor knife-edge, not that comshift creates a new failure) -- NOT a contradiction, just a more precise same-seed replication of that mechanism. SKILLS row corrected accordingly.

