# cw-walk-groundtilt8-comshift-r2-rr1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T03:55:27+00:00

**pod**: hexapod-mjx-train-9

**steps**: 20000000

**parent**: cw-walk-groundtilt8-comshift

**wandb_id**: 4emtqsza

**hardware_ready**: False

**hypothesis**: Retry (r2) of cw-walk-groundtilt8-comshift: r1 was lost to a drain-process interrupt (infra, not science, 0 steps). Same hypothesis: marginal 8deg tilt exposure (groundtilt8-r3 PASS-with-caveat) x off-center CoM payload (0.03m comshift envelope), untried pairing.

**gate**: Own-cfg (tilt u(0,8deg) + dr.com_offset_m=0.03) det+sto 6/6 @30s: gait_valid 12/12, 0 term, det med fwd>=1.1m, crater fraction<=3/6, 0 falls; DR0 flat-no-offset retention det 6/6 gv, slip/m<=1.24; frames watched det

**verdict**: PASS: marginal-tilt(0-8deg) x off-center CoM payload compose holds cleanly. Own-cfg det med fwd 1.29m (>=1.1 gate), sto med fwd 1.37m; crater fraction 2/6 det (slip 3.5/4.7, fwd 0.71/0.43m) matching the groundtilt8 lineage's known shuffle tail (<=3/6 cap), 0 falls/flag-leg on any draw (video: six legs cycling even on craters). DR0 flat-no-offset retention det 6/6 gv, slip/m 1.13<=1.24 cap, prog 0.94. New pairing (comshift onto groundtilt8) is safe.

