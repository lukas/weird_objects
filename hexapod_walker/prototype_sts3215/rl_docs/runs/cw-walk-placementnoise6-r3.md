# cw-walk-placementnoise6-r3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T23:09:51+00:00

**pod**: hexapod-mjx-train-5

**steps**: 20000000

**parent**: cw-walk-placementnoise6r

**wandb_id**: 0yrg0d82

**hardware_ready**: False

**hypothesis**: RETRY #3 of the hand-placement-slop axis (dr.placement_noise_deg=6): first two attempts both died to the same host-wide EOFError launch-collision storm (0 steps each, no science result). Same spec unchanged; queued to backlog for the self-repairing drain.

**gate**: Own-cfg harness at DR0 + placement_noise_deg=6, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2m; DR0 nominal retention det 6/6 gv, slip/m <=1.24; parent baseline under same spread compared at triage; frames watched det

**verdict**: PASS (retry #3 -- first two died at 0 steps to the launch-collision storm; this is the first real science on the hand-placement-slop axis, dr.placement_noise_deg=6deg). Own-cfg det+sto 12/12 gait_valid, 0 term, det median fwd 1.23m (>=1.2 gate), slip med 1.10-1.36; 3/6 det draws at the noisier end degrade to prog 0.45-0.88/fwd 0.51-1.15m (heavy tail, no falls/flag-leg -- same shape as other exposure axes). DR0 nominal-placement retention is CLEAN: det med fwd 1.50m, slip 1.10, gv 6/6, 0 term, all six det episodes ok -- no erosion from the noise exposure. Det frames watched on both passes: level six-leg cycling, no flag leg, same known paddle/foot-slide gait as the champion -- not a new defect, not hardware-ready.

