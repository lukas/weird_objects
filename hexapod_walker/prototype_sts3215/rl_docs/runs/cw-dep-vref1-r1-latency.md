# cw-dep-vref1-r1-latency

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-10T07:32:12+00:00

**pod**: hexapod-mjx-train-3

**steps**: 20000000

**parent**: cw-dep-vref1-r1

**wandb_id**: qw367uro

**hardware_ready**: False

**hypothesis**: PROTECT THE NAMED HARDWARE CANDIDATE: vref1-r1 is the leading checkpoint for tonight's hardware attempt #2, but has never been exposed to bus/comms latency jitter (0.5-2.5x, validated on driving lines) -- directly relevant since the deployed contract runs over a real serial/WiFi link with variable delay. Per P0 rule 3, k_current=0. If-true: own-cfg det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band; latency composes free like it does on every driving lineage. If-false: contract-exact meas:=ref velocity obs is more latency-sensitive than legacy privileged-velocity obs (a stale honest-velocity reading is worse than a stale privileged one) -- flag before hardware.

**gate**: Own-cfg (DR0.35+latency0.5-2.5x) det+sto 6/6 @15s: gait_valid 6/6, 0 term, slip/m within vref1-r1's own band (~0.89-1.13 det, ~1.13-1.36 sto); DR0 no-latency retention clean; frames watched det

**verdict**: PASS -- bus/comms latency jitter (0.5-2.5x) composes free onto vref1-r1; the honest (meas:=ref) velocity obs is NOT more latency-sensitive than the legacy privileged obs (if-false branch refuted). Own-cfg (DR0.35+latency) det+sto gv 6/6, 0 term, det slip/m med 1.13 sto med 1.10 -- inside/at-edge of vref1-r1's own band (0.89-1.13/1.13-1.36). DR0 no-latency retention gv 6/6, 0 term, det slip/m med 0.91 sto med 0.97 (same known lineage fixed-draw sto/4 crater, slip 4.72 vs parent's 5.97, same draw). Own-cfg det/5+sto/0+sto/1 crater cluster matches the gyronoise/imumount siblings episode-for-episode -- a shared DR0.35+seed0 lineage draw, not a latency-specific regression. Frames clean six-leg creep, no flag leg/drag/fall. Training finished clean (reward quarters 581/668/662/647).

