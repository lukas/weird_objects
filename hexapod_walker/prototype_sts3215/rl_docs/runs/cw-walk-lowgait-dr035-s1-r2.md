# cw-walk-lowgait-dr035-s1-r2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T23:08:37+00:00

**pod**: hexapod-mjx-train-6

**steps**: 20000000

**parent**: cw-walk-lowgait-dr035-s1

**wandb_id**: xietue4o

**hardware_ready**: False

**hypothesis**: Second infra retry of cw-walk-lowgait-dr035-s1 (both prior attempts lost launch-collision races amid concurrent-cycle drain storms, worker EOFError at init, 0 steps each -- no science result either time; COMMANDS.md gotcha 13b). Same spec unchanged: -50mm crouch, dr-scale 0.35, seed 1.

**gate**: Own-cfg DR0.35 15s at -50mm 6+6: gv 12/12, 0 term, mean end-height err <=10mm, slip/m med <=1.6; DR0 det retention gv 6/6, height err <=8mm, slip/m <=1.15; frames watched det

**verdict**: PASS. Seed-1 twin of lowgait-dr035 confirms the -50mm crouch survives DR0.35 independent of seed. Own-cfg DR0.35 det+sto 6/6 gv, 0 term, height err mean 3.1mm det/1.4mm sto (gate<=10mm), slip/m med 1.08 det/1.39 sto (gate<=1.6), fwd med 0.72m det/0.66m sto @15s. DR0 nominal retention clean: gv 6/6, 0 term, height err mean 3.3mm (gate<=8mm), slip/m med 1.01 (gate<=1.15). Frames (det, own-DR): low stable crouch stance, all six legs cycling, no flag leg. One-two fixed-draw churn-tail episodes in sto passes (low prog, high slip, no fall) matching the known canary-class pattern. Seed-confirmed: crouch-DR0.35 is not a seed-0 fluke.

