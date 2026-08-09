# cw-walk-multiaxis1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T19:31:13+00:00

**pod**: hexapod-mjx-train-9

**steps**: 18000000

**parent**: cw-walk-longdist-r2

**wandb_id**: jbnh5uvy

**hardware_ready**: no

**hypothesis**: Compose rung (operator 08-09, fill idle slot): the four INDIVIDUALLY validated physics axes (payload 1.0-1.4x mass, latency 0.5-2.5x, deadband 1.0-3.0x, CoM +30mm) randomized TOGETHER at dr-scale 0 off the champion. If-true: axes compose without interference (multiaxis own-cfg gv 12/12, 0 term, det med fwd >=1.2m; DR0 nominal retention clean) - recipe becomes the robustness-champion base. If-false: interference collapses gait (terminations/prog craters) - compose needs staging (add one axis per rung) not all-at-once. Strongest alternative: policy passes by finding one conservative crouch gait that ignores the axes - height_err/duty/frames will show it.

**gate**: Own-cfg harness at dr-scale 0 + all four dr overrides, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2m; plus DR0 nominal retention det 6/6 gv; frames watched det

**verdict**: PASS (pre-registered gate met): 4-axis compose panel (payload 1.0-1.4x + latency 0.5-2.5x + deadband 1-3x + CoM +30mm) gv 12/12, 0 term, det med fwd 1.29m>=1.2; DR0 nominal retention CLEAN (gv 6/6, prog 1.02, slip/m 1.06 = champion band, no payload-dr05-style erosion). Caveat: 2/6 det heavy draws slow-shuffle at ~40% speed (prog 0.42-0.45, slip 3.3-3.7) — same tail as isolated payload; never parks, no flag leg, no falls. Alternative (one conservative gait ignoring axes) REFUTED: benign draws run champion speed 1.5m, only extreme draws slow. The four individually-validated axes compose without interference — recipe eligible as robustness-champion base (seed twin queued). hardware-ready: no (paddle lineage).

