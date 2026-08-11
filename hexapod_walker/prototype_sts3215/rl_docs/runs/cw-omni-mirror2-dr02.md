# cw-omni-mirror2-dr02

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: DONE

**created**: 2026-08-11T01:33:16+00:00

**pod**: hexapod-mjx-train-1

**steps**: 40000000

**parent**: cw-omni-mirror2

**wandb_id**: hbxejemv

**hardware_ready**: False

**hypothesis**: Matched DR twin of cw-omni-mirror2 (operator question, 08-10 eve: is domain randomization making the omni task too hard?). Identical spec — same income fix (walk_kernel_yaw_gate), same warm start from the mirror1 probe, same mirror coef 1.0 — with the ONE change dr-scale 0.5 -> 0.2. The omni recipe inherited DR 0.5 from the driving-champion line, but 0.2 is the from-scratch field standard (fresh1/uni-rfix precedent), and r1's monotonic std climb 0.39->1.69 is consistent with optimization strain, not just the pricing hole. If-true (dr02 learns turn tracking / keeps a healthy gait where mirror2 stalls or collapses): DR 0.5 is a real difficulty knob on this task — harden the eventual winner back up the DR ladder the usual way. If-false (both twins behave the same): DR exonerated, the pricing fix carries the verdict, and the omni line stays at DR 0.5. Either way the operator's question gets a controlled answer instead of an opinion.

**gate**: Same panel as cw-omni-mirror2, judged AS A PAIR: eval_yaw both signs + zero-cmd drift + joystick incl backward/lateral + gait_valid + video, each twin vs the dep1 baseline AND vs each other. Kill-early signature identical to mirror2 (det forward travel collapsing with frozen episodes out-earning walking; std past ~2x start).

**verdict**: STOP - known exploit (identical to parent cw-omni-mirror2, DR was not the cause). At DR 0.2 the gated 40M re-hardening still collapses into the same leg-sacrifice pattern in half of det episodes: gait_valid 3/6 det (sacrificed legs [0,2,4] and [1,3], fwd_dist 0.00-0.01m, return 473-485) vs 3/6 healthy walking (fwd 0.05-0.10m, return 646-889) - walking out-earns sacrifice just like mirror2 (646-889 vs 473-485 here; 526-922 vs 399-425 there), so the freeze-floor income fix still holds, but the base gait never stabilizes regardless of DR. sto: gait_valid 6/6 but slip_per_m 4-18, prog_ratio 0.09-0.49, 0/6 success both modes both configs. train/std 1.10 (dr02) vs 1.30 (mirror2) at 40M - milder optimization strain but same terminal pathology. PAIR VERDICT: DR-scale is EXONERATED as the driver of the leg-sacrifice collapse (if-false branch) - the gait pathology is a property of this reward/task stack, not a DR difficulty knob. Do not requeue a DR ladder on this line; next lever is the term-by-term WALK-kernel income re-probe already queued for the mirror2 pathology (RL_PLAN backlog item 1), not another DR twin.

