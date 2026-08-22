# cw-amp-m2-bcinit-sec5-noamp

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-22T20:07:13+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-amp-m2-sec5-noamp

**wandb_id**: i97ixmxj

**hypothesis**: Plain English: the task-only twin of cw-amp-m2-bcinit-sec5-style05 — same BC-clone init (a policy that already walks) and same sec5 minimal reward, with ZERO AMP flags — isolating what the AMP style channel adds (or costs) on a genuinely locomoting actor, which no prior arm could measure because every from-scratch policy was a statue. Same config discipline as the twin: sec5 minimal reward verbatim (bank PASS), clone-compatible obs/env (phase obs, body_vel=2, fast servo, stress_mix fixed 0.08, no yaw-cmd obs, no asym-critic), warm-log-std -2.0, seed 7, DR-0, 2M. Prediction-if-true (walking survives): gives the noamp baseline the style arm is judged against. Prediction-if-false (collapse to crouch statue): together with a twin collapse, convicts the sec5 reward shape itself — task restructuring (height/upright/termination pricing) becomes the next M2 lever; if ONLY this arm collapses while the style twin walks, style is protective and AMP earns its first positive result.

**gate**: Discovery (2M, judged on det video + DR-0 gate harness, NOT the joystick DONE gate; read JOINTLY with bcinit-sec5-style05 twin). INFORMATIVE-PASS = sustained cyclic six-leg walking, det fwd >=0.10m/15s, no crouch collapse (height_err stable). FAIL-collapse = statue/crouch basin reappears despite the walking init — sec5 reward destroys locomotion, task restructuring is the next lever. Joint read vs twin: style-vs-noamp delta on gait_valid/slip/naturalness is the batch's primary measurement.

**verdict**: Task-only twin of style05 -- also PASSES, confirming BC-init alone (no AMP style) already rescues the sec5 reward from the statue basin. DR-0 gate det gait_valid 6/6, sto 6/6, zero sacrificed legs, real net travel (det fwd med 0.64m/15s, prog med 1.09; sto fwd med 0.24m, prog med 0.52) -- frame-strip of walk_det_4 shows clean six-leg alternating-tripod stepping with visible forward displacement, same quality as the style05 twin. env/height_err_mm 18-31mm the whole run, no crouch. Slightly WORSE than style05 on every axis (det slip 2.13 vs 1.88, sto slip 5.38 vs 4.71; prog both modes lower) -- a modest, consistent style benefit on top of a walking init, first time this comparison has been measurable (every prior style-vs-noamp arm was statue-vs-statue). Root cause: same as style05 -- prior 0/4 sec5-grid FAILs were an init/exploration problem (PPO from random can't find the walking basin under this minimal reward), not a reward-shape defect; BC-init starts inside the basin and the reward preserves it. Config forward-only / clone-compatible obs (no heading diversity yet) -- not the full M2 milestone. hardware-ready: no (2M sim discovery, DR-0, forward-only). Next: heading-diversity continuation launched this cycle (bcinit-sec5-noamp-headings20) paired with the style05 twin.

