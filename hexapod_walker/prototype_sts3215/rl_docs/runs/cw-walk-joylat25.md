# cw-walk-joylat25

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T17:47:10+00:00

**pod**: hexapod-mjx-train-8

**steps**: 18000000

**parent**: cw-walk-joyjit-dr05-c1

**wandb_id**: r6puxbh2

**hardware_ready**: no

**hypothesis**: Compose the just-validated latency axis onto the BEST DRIVING CANDIDATE: latjit25 PASSed (latency 0.5-2.5x trainable by exposure on the walk champion, no forgetting), and real STS3215 bus latency jitters most during rapid command changes - exactly what joyjit-dr05-c1 (abrupt-flip + DR0.5 driving candidate) will face under a joystick. One variable off joyjit-dr05-c1: add dr.latency_scale=0.5,2.5 absolute override on top of its DR0.5. If-true: JOYSTICK GATE @DR0.2 still zero falls AND own-cfg (DR0.5 + latency override) gv 12/12, prog med >=0.85 - latency-hardened driving candidate supersedes joyjit-dr05-c1. If-false: wide latency + abrupt flips destabilize (delay hurts most mid-flip) - latency on driving lines needs estimator machinery (contact-aux/DreamWaQ rung), exposure only works for steady gaits. Strongest alternative: policy passes by slowing/parking through flips - check eval_drive per-scenario dist (left/right >=0.15m) and flip-stress trk_err vs parent (0.025-0.056).

**gate**: JOYSTICK GATE: eval_drive --dr-scale 0.2 own cfg - ZERO in-envelope falls, left/right dist >=0.15m; own-cfg (DR0.5 + dr.latency_scale=0.5,2.5) det+sto 6/6 @15s: gait_valid 12/12, 0 term, prog med >=0.85; DR0 retention det 6/6 gv; frames watched det

**verdict**: PASS. All 3 gate parts: JOYSTICK GATE @DR0.2 own cfg 0 in-envelope falls (left 0.197/right 0.275 m >=0.15; flip-stress trk_err 0.029-0.035 vs parent joyjit-dr05-c1 0.025-0.056). Own-cfg DR0.5+latency 0.5-2.5x det+sto: gv 12/12, 0 term, prog med 0.93/0.94 (gate >=0.85), slip/m 1.48/1.51 (steering-DR band). DR0 retention gv 12/12, prog 0.95/0.94 = parent (0.97/0.95). Frames det: level, six legs cycling, no flag leg; paddle foot-slide persists. Latency jitter and abrupt-flip driving COMPOSE - joylat25 supersedes joyjit-dr05-c1 as strongest driving candidate. Not hardware-ready (contact-pricing slip root).

