# cw-dep-bcgait1-hard1-steer2-blend1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-18T21:13:50+00:00

**pod**: hexapod-mjx-train-4

**steps**: 2000000

**parent**: cw-dep-bcgait1-hard1-steer2-stagecurric1

**wandb_id**: 608m9qc1

**hypothesis**: Teach the tall walker to survive instant joystick direction reversals without a joint jamming against its limit by giving the COMMANDED reference itself a brief ease-in time on each switch during training, instead of asking the policy to master literal zero-blend flips through more exposure alone. Two hardening attempts at the identical zero-blend (goal.walk_cmd_blend_s_min/max=0.0, the hardest possible instant-flick bar) have now failed the same pre-registered clause: steer1-hard20m1 (full-mix from tick 0) scored 3/24 over_current + 1/24 severe 3-leg tangle; steer2-hard20m1-r1 (staged 0->2 ramp then 20M at full stage 2) scored 2/24 over_current (video confirms at least one is a genuine tip-over fall, not just a stall) + 0/24 severe tangle but a NEW elevated slip/m across nearly the whole panel (median ~2.2-3.9/m vs hard1's clean ~1.3-1.5/m and vs its own stagecurric1 canary's clean video) -- two misses on 'more exposure at zero blend cures the jam', per the two-miss rule the lever changes, not the step count. This is a 2M discovery/mechanism canary, warm-started from the stagecurric1 checkpoint (keeps the staged-ramp-acquired tall gait), that sets goal.walk_cmd_blend_s_min=0.4/max=1.2 for TRAINING (a real but still much-faster-than-legacy-default 1.0s ease, not the maximally-hard instant flick). Prediction-if-true: mechanism stays healthy AND a hand-run 24-episode direction-switch panel evaluated with blend FORCED BACK TO 0 at eval time (the original hardest bar, matching the gate this lineage has always been judged on) shows fewer over_current terminations and slip/m closer to hard1's band than both priors -- training-time ease generalizing to the harder instant-switch eval licenses a matched ~20M hardening continuation. Prediction-if-false: over_current/tangle/slip stays similar or worse even under zero eval-time blend, closing the blend-time lever and escalating to reward-side yaw-margin pricing as the next structural attempt instead.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. CANARY (mechanism health, 2M): finite losses, no kl_rollback storms, tall in-band height + six-leg cycling retained on periodic eval/video, no early behavioral collapse. INFORMATIONAL read (not a final verdict, does not touch today's download answer): a hand-run 24-episode direction-switch panel (stress_mix cfg, DR-0 + own-DR-0.35, det+sto, 120s episodes) run with --cfg-set goal.walk_cmd_blend_s_min=0 --cfg-set goal.walk_cmd_blend_s_max=0 REGARDLESS of this run's own training cfg (forces the original hardest instant-flick bar at eval time) -- report over_current count, sacrificed-leg count, and slip/m median against steer1-hard20m1 (3/24 over_current, 1/24 severe tangle, slip/m 1.3-2.4 on clean episodes) and steer2-hard20m1-r1 (2/24 over_current incl. >=1 video-confirmed fall, 0/24 severe tangle, slip/m median ~2.2-3.9). A clean win (fewer/zero over_current AND slip/m back near hard1's ~1.3-1.5 band) licenses a matched ~20M hardening continuation with the SAME --best-ckpt retention guard; a wash or worse CLOSES the blend-time lever on this clause and the next arm must be reward-side (yaw-margin pricing), not a third exposure/schedule resweep.

