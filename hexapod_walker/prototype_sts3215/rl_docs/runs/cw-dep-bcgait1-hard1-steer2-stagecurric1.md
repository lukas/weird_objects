# cw-dep-bcgait1-hard1-steer2-stagecurric1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-18T18:35:04+00:00

**pod**: hexapod-mjx-train-4

**steps**: 2000000

**parent**: cw-dep-bcgait1-hard1-steer1c

**hypothesis**: Teach the tall walker to survive abrupt joystick direction changes by RAMPING difficulty instead of maxing it from step 0: the previous attempt (-steer1-hard20m1, 20M full stress-mix from tick zero) FAILED its behavioral gate (3/24 long direction-switch episodes tripped an over_current safety cutoff from a jammed joint, 1/24 sacrificed 3 legs in a near-total tangle/stall), confirming that runs own pre-registered prediction-if-false: full-mix exposure from step 0 is not enough. This canary uses the existing, already-tested sched.* in-run scheduler (built 08-17 for exactly this failure class, unused on this lineage until now) to ramp goal.walk_cmd_stage 0->2 over the first 60% of a 2M-step budget -- forward/back-only switches first (stage 0: flip_180+stop_go, heading forced to 0), then headings/circles/squares (stage 1), full family set incl. jitter only by the end (stage 2) -- instead of throwing every schedule family at the policy from tick 0. Same reward/eval stack, same bcgait1_hard1 warm-start as steer1c. Prediction-if-true: mechanism stays healthy (finite losses, no KL-rollback storm, tall gait retained on periodic eval/video) AND the SAME 24-episode direction-switch panel run at hard20m1 shows fewer/zero over_current or tangle events at this small budget -- evidence the staged approach is the right lever, worth a full hardening-scale continuation. Prediction-if-false: the staged ramp still produces jamming/tangle events at a similar rate, meaning the problem is not exposure SCHEDULE but something structural (e.g. the yaw-limit margin itself, or the switch being genuinely instantaneous/no-blend) -- next lever would be blend-in time on switches or a yaw-margin reward term, not more curriculum tuning.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. CANARY/DISCOVERY (2M): PASS requires (1) finite losses, no sustained KL-rollback storm, value loss not diverging; (2) periodic eval + final video keep the tall in-band height and six-leg cycling gait (no re-crouch, no permanent leg-sacrifice) -- mechanism health only, mature tangle-robustness NOT the final verdict at this budget. INFORMATIONAL (folded into the same-cycle verdict, not a separate gate): run the identical 24-episode (6 families x 2 seeds x det+sto, DR-0 + own-DR) direction-switch panel used to fail hard20m1 and report over_current-termination count + tangle(sacrificed-leg) count vs hard20m1s 3+1/24 -- fewer/zero = staged curriculum looks like the right lever (queue a matched ~20M hardening continuation with the SAME retention guard); same-or-worse = the schedule-vs-mechanism prediction-if-false fires, do not just re-run longer, redesign (blend-in time or yaw-margin pricing).

