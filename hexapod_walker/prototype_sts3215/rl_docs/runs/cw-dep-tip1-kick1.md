# cw-dep-tip1-kick1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-12T00:39:36+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-dep-tip1-takeoff25-r1

**hypothesis**: Teach the deployed walking policy to survive the roll JOLT every real walk shows in its first second — the takeoff transient (18 bench walks: all cross 5deg roll by 0.6-1.5s at 11-46deg/s, peak 13-27deg; falls a coin flip for BOTH champions, no A/B winner). The static-lean dose is a CLOSED lever (takeoff25-r1: child==parent, axis saturated — sim already recovers static 20-25deg starts); the gap is the roll RATE, i.e. DYNAMIC. ONE CHANGE vs tip1: dr.walk_kick_* (commit 7d34fc6, bank green) — walk episodes carry a transient one-side fold pulse on the physical servo command over the first ~second of gait (half-sine, net-zero terminal offset, same command-side wiring as tipped/rise-rock). dr.walk_kick_prob=0.5 absolute (half the episodes stay nominal for retention), deg 8-18 / 0.5-1.2s (authored from the bench rate band); tipped_start restored to tip1's exact effective regime (0.105 prob / 6-18deg — absolute overrides reproduce default*dr_scale). Prediction-if-true: the policy learns to damp the pulse (active leveling during gait start), injected-eval falls/peak-roll separate from the matched parent. Prediction-if-false: child==parent under injection (axis saturated like the static dose) — then the takeoff gap is NOT command-side and the remaining lever is contact/pinning. Strongest alternative: the pulse teaches a crouch-brace that erodes the nominal gait (retention gate catches it).

**gate**: PASS if injected eval (eval_checkpoint det, --cfg-set dr.walk_kick_prob=1.0 --cfg-set dr.walk_kick_deg=14,14, --baseline ppo_goal_cw_dep_tip1.zip same injection+seed) shows the child with ZERO tilt falls and gait_valid >=5/6 while the matched parent degrades (>=1 fall or gait_valid <=4/6 or clearly higher peak roll) AND nominal retention: det walk gait_valid >=5/6, speed in the 0.05-0.06 band, slip <=1.8, no park (tip1's own band). NULL if child==parent both clean under injection (axis saturated — closes the command-pulse family for takeoff, points at contact/pinning). FAIL-retention if nominal gait erodes -> one dose retry at prob 0.3 or deg 8,14; double-miss closes the axis. PASS -> hardening + Gate 0 export + bench A/B vs tip1 judged by fell/tail-roll per bench_report.

**refused_reason**: hexapod-mjx-train-0 already runs cw-dep-tip1-kick1 — GPU pods host exactly one run; pick a free GPU pod.

