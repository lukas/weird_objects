# cw-nobc-slipwalk1-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-21T13:59:28+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-nobc-slipwalk1

**wandb_id**: mo2qew0u

**hardware_ready**: False

**hypothesis**: Mechanical retry of cw-nobc-slipwalk1, which never trained a single step (all 24 workers SIGBUS'd at boot: obs.history_frames=16 at n-envs=4096 overflows the 64M default /dev/shm on hexapod-mjx-train-0 — documented gotcha COMMANDS.md #13c). Same spec, same operator hypothesis (order 20260821T133626Z): teach a from-scratch robot to walk forward with no BC anchor and no speed target, earning income for any commanded-direction progress while paying hard for loaded-foot slip and for parking/marching in place. The only change is placement on a pod that already carries the 4Gi dshm fix, so the run actually boots.

**gate**: Deterministic DR-0 harness, walk mode, 6 episodes x 15 s, run's own cfg. PASS requires ALL: (a) zero falls/terminations, 6/6 episodes survive; (b) real distance: median along_dist_m >= 0.15 m/episode (scripted tripod at this command measures 0.22 m) - the near-zero fingerprint of dragstance1/rsi1/slowfirst1/sched1/ease1 is a FAIL; (c) direction: median walk direction error <= 30 deg, no wrong-way episode; (d) legitimate stepping: gait_valid >= 4/6 AND video shows all six legs cycling contact/swing (flag-leg, stilt, tripod park = automatic STOP); (e) low slip absolute AND normalized: median slip_per_m <= 3.0 AND median slip_m_total <= 1.0 m. Harness walk 'success' (in-band speed) is NOT part of this gate - this arm has no speed target. KILL EARLY at the 1M periodic eval if travel is still ~0 (freeze/march) or slip/m > 6 (skating). PASS stages rung 2 (second direction, then commanded direction changes); FAIL verdicts honestly and stops the sub-line - no re-run with more steps.

**verdict**: FAIL - freeze/skate fingerprint, not walking. Det: along_dist_m 0.001 m (need >=0.15), direction err 72.6 deg (need <=30), gait_valid 0/6 with 4/6 legs sacrificed, slip/m 6.75 (cap 3.0); sto matches (dir err ~80-88 deg, slip/m 12.6-15.8). Video/contact-sheet shows the robot locked in a static splayed stance for the whole 15s clip -- two legs near-permanently planted (duty 0.98/0.91), four legs barely loaded (duty 0.04-0.07) and thrashing without net travel: the exact freeze+skate fingerprint the gate pre-registered as an automatic STOP. Reward also fell every quarter of training (-70 -> -461 -> -958 -> -1241), so this is not a reward-up/behavior-bad cheat -- the anti-slip/idle pricing (k_loadslip_excess=6, k_walk_idle_charge=20) never found honest stepping. Per the pre-registered plan: verdict honest, stop the sub-line, no re-run with more steps.

