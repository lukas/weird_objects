# cw-gait-slowfirst1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-12T03:57:23+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-gait-dragstance1

**wandb_id**: u3z8h6bp

**hardware_ready**: False

**hypothesis**: Teach a from-scratch (no imitation, no BC anchor) walking network to lift its feet by commanding a much slower target speed, so a small honest step can satisfy the command without the large continuous slide the paddle needs to hit the champion's own 0.05-0.06 m/s band. cw-gait-dragstance1 (structural per-stance drag charge alone) and cw-gait-rsi1 (same charge + mid-stride RSI spawns) both collapsed into the identical near-still/marching-in-place habit -- env/walk_loadslip_factor floored by step 49 both times, drag charge never resolved. This is nobc's remaining GAIT.md P3 lever 5 (slow-speed-first), the only one launchable without new curriculum-scheduler code (levers 2/3 need an in-run coefficient ramp that doesn't exist yet). ONE variable vs cw-gait-dragstance1: goal.walk_speed_min_m_s/max_m_s lowered from 0.05/0.06 to 0.02/0.025 (60% below the champion's own band), same audit-derived drag charge (k=8000/m, 6mm allowance, 0.25mm floor), no RSI. Prediction-if-true: det gait_valid shows real six-foot cycling with slip/m near or below the paddle band (1.1-1.5) and prog_ratio meaningfully above 0 (real, if slow, travel). Prediction-if-false: the policy still floors walk_loadslip_factor and marches in place / freezes regardless of how slow the target is -- slow-speed-first is refuted too, and every nobc-legal from-scratch gait lever (2,4,5) is now closed; the honest next step is either the physics-easing lever (3, needs a training-time schedule -- CODE, spec first) or accepting the BC-anchor line as the only route to clean gait, same as hw.

**gate**: PASS if det gait_valid shows real six-foot cycling (not frozen/parked) in ANY episode with slip/m below the paddle band (1.1-1.5) AND prog_ratio clearly above 0 -- slow-speed-first helps discovery, worth hardening. FAIL if det+sto reproduce the identical near-zero-travel/floored-loadslip fingerprint from dragstance1 and rsi1 regardless of target speed -- lever 5 is refuted; move to lever 3 (physics easing, spec a training-time cfg schedule first) or close the from-scratch gait line for now.

**verdict**: FAIL -- reproduces the identical near-zero-travel/marching-in-place fingerprint as dragstance1 and rsi1, even at a target speed 60% below the champion band. det: gait_valid 6/6 but fwd travel 0.00m every episode, prog_ratio ~0.00, slip/m 7.31 (vs the paddle band 1.1-1.5) -- video confirms legs shuffle in place, zero net displacement. sto is worse (prog -0.07 median, slip/m 17.8, i.e. sliding, not stepping). Per pre-registration this is the FAIL branch: lever 5 (slow-speed-first) is refuted. All three nobc-legal from-scratch gait levers launchable without new code (2 drag-charge, 4 RSI-for-walk, 5 slow-speed) are now CLOSED on the identical mechanism (env/walk_loadslip_factor floors early and never resolves regardless of pricing/state-injection/target-speed). Next lever is CODE (physics-easing training-time schedule, spec first) or accept BC-anchor as the only route to clean gait (same as hw, but out of scope for nobc by charter).

