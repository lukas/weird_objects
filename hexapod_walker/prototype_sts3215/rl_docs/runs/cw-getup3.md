# cw-getup3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: DONE

**created**: 2026-08-12T02:18:04+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-getup2-r1

**wandb_id**: j36tw442

**hardware_ready**: False

**hypothesis**: Teach the from-scratch unified get-up-and-walk policy to actually stand, this time with an explicit teaching signal instead of relying on a warm-start head start alone. cw-getup2-r1 showed the warm-started specialist's stand skill does NOT survive the getup task -- it decays back into the same static collapse cw-getup1 hit with no head start at all, so the barrier is that a soft prior is overwritten by ordinary training noise, not that the knowledge can't transfer. ONE variable vs cw-getup2-r1: turn on the new train.bc_anchor_getup lever (CODE landed this cycle, default off, cfg-gated, bit-exact when off) -- it pulls the policy's action toward a recorded stand-up demonstration at the joint-space-nearest point on every getup tick, the same kind of explicit pull that already fixed the rise task's identical warm-start-drift problem. Prediction-if-true: env/getup_S holds near or above cw-getup2-r1's peak (>0.15, ideally trending toward 0.3) instead of decaying, and the final video shows a sustained stand (not a collapse) from at least one floor-adjacent start. Prediction-if-false: getup_S still decays despite the explicit anchor -- meaning the barrier is not warm-start/anchor fragility but the getup reward/observation wiring itself (e.g. untangle/load credit outearning real standing), and the next lever is a getup-reward income audit, not more imitation supervision.

**gate**: Same MDP_PREFLIGHT GETUP bank (green) plus the new getup-mode BC-anchor tests in test_bc_anchor.py (green, this commit). PASS if by 2M steps env/getup_S is NOT monotonically declining and ends clearly above cw-getup2-r1's ending band (target >0.15, ideally approaching 0.3), AND video shows a sustained, credible stand (not a static collapse) from at least one floor-adjacent start with no flag-leg/park exploit. Either outcome is informative: holding/climbing confirms the anchor fixes warm-start drift on getup same as it did on rise; still-decaying closes that lever and points at the reward/obs wiring itself as the barrier.

**verdict**: PASS (informative) — the explicit BC-anchor-on-getup lever stops the warm-start decay: env/getup_S rose 0.09->0.17 over the full 2M steps (r1 warm-start-only fell 0.09->0.06), clearing the >0.15 gate target. Training video confirms a genuine, sustained rise-to-stand from a floor-adjacent (tangled/flat) start -- height climbs 2mm->110mm over ~3s then holds level and motionless for the rest of the episode, all six legs symmetric, zero flag-leg/park cheat (env/getup_f_flag 0.96). Not every start rises yet (one of four sampled episodes stays stuck low, holding but not climbing) -- the anchor helps but doesn't yet generalize to every start kind.

