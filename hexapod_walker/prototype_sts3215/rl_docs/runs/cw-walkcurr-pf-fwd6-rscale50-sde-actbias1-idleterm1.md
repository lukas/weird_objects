# cw-walkcurr-pf-fwd6-rscale50-sde-actbias1-idleterm1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-24T03:10:24+00:00

**pod**: hexapod-mjx-train-5

**steps**: 2000000

**parent**: cw-walkcurr-pf-fwd6-rscale50-sde

**wandb_id**: ugm8iobn

**hypothesis**: Plain English: covers the named failure branch of the sde-actbias1 sibling -- if the recentered stable stance ABSORBS the gSDE excursions back into a clean park-stand (exactly what actbias1 alone converged to), the qvel idle-terminate mechanism evicts that absorbing state so the only long-episode income left is actual walking, while the sde noise supplies the directional escapes idleterm's raw-a=0 lineage never had. Full three-ingredient stack (stable zero-point + park eviction + correlated exploration), each ingredient individually built, bank-proven and behavior-verified; fresh 2M discovery. Prediction-if-true: episodes stop running full length from a park (idle-terminate fires), freeprog leaves the dead band and det gate shows stepping where sde-actbias1 alone parks. Prediction-if-false (evicted park just cycles into re-park or falls): parking is priced AND evicted AND escapable yet still wins -- strong evidence the discovery gap is value-initialization, feeding the BC-kickstart operator question (q_20260824T0233Z) with the completed lever inventory.

**gate**: Same rung-1 gate as every fwd6 arm: C-env det fixed-forward panel -- zero tilt terms, cmd_prog_frac >= 0.35, direction_err <= 30 deg, slip/m <= 3.0, six legs cycling on >=4/6 episodes. Mechanism-health: idle-terminate fire rate > 0 if parking recurs; tilt-term rate vs sde parent; freeprog vs dead band.

**verdict**: The full three-ingredient stack (recentered action zero-point + qvel park-eviction + gSDE correlated noise) fails by a NEW dodge: the policy fidgets in place fast enough that idle-terminate never fires, and never converts sde excursions into forward walking. Evidence: DR-0 gate det 0/6 gait_valid, fwd 0.02m/25s, slip/m 11.2, sac [0,3], ZERO terminations — mean |qvel| stays above the 2deg/s idle floor by jittering, the exact 'fake fidget' cheat the idleterm1 launch entry pre-flagged; sto 0/6, prog med -0.06 (slightly BACKWARD — the sde parent's +0.32 forward excursions do NOT survive this stack), 6/6 tilt terms, video shows legs splaying sideways into a topple. W&B: training is fall-dominated (tilt_pitch 554 + tilt_roll 647 per window, ep_len_mean ~90 steps, truncations ->1), freeprog -0.19->-0.16 (below even the [-0.10,-0.05] dead band), NO terminations/walk_idle_terminate key logged = idle-terminate fire rate ~0 all run, reward flat 15.9/11.4/10.7/11.0 — aligned FAIL per 08-21, the pre-registered prediction-if-false ('evicted park just cycles into re-park or falls'). Why: eviction + correlated noise removes the quiet park option but fall/termination economics still punish walking attempts, so the optimum shifts to in-place fidget just above the qvel floor. Idle-termination is now closed in all three tested configurations (rscale50 base, actbias1 base, sde+actbias1 base); raising the qvel floor would start misclassifying real walking and is not funded. Also: the watcher SUSPECT checkup (global_step stalled at 2007040) was a false alarm — the run had simply completed its 2M budget (wandb state=finished, 2064384 steps). Next: sde-actbias1 (no idleterm) and sde-s2 seed-replicate reads (other cycles) decide whether the sde forward-excursion finding is real; fork per the pinned rule.

