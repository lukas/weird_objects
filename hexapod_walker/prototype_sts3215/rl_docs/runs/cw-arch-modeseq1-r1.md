# cw-arch-modeseq1-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-14T13:14:25+00:00

**pod**: hexapod-mjx-train-0

**steps**: 10000000

**parent**: cw-arch-modeseq1

**wandb_id**: ysdjp7yr

**hardware_ready**: False

**hypothesis**: Teach one model to stand up, walk, sit down and stand up again on command, in a single continuous run - this arm tests whether training directly on chained mode sequences (goal.mode_seq, 75% sequence / 25% single-mode diet) gets the RL policy through the mode switches that specialists only survive when an external script re-anchors them. Identical spec to cw-arch-modeseq1 (TRANSITIONS_DIRECTIVE Arm 2, warm from the dagger1 BC init per the pre-registered order), relaunched after its infra death: the sharded MJX path now mints the canonical segment frames (pod-verified bit-identical to the in-process reference, snapshot dab1165).

**gate**: Pre-registered (TRANSITIONS_DIRECTIVE Arm 2): sequence eval det+sto DR0 + own-DR0.5 zero falls >=11/12 det AND per-segment criteria >=9/12 AND single-mode retention at dual1 levels (walk gait_valid >=5/6 prog >=0.80, hold >=4/6, lower >=4/6, rise n=12 method >= its own init 3/12) AND switch-window max tilt reported (baseline, no bar in v1).

**verdict**: FAIL per pre-registered gate (sequence det DR0 zero-fall 2/12, bar >=11/12; sto 3/12) after canary auto-stop at 4.56M/10M. DIG-IN OBSERVATIONS: (1) sequence eval (eval_modeseq --single, n=12 det+sto DR0): det 2/12 zero-fall vs transdagger2 distill 12/12 and specialist baseline 11/12; falls are ALL inside rise segments -- every flat cold-start rise fell (0/3) and 5/7 post-lower rises fell; switch windows clean (tilt med 1.7 / max 8.2 deg det), walk segments 9/9 gait_valid prog_med 1.014, lower 7/7. (2) matched dual2-style rise recheck (n=12/seed=1 DR0): det 5/12 = crouch 5/5, bridge 0/4, flat 0/3 -- the EXACT profile of dual2's stopped 3.18M checkpoint (5/12 crouch-only) vs the shared dagger1-init control's 3/12 all-non-crouch (bridge 2/4, flat 1/3); flat/bridge episodes now TERMINATE (fall) rather than stall. (3) single-mode retention letter-PASS: walk gait_valid 6/6 det prog 1.04 vel_err 0.036, hold 6/6, lower 6/6 worst_clear 0mm, roll_tails 0.2-1.5deg, drag 114-361mm det. INTERPRETATION: second independent warm-RL from the dagger1 init reproducing the identical crouch-attractor swap -- and a 75% mode_seq sequence diet did NOT protect the demo rise, so the erosion is not a data-distribution artifact of single-mode training; PPO+stance-anchors spend the rare hard-start demo competence regardless of diet. None of the three pre-registered Arm-2 FAIL branches matches (no walk freeze; no switch falls; single-mode rise degraded too, so sequence-RSI does not apply). VERDICT: warm-RL-from-dagger1 is CLOSED per the two-miss rule (dual2 + this). Next per the directive's pre-named fork: bridge/flat-heavy transitions demo mix on the transdagger2 recipe (transdagger3, CPU distill; --cfg-set passthrough landed this cycle) and the operator's option (b) rise-only variant distill stays open. Evidence: logs/ckpt_eval/modeseq1_r1_seq_{det,sto}.json, cw_arch_modeseq1_r1_{gate,owncfg,rise12}, gru_dual_bc_dagger1_rise12_pod (control). NOTE: the watcher's pre-staged gate eval inherited goal.mode_seq=0.75 (would mislabel 75pct of single-mode episodes as sequences) and was killed; the recorded gate/owncfg artifacts are clean 15s single-mode runs matching the dual1/dual2 instrument.

