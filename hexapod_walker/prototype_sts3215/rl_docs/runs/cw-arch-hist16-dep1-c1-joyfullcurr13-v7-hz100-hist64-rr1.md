# cw-arch-hist16-dep1-c1-joyfullcurr13-v7-hz100-hist64-rr1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-24T18:30:14+00:00

**pod**: hexapod-mjx-train-2

**steps**: 40000000

**parent**: cw-arch-hist16-dep1-c1-joyfullcurr13-v7-hz100

**hypothesis**: Plain English: the 100 Hz retrain of the best joystick recipe died at launch because the 25 Hz-trained brain was fed its 16-frame history at 4x the frame rate (its 640 ms of context shrank to 160 ms of oddly-spaced frames); this relaunch gives it a 64-frame history with a stride-4 weight transplant so at initialization it sees exactly the 40 ms-spaced inputs it was trained on (verified equivalent to the parent within 7e-6 fp noise on the real checkpoint), and training can then grow into the denser 100 Hz history. Requested by fb_20260824T180427_4c2e26; recipe otherwise the V7 certfreeze ladder at control.hz=100, physical slew preserved at 37.5 deg/s (max_delta_q_deg=0.375), warm start ppo_goal_cw_arch_hist16_dep1_c1.zip. RATE-CONVERSION CAVEAT: 40M ticks at 100 Hz is 1/4 the simulated seconds of a 25 Hz 40M run. Prediction-if-true: precert b0 clears at init (prog>=0.50, the parent's own gait) and the V7 ladder trains at 100 Hz with reward and frontier promotions rising together. Prediction-if-false: precert fails again — the transfer gap is per-tick action/physics semantics at 100 Hz, not temporal context; follow-up is a labeled from-scratch 100 Hz arm, never guard relaxation. Strongest alternative: precert passes but the ladder stalls later at 100 Hz — reads as budget/rate question per the 08-21 ruling (continue), or a 100 Hz reward/eval mismatch if reward rises while rungs do not (audit before seeds).

**gate**: PASS: init precert b0 clears (prog>=0.50) AND training healthy at 100 Hz with reward/eval AGREEMENT (frontier promotions track reward) AND 60s randomized joygate at 100 Hz: falls<=2/48, no over_current cluster, directions followed on side/rear/turn/reversal stress, slip <=~2.9/m, video shows all six feet cycling. PARTIAL: precert passes and genuinely learning (promotions + improving evals) but short of the 25 Hz lineage at budget end — continuation candidate. FAIL: precert fails again (temporal-context hypothesis refuted -> from-scratch 100 Hz arm), or reward rises while rung evals stay flat (100 Hz reward/eval mismatch — audit, do not seed-sweep).

**verdict**: Repro attempt of the --hist-stride-transplant repair CONFIRMS the mechanism is broken, not a fluke. Plain English: this is a straight re-launch of the exact hist64-transplant config that already failed once; it reproduces the same fail-closed outcome (init precert b0 prog=0.056 vs the first attempt's 0.052, both far under the 0.50 bar, 0 falls, slip/m 3.31) so the earlier reading was not noise. The script's own fail-closed guard stopped it before any PPO training (never called model.learn()), so this run spent ~0 GPU budget. Evidence: pod-2 /tmp/train_cw-arch-hist16-dep1-c1-joyfullcurr13-v7-hz100-hist64-rr1.log (copied to logs/ckpt_eval/ before rotation risk). Per the already-recorded root-cause conclusion (per-tick action authority quartered at 100Hz, not temporal context), no further --hist-stride-transplant arms are warranted; the labeled from-scratch 100Hz arm (cw-arch-hist64-joyfullcurr13-v7-hz100-scratch-s0, already VERIFIED RUNNING on train-2) is the correct live lever and needs no repro of its own transplant ancestor. Closing this arm; -r2 (plain 100Hz, no transplant) remains the live rate-conversion read.

**failed_reason**: run never appeared as 'running' in W&B within 240s

