# cw-arch-hist16-dep1-c1-joyfullcurr13-v7-hz100-hist64

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-24T18:17:30+00:00

**pod**: hexapod-mjx-train-2

**steps**: 40000000

**parent**: cw-arch-hist16-dep1-c1-joyfullcurr13-v7-hz100

**hypothesis**: Plain English: the 100 Hz retrain of the best joystick recipe died at launch because the 25 Hz-trained brain was fed its 16-frame history at 4x the frame rate (its 640 ms of context shrank to 160 ms of oddly-spaced frames); this relaunch gives it a 64-frame history with a stride-4 weight transplant so at initialization it sees exactly the 40 ms-spaced inputs it was trained on (verified equivalent to the parent within 7e-6 fp noise on the real checkpoint), and training can then grow into the denser 100 Hz history. Requested by fb_20260824T180427_4c2e26; recipe otherwise the V7 certfreeze ladder at control.hz=100, physical slew preserved at 37.5 deg/s (max_delta_q_deg=0.375), warm start ppo_goal_cw_arch_hist16_dep1_c1.zip. RATE-CONVERSION CAVEAT: 40M ticks at 100 Hz is 1/4 the simulated seconds of a 25 Hz 40M run. Prediction-if-true: precert b0 clears at init (prog>=0.50, the parent's own gait) and the V7 ladder trains at 100 Hz with reward and frontier promotions rising together. Prediction-if-false: precert fails again — the transfer gap is per-tick action/physics semantics at 100 Hz, not temporal context; follow-up is a labeled from-scratch 100 Hz arm, never guard relaxation. Strongest alternative: precert passes but the ladder stalls later at 100 Hz — reads as budget/rate question per the 08-21 ruling (continue), or a 100 Hz reward/eval mismatch if reward rises while rungs do not (audit before seeds).

**gate**: PASS: init precert b0 clears (prog>=0.50) AND training healthy at 100 Hz with reward/eval AGREEMENT (frontier promotions track reward) AND 60s randomized joygate at 100 Hz: falls<=2/48, no over_current cluster, directions followed on side/rear/turn/reversal stress, slip <=~2.9/m, video shows all six feet cycling. PARTIAL: precert passes and genuinely learning (promotions + improving evals) but short of the 25 Hz lineage at budget end — continuation candidate. FAIL: precert fails again (temporal-context hypothesis refuted -> from-scratch 100 Hz arm), or reward rises while rung evals stay flat (100 Hz reward/eval mismatch — audit, do not seed-sweep).

**failed_reason**: process died; log tail:
[walkcurr-precert] b0 bridge_10s @ step 0 (pre-PPO, det, n=8): FAIL six_leg_gait,progress,slip,progress_p10 prog=0.052 falls=0.00 slip/m=3.65 roll=1.5 h_err=-0.8mm hf=1.00 slew=0.87 stop=-1.0000 stop_settled=-1.0000 settled_frac=-1.00
[walkcurr-precert] b1 front45_20s @ step 0 (pre-PPO, det, n=8): FAIL progress,slip,progress_p10 prog=0.164 falls=0.00 slip/m=6.47 roll=2.7 h_err=-4.1mm hf=0.99 slew=0.91 stop=0.0090 stop_settled=0.0090 settled_frac=1.00  [informational]
[walkcurr-precert] the INITIAL policy fails the survive/walk bar under exact b0 (falls=0.00, prog=0.052 < 0.50, WORSE than the plain hz100 attempt's 0.203) -- hist-stride-transplant did not fix the transplant/obs mapping, made it worse; needs dig-in before retry (fb_20260824T180427_4c2e26 lineage)
wandb: View run cw-arch-hist16-dep1-c1-joyfullcurr13-v7-hz100-hist64 at: https://wandb.ai/l2k2/hexapod-balance/runs/ppjeexty

