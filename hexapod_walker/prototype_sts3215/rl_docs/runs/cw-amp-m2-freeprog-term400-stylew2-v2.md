# cw-amp-m2-freeprog-term400-stylew2-v2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-22T16:11:12+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m2-freeprog-term400-style05

**wandb_id**: k2odtq4b

**hypothesis**: Plain English: make the AMP style channel earn enough to be worth leaving the statue for. The M2 dig-ins showed the statue basin survives every non-reward lever (term_penalty, std-anneal, stage curriculum -- all FAILED) and the bank audit shows the task reward already RANKS walking above marching/parking, so the defect is a basin barrier: no accessible income gradient from statue toward stepping. At style/task 0.5/0.5 the style channel's max income is 0.5/tick against ~-1.5/tick statue charges (the -c1 dig-in measured style priced out 30-60x under legacy pricing; still ~3x under SLIPWALK) -- so even a policy the discriminator loves cannot out-earn standing still. This arm raises the blend to style 2.0 / task 1.0 (style income up to 2/tick, now larger than the statue's freeprog+idle charges) on top of the clean teacher_v2 library, so teacher-like leg cycling becomes the best-paying ACCESSIBLE behavior and the discriminator gradient can pull the policy out of the statue. Two changes vs -style05 (lib + weights), but judged against its own single-change twin -style05-v2 (launched this cycle, same lib at 0.5/0.5), which isolates the dose.

**gate**: Discovery (2M, DR-0 harness walk mode, 6 det + 6 sto, own cfg), judged RELATIVE to -noamp and -style05-v2 at matched budget: PASS = median det fwd travel >= 0.10 m/15s AND gait_valid >= 4/6 det AND no sacrificed legs AND video shows six-leg cycling with net displacement. Also required for PASS: no style-reward degenerate loop (amp/d_real-d_fake must not saturate to a constant; style_rew not pinned at 1.0 with the robot doing teacher-mimicry in place -- in-place teacher mimicry at high style weight is the pre-registered NEW cheat to watch, and if seen it gets banked before any follow-up). Prediction-if-true: travel appears and freeprog_pen finally leaves -1.5/tick; next arm is the brief's proper style sweep at 40M. Prediction-if-false-with-style05-v2-also-statued: the AMP-income route is refuted at 2M discovery scale for this pricing family -> next is a genuinely new income mechanism (foot-airtime/contact-phase income, bank-first), not another blend dose.

