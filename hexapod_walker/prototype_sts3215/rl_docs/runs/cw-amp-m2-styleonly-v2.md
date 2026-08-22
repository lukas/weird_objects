# cw-amp-m2-styleonly-v2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-22T17:59:23+00:00

**pod**: hexapod-mjx-train-3

**steps**: 2000000

**parent**: cw-amp-m2-freeprog-term400-style05-v2

**wandb_id**: lj5om8cc

**hypothesis**: Plain English: can the AMP style reward ALONE teach a from-scratch policy to move like the teacher, once it is not buried under the task reward's charges? Every M2 style arm to date (style05/stylew2/rsi1-style05b lineage) ran style income (realized 0.03-0.14/tick) an order of magnitude under the freeprog task stack's flat -1.4 to -1.5/tick charges, and amp/style_reward_mean stayed pinned at 0.05-0.07 in all of them — the policy never even partially learned to imitate, so whether the discriminator GRADIENT is learnable at all in this stack has never actually been tested. Single change vs cw-amp-m2-freeprog-term400-style05-v2 (teacher_v2 lib): --amp-task-weight 0.5->0.0, --amp-style-weight 0.5->1.0 — canonical AMP pretraining, pure imitation, no task income or charges (term_penalty=400 is inert at task weight 0; style income is positive per tick so survival pays naturally, no suicide-basin risk). Prediction-if-true: style_reward_mean climbs decisively off the 0.05-0.07 pinned band and video shows coherent six-leg cyclic tripod-like motion (net travel NOT required — there is no command to follow); the M2 fix is then task/style income rebalancing, not AMP mechanism work. Prediction-if-false: style stays pinned ~0.05-0.07 even with zero competition — the discriminator gradient itself is too weak/mis-scaled (features, disc lr, update ratio), and the dig-in goes to the AMP mechanism, not reward pricing. Strongest alternative: style climbs via a degenerate in-place teacher mimicry — still INFORMATIVE (proves the channel is learnable), must be named on video.

**gate**: Discovery (2M, judged on W&B amp/* + video, harness walk numbers recorded but NOT the bar — no task reward means no command-following claim): INFORMATIVE-PASS = amp/style_reward_mean rises clearly above the historic 0.05-0.07 pinned band (>=0.3 by run end) with d_real/d_fake unsaturated, AND video shows cyclic multi-leg motion (in-place mimicry counts, must be named). FAIL = style_reward_mean stays pinned <=0.1 all run — AMP-mechanism defect, redirect dig-in to discriminator scaling/schedule. Void if the discriminator saturates (d_fake collapse) — that is a different, named defect.

