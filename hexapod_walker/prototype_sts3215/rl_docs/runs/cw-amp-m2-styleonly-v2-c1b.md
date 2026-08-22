# cw-amp-m2-styleonly-v2-c1b

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-22T18:17:07+00:00

**pod**: hexapod-mjx-train-3

**steps**: 10000000

**parent**: cw-amp-m2-styleonly-v2

**wandb_id**: sjtm6znp

**hypothesis**: Plain English: continuation of the pure-imitation probe — does AMP style income keep climbing into actual cyclic teacher-like motion with 5x the budget, or does it plateau at the stork-pose mimicry seen at 2M? (Name suffix b: first attempt tag-collided with its own phase-cap REFUSAL, never launched.) cw-amp-m2-styleonly-v2 ended with style_reward_mean 0.119 rising monotonically (no plateau, disc unsaturated) but the behavior was a half-tripod stork statue (legs 1,3,5 sacrificed, one leg waving, fwd 0.02m). +10M steps, policy AND discriminator warm-started (the 08-21 continue branch: the gate metric IS the reward and it was still rising; the joystick lineage's re-init rule applies to PRICING repairs — this changes nothing about the objective). Prediction-if-true: style_reward_mean crosses ~0.3 and video shows multi-leg cyclic motion (in-place stepping counts) — the M2 fix is then task/style income rebalancing (feeds q_20260822T1815Z's brief-section-5 redesign). Prediction-if-false: style plateaus <=0.2 with the same static stork — the discriminator rewards static pose resemblance too well, indicting the 60-dim single-step transition feature set (too little temporal context) rather than reward pricing; dig-in goes to feature/horizon design. Strongest alternative: style climbs via a DIFFERENT degenerate loop (e.g. rhythmic twitching without weight transfer) — name it on video, still informative for the feature-design question.

**gate**: Acquisition continuation (10M, W&B amp/* + video + DR-0 harness walk mode recorded-not-gated): INFORMATIVE-PASS = style_reward_mean >=0.3 by run end with disc unsaturated AND video shows cyclic multi-leg motion with weight transfer (net travel not required). FAIL = style plateaus (<=0.2, flat final quarter) with the stork statue persisting — closes budget as the missing ingredient for pure imitation and redirects to discriminator feature/horizon design. Void on disc saturation (d_fake collapse), a separate named defect.

