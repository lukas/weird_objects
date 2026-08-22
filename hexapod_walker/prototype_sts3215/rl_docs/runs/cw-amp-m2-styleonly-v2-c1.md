# cw-amp-m2-styleonly-v2-c1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-22T18:16:14+00:00

**pod**: hexapod-mjx-train-3

**steps**: 10000000

**parent**: cw-amp-m2-styleonly-v2

**hypothesis**: Plain English: continuation of the pure-imitation probe — does AMP style income keep climbing into actual cyclic teacher-like motion with 5x the budget, or does it plateau at the stork-pose mimicry seen at 2M? cw-amp-m2-styleonly-v2 ended with style_reward_mean 0.119 rising monotonically (no plateau, disc unsaturated) but the behavior was a half-tripod stork statue (legs 1,3,5 sacrificed, one leg waving, fwd 0.02m). +10M steps, policy AND discriminator warm-started (the 08-21 continue branch: the gate metric IS the reward and it was still rising; note the joystick lineage's re-init rule applies to PRICING repairs — this changes nothing about the objective). Prediction-if-true: style_reward_mean crosses ~0.3 and video shows multi-leg cyclic motion (in-place stepping counts) — the M2 fix is then task/style income rebalancing (feeds q_20260822T1815Z's brief-section-5 redesign). Prediction-if-false: style plateaus <=0.2 with the same static stork — the discriminator rewards static pose resemblance too well, indicting the 60-dim single-step transition feature set (too little temporal context) rather than reward pricing; dig-in goes to feature/horizon design. Strongest alternative: style climbs but via a DIFFERENT degenerate loop (e.g. rhythmic twitching without weight transfer) — name it on video, still informative for the feature-design question.

**gate**: Discovery continuation (10M, W&B amp/* + video + DR-0 harness walk mode recorded-not-gated): INFORMATIVE-PASS = style_reward_mean >=0.3 by run end with disc unsaturated AND video shows cyclic multi-leg motion with weight transfer (net travel not required). FAIL = style plateaus (<=0.2, flat final quarter) with the stork statue persisting — closes budget as the missing ingredient for pure imitation and redirects to discriminator feature/horizon design. Void on disc saturation (d_fake collapse), a separate named defect.

**refused_reason**: discovery runs cap at 2000000 steps (asked 10000000): the question is 'did qualitatively correct behavior emerge?' - continue as --phase hardening with --evidence.

