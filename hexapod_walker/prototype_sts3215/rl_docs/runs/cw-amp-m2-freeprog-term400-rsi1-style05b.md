# cw-amp-m2-freeprog-term400-rsi1-style05b

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-22T17:42:03+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m2-freeprog-term400-style05-v2

**wandb_id**: 7m4yd9ar

**hypothesis**: Style-vs-control twin to cw-amp-m2-freeprog-term400-rsi1-noamp (respec retry of rsi1-style05, which hit a tag collision from an earlier REFUSED attempt and never actually launched -- this is the real first attempt): does the AMP style channel (teacher_v2 lib, 0.5/0.5, the already-verdicted-FAIL style05-v2 config) add anything ON TOP of the RSI-for-walk mid-gait spawn (goal.walk_gait_start_frac=0.5), now that the whole reward-side ladder for this basin (term-penalty, std-anneal, staging, style-dose alone, k_walk_swing) is closed and RSI is the first lever that changes the INITIAL STATE instead of the reward? Single lever vs cw-amp-m2-freeprog-term400-rsi1-noamp: identical RSI cfg, PLUS teacher_v2 AMP style/task 0.5/0.5. Prediction-if-true: this arm's fwd travel clears noticeably ABOVE its rsi1-noamp twin's, because once RSI removes the discovery barrier the discriminator can finally push the sustained gait toward the teacher's clean cadence/coordination instead of a shuffle. Prediction-if-false: both RSI arms land at the same fwd-travel reading -- RSI supplies the whole gradient (if any) and AMP's contribution to this family stays unproven even with the discovery barrier removed. Cheat watch: in-place teacher mimicry (style_reward_mean pinned near 1.0 with fwd travel ~0) as in every prior style arm (never fired so far).

**gate**: Discovery (2M, DR-0 harness walk mode, 6 det + 6 sto, own cfg), judged RELATIVE to cw-amp-m2-freeprog-term400-rsi1-noamp (twin) and the whole statue family at matched budget: PASS = median det fwd travel >= 0.10 m/15s AND gait_valid >= 4/6 det AND no sacrificed legs AND video shows six legs cycling with net displacement. INFORMATIVE = fwd travel clears meaningfully above its rsi1-noamp twin's reading (AMP adds real value once RSI removes the discovery barrier) even short of the 0.10m bar, or vice versa (indistinguishable = AMP still unproven for this family). Cheat check: in-place-teacher-mimicry (style_reward_mean pinned near 1.0, fwd travel ~0) voids a PASS/INFORMATIVE read regardless of return.

