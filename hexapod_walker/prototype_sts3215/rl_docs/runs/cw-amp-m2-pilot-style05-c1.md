# cw-amp-m2-pilot-style05-c1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-22T11:44:58+00:00

**pod**: hexapod-mjx-train-1

**steps**: 38000000

**parent**: cw-amp-m2-pilot-style05

**wandb_id**: fn64do78

**hardware_ready**: no

**hypothesis**: Does an adversarial motion prior buy visible gait quality? Continuation of the 2M pilot to the gate's own 40M comparison point — at 2M both arms were equally pre-locomotion (~1.3 episodes/env), so the style-vs-control question is still open. Warm-starts BOTH the policy (pilot checkpoint) and the discriminator (--amp-disc-init, 124 updates of head start). Prediction-if-true: at 40M total, videos show a recognizably cleaner six-leg alternating tripod than cw-amp-m2-pilot-noamp-c1 (all six feet cycling, no sacrificed legs 1&3 like the pilot's det episodes), with amp/d_real > d_fake un-saturated and style reward off its 0/1 pins for >80% of the run. Fail branches (i)-(iii) inherited from the pilot: (i) style reward pins ~0 (disc too strong at scale) -> cut disc steps to 2; (ii) pins ~1 -> raise disc steps/batch; (iii) both arms still not locomoting at 40M -> the §6 command envelope is the binding problem, narrow it per brief §6 curriculum before touching AMP knobs.

**gate**: Non-champion M2 pilot continuation, judged RELATIVE to cw-amp-m2-pilot-noamp-c1 at 40M total (matched budget): PASS = videos (several det command draws) show a qualitatively cleaner alternating-tripod gait than the control (all six feet cycling, no paddle-creep/dragging/sacrificed legs), plus amp/d_real > amp/d_fake all run without style-reward saturation (not pinned 0/1 for >80% of run). Reward-rising-at-end + unclear video = continue per 08-21 ruling. No SKILLS/champion updates; decides wave-1 sizing (STATUS Next item 5).

**verdict**: MISALIGNED (08-21 ruling); style-vs-control question UNANSWERED, not refuted. Identical frozen half-tripod statue as the no-AMP control at matched 40M (gait_valid 0/12, prog ~0.01, slip 9-13/m) because the task reward statue income (~1.5-1.9/tick) outbids the style channel ~30-60x (style_reward_mean 0.06 x weight 0.5 = 0.03/tick) — the AMP mechanism itself stayed healthy all run (d_real 0.97 vs d_fake -0.96, gp finite, 2444 disc updates, never saturated). A discriminator cannot rescue a task reward whose optimum is a statue. Fix arm: cw-amp-m2-freeprog-style05 on the bank-calibrated freeprog stack, from scratch.

