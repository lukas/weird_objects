# cw-amp-m2-pilot-style05

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: DONE

**created**: 2026-08-22T10:38:04+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**wandb_id**: pcvbw1nw

**hardware_ready**: no

**hypothesis**: Does an adversarial motion prior actually buy gait quality? First from-scratch policy trained with the newly-wired AMP style reward: task/style 0.5/0.5 (brief §5.2 mixture B) over the AMP-brief §6 joystick command envelope (the asymcritic-smoke-v3 stress_mix bundle: speed 0-0.60 m/s, full-circle heading, yaw 0-1.0 rad/s w/ 0.15 zero-frac, resample 0.5-3.0 s jittered, mixed abrupt/ramped blends), --asym-critic, discriminator co-trained online 4x512/rollout against teacher_v1 (R1 gp=10, replay 500k) per snapshot 581b03d5 (AMPStyleVecWrapper; smoke smoke_amp_style_wire_v1: finite losses, 20 disc updates, off-path bit-exact). Matched control cw-amp-m2-pilot-noamp differs ONLY in amp flags. Prediction-if-true: vs the control at equal budget, videos show a recognizably cleaner six-leg alternating tripod (all six feet cycling contact/swing), fewer falls in det panels, and amp/style_reward_mean climbs off ~0 while amp/d_real vs amp/d_fake stay separated but NOT saturated (style reward not pinned at 0 or 1). Prediction-if-false-(i): style reward pins ~0 all run (discriminator wins instantly) - raise --amp-disc-lr down / --amp-gp-weight up or cut --amp-disc-steps to 2, single change. Prediction-if-false-(ii): style reward pins ~1 (policy fools a too-weak disc, no gait change) - raise disc steps/batch. Prediction-if-false-(iii): both arms equally bad/unstable - the task envelope, not AMP, is the binding problem; narrow the command envelope (curriculum per brief §6 30-50% start) before touching AMP knobs.

**gate**: Non-champion M2 pilot, assessed RELATIVE to cw-amp-m2-pilot-noamp (matched control), not the joystick gates: PASS = at 40M, videos (det, several command draws) show the style05 arm with a qualitatively cleaner alternating-tripod gait than the control (all six feet cycling; no paddle-creep/dragging), plus amp/d_real > amp/d_fake all run without style-reward saturation (mean not pinned 0/1 for >80% of the run). FAIL branches per hypothesis (i)-(iii). Reward-rising-at-end + unclear video = continue per 08-21 ruling. No SKILLS/champion updates from a pilot; decides wave-1 (STATUS Next item 5) sizing.

**verdict**: PASS as mechanism-health discovery pilot, pre-gait on behavior (expected at this budget): 2M steps across 4096 envs is ~1.3 episodes/env — both arms are still pre-locomotion (det fwd 0.007m/15s, slip ~8.8-11.7/m, legs 1&3 sacrificed in all det episodes; 0 falls; contact sheet = sprawled near-frozen stance). The AMP machinery itself is healthy at scale: amp/d_real 0.78 vs d_fake -0.96 separated all run, style_reward_mean ~0.08 (low, discriminator winning, but NOT pinned 0 — branch (i) did not fire), gp finite, 124 disc updates, replay full. No style-vs-control difference is judgeable this early; the gate's own comparison point is 40M. CONTINUING both arms matched to 40M total (respec -c1, --init-from + --amp-disc-init) per the 08-21 ruling.

