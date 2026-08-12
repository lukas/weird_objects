# cw-getup1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-12T01:21:57+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**wandb_id**: zjltaik9

**hardware_ready**: False

**hypothesis**: From-scratch unified recover-stand-walk: teach ONE policy to get up from ANY pose (random tangle, belly zero, partial curl, crouch, plant, tripod park), reach a supported stand, and walk on command — with NO reference trajectory, NO BC anchor, and NO warm start (fresh init IS the hypothesis, per operator directive 08-11: the right reward should train this from scratch). Income only pays measured, supported progress: a one-shot staged ratchet (untangle -> weight-on-feet -> supported-stand score S coupling height to MEASURED foot load, RL_PLAN queue 2b) plus S-gated hold/walk pay; kernel income stripped; falls non-terminal at a 60 deg envelope so recovery is a state, not a death. The diverse start distribution IS the curriculum (backward-chaining across the unpaid curl). Prediction-if-true: within 2M steps env/getup_best climbs well above the spawn-seed band (~0.2) toward 0.5+, reward_getup_hold becomes a real income line, and eval video shows deliberate untangle-crouch-rise attempts. Prediction-if-false: getup_best stays pinned at seed level with near-zero hold income — the unpaid curl valley is too wide for PPO exploration and the mode needs a bridge (e.g. start-mix reweighting toward partial/crouch). Strongest alternative: the policy only farms plant/park-start episodes for S-hold income and never recovers from floor starts — visible as getup_best bimodal by start kind and flat floor-start hold income.

**gate**: MDP_PREFLIGHT GETUP banks green at launch commit fa84a39 (8/8 pass: replay dominates freeze/flagleg/thrash 2x+50, freeze earns ~0, stilt/flagleg hold scraps per-tick, gait dominates park/shuffle 2x+50). Run PASS if by 2M steps mean env/getup_best > 0.4 AND floor-start (tangle/zero) episodes show rising getup_S in eval AND video shows a credible getup attempt with no flag-leg/belly-shuffle exploit dominating; FAIL if getup_best flat at seed level or an exploit dominates video.

**verdict**: FAIL — fresh-init unified getup never learns to stand. env/getup_S (the actual stand-quality factor) stayed pinned at 0.007-0.03 the whole 2M steps (spawn-level); the reported env/getup_best rise (0.38->0.47) is explained entirely by the cheap untangle+load terms (max 0.40 without any real stand), not by standing. ep_rew_mean fell monotonically -9.7->-34.8 as ep_len_mean grew 9->465 (falls non-terminal just bought longer, more negative episodes, no more task income). W&B rollout videos (4 getup episodes at the final checkpoint) show no getup attempt at all: the robot settles into a static splayed/collapsed pose and holds it for the whole episode in every episode — this is the pre-registered FAIL alternative almost verbatim ('the unpaid curl valley is too wide for PPO exploration'). No harness eval possible (eval_checkpoint.py has no 'getup' goal mode — a real gap, noted for backlog, not blocking this verdict since training-video + training-metric evidence alone already fail check 1 of RUN_INTERPRETATION_RULES).

