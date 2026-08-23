# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-stdanneal45

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INVALID

**created**: 2026-08-23T14:12:18+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518

**wandb_id**: 4wd90gkv

**hypothesis**: Plain English: this family's walk drags every loaded foot ~2x farther per stance than the joystick champion (11.49mm median vs 5.5mm, same command/geometry), and every reward-side lever is now measured useless (additive 6x/12x, full income gate, swing income — swing1 FAIL: median WORSENED to 13.54mm, only the tail compressed). The one big mechanical difference left: this family trains at a FIXED action-noise std 0.135 (--warm-log-std-override -2, no anneal) while the 5.5mm-stance champion stotight45s13 annealed to log-std -4.5, and the joystick track MEASURED (08-22 pd8 dig-in) that the 0.135 noise floor is exactly what makes honest stances drag — the learned mean gait is shaped by the noise it trains under. This arm changes ONE lever: --log-std-final=-4.5 (std 0.135 -> 0.011 over the run) on the unchanged pushcal518 recipe. Prediction-if-true: slipdist probe stance median drops from 11.49 toward <=9mm and m5 walk det slip med reaches <=3.5 with falls/gait/tips unchanged. Prediction-if-false: median unmoved — noise floor is not the amp slip mechanism. Strongest alternative: the slip level is DEMO-ANCHORED — the AMP style term holds slip near the teacher demos' own slip (noamp1 showed style is the slip floor-holder), so no policy-side noise change moves it; if so the next mechanism is a cleaner motion library (CPG contextual winner gait, slip 0.56-0.63/m, as teacher v4).

**gate**: eval_amp_m5 walk+yaw own cfg + slipdist probe rerun (hazard-free own-cfg, seed 0, 6 eps). PASS = 0/12 raw falls AND walk det slip med <=3.5 AND walk gait_valid 12/12 AND tips within 0.25 band AND probe stance median <=9mm. PARTIAL = probe median drops >=2.5mm vs parent's 11.49 (mechanism real at stance level) but m5 slip misses 3.5 — dose next (-5.0 rung per the joystick ladder). FAIL = probe median unmoved (>=10.5) AND slip unmoved (+-0.15 of 3.67) — noise-floor mechanism refuted for amp; remaining fork = demo-anchor mechanism (teacher v4 from CPG winner) vs the 3.5-bar ruling amendment (q_20260823T0130Z), with pricing/income/demos/gait-income/noise ALL then measured refuted.

**verdict**: INVALID — this run never trained: zero PPO gradient steps were taken, so the noise-floor hypothesis is UNTESTED, not refuted. Forensics: the saved checkpoint's 12 network tensors (actor AND critic) are byte-identical to the warm-start ancestor ppo_goal_cw_amp_m4_turnfault_seq1.zip; only log_std moved (annealed to -4.5 on schedule). Root cause: the --log-std-final callback sets log_std at on_rollout_end, i.e. BETWEEN collection and PPO's train(), so every first minibatch shows approx_kl ~0.13 purely from the log_std shift (18 dims x 0.082 log-std step/rollout at this 2.5-nat/2M anneal rate); SB3's target_kl=0.02 early-stop (trainer default) fires BEFORE optimizer.step() — the pod log shows 'Early stopping at step 0 due to reaching max kl' on 31/31 updates. The 'rising reward' curve (43->268) was pure artifact: shrinking action noise makes the SAME frozen policy score better — a textbook trap for the 08-21 reward-rising ruling; weight-level checks now added to triage. Blast radius audited: the joystick stotight ladder (same flag, target-kl 0.02) is CLEAN — all champions show real weight deltas (their anneal rate per rollout was ~6x smaller, KL bump below the early-stop line). Sibling swinganneal45 froze identically (byte-identical policy to this run — its swing-income cfg only re-priced the same trajectories). Next: trainer fixed this cycle (anneal moved to on_rollout_start so buffer log_probs stay consistent, ratios start at 1), smoke-verified weights move, both arms relaunched as -r2 on the fixed code with unchanged hypotheses/gates.

