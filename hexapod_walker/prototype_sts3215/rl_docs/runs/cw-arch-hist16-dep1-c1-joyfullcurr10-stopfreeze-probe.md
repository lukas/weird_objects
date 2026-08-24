# cw-arch-hist16-dep1-c1-joyfullcurr10-stopfreeze-probe

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INFORMATIVE

**created**: 2026-08-24T08:47:01+00:00

**pod**: hexapod-mjx-train-0

**steps**: 40000000

**parent**: cw-arch-hist16-dep1-c1-joyfullcurr9-stopcur2

**hypothesis**: Plain English: the stopsettle-probe (this cycle) proved the V6 b1 stop-cert creep is a genuine post-grace floor, not a measurement artifact or a price-sensitive quantity (settled speed 0.0311 vs raw 0.0326, ~5%) -- refuting the entire stop-speed/stop-current reward-pricing lever and naming its own next step in its gate text: 'the next lever must be structural (an explicit freeze/hold controller on stop commands) rather than more reward pricing.' This cycle built that exact lever: goal.walk_stop_freeze_s (sim_env._walk_stop_freeze_override, default 0.0=off/bit-exact, 5/5 unit tests in test_walk_stop_freeze.py) discards the policy's own action once a walk-mode stop segment has been commanded past the threshold and re-issues the PREVIOUS tick's own safe command instead -- a physical hold, not a price. This is another precert-only DRY RUN (no PPO, exits in ~15s) reading the SAME already-trained joyfullcurr9-stopcur2 checkpoint, testing whether the structural hold (not retraining) is enough to pass the b1 cert outright.

**gate**: Diagnostic only. If walkcurr/pre_b1_stop_speed_m_s reads <=0.015 with the freeze on: the structural hold alone satisfies the existing b1 cert on an UNCHANGED checkpoint -- next step is wiring walk_stop_freeze_s into an actual training run (or a permanent eval/deploy-time supervisory mode) rather than any further reward-pricing arm. If it stays above 0.015: the hold itself is imperfect (e.g. contact/gravity settling drift even from a truly static command, or the override's grace/exemption logic is leaking) -- dig into the per-tick command trace before trying a wider-scope override (e.g. holding position via a low-level PD setpoint rather than the last IK-mapped command).

**verdict**: TRUE branch of the pre-registered gate: the structural stop-hold lever (goal.walk_stop_freeze_s, sim_env._walk_stop_freeze_override, built+unit-tested this cycle, 5/5 test_walk_stop_freeze.py, default off/bit-exact) makes the SAME already-trained joyfullcurr9-stopcur2 checkpoint PASS the b1 front45_20s stop cert outright with ZERO additional training: walkcurr/pre_b1_pass 0->1, stop_speed_m_s 0.0326->0.0133 (under the 0.015 cap for the first time in the whole V6 ladder), stop_speed_settled_m_s 0.0054, with no cost elsewhere (prog_frac 1.06, falls 0, roll 2.8deg, h_err -0.7mm, slew 0.91 -- all in the parent's own healthy band). This is a genuinely different fix from every prior reward-pricing arm (joyfullcurr6-10): a supervisory command override that discards the policy's own action once a walk-mode stop segment passes the same 0.4s grace window the reward charges already exempt, and re-issues the previous tick's own safe command -- a physical hold, not a price. Confirms the stopsettle-probe's own prediction exactly (methodology audit -> structural lever -> works). Mechanically it means the entire stop-speed/stop-current reward-pricing question this ladder spent 6 arms on was never going to be solved by pricing alone; the fix was a control-layer addition. REFILL: launched cw-arch-hist16-dep1-c1-joyfullcurr11-freeze40 (real 40M-step warm-start from stopcur2 + this lever) to test whether walkcurr/frontier finally promotes past b1 into the untouched b2-b9 buckets (side90/rear/full-circle, the actual point of the operator's full-circle order) under real training + the full randomized joygate mix, not just the scripted precert probe. Evidence: logs/experiments/cw-arch-hist16-dep1-c1-joyfullcurr10-stopfreeze-probe/, W&B run u3go1oj8; code: sim_env.py _walk_stop_freeze_override, walk_task.py (3 reset-site inits), test_walk_stop_freeze.py (5/5 new), snapshot a6a4546e.

**failed_reason**: run never appeared as 'running' in W&B within 240s

