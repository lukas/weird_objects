# cw-amp-m2-bcinit-sec5-style05-yawcmd-tip90-clockyaw

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-22T23:20:30+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-amp-m2-bcinit-sec5-style05-yawcmd-tip90

**wandb_id**: vz07tkta

**hypothesis**: Plain English: dose twin of tip50-clockyaw -- with the gait clock finally ticking during commanded turns, does maximum turn exposure (90% of episodes) speed up or strengthen turn-in-place learning? Same frozen-clock root cause and fix as the tip50-clockyaw arm (goal.walk_phase_run_on_yaw=1, landed+tested this cycle, default-off bit-exact); byte-identical to tip90 otherwise, so tip90 (parked, tip err 0.2982/0.2999 == |wz_ref|) is the matched control. The original tip50/tip90 pair showed ZERO dose-response because the skill was unlearnable with a frozen clock at any exposure; if the clock was the binding constraint, dose-response should now APPEAR (this arm learns turn-in-place faster/stronger than tip50-clockyaw) -- that pattern would confirm the clock diagnosis twice over. Prediction-if-true: eval_yaw (with the clock key in cfg-set) tip errs drop well below 0.30 with command-signed wz both directions, at least matching tip50-clockyaw. Prediction-if-false: both clockyaw arms still park -- clock refuted as binding constraint, BC-turn-clone next. Translation erosion tolerated on this 0.9-frac arm (stage read, same as tip90).

**gate**: Discovery (2M, DR-0). PASS = eval_yaw manual on the pod with the run cfg INCLUDING goal.walk_phase_run_on_yaw=1 (--speed 0.08 --wz-max 0.3): tip-left AND tip-right err <= 0.20, achieved wz sign matching wz_ref both directions, zero falls (translation erosion tolerated at 0.9 frac; report non-tip-episode gait from the DR-0 panel per-episode -- medians will be ~90% tip-contaminated, see tip50-r2 verdict gotcha). PARTIAL-step = stepping-in-place on tip video with |wz| < 0.1. FAIL-park = tip errs ~0.30 with running clock. Joint dose read with -tip50-clockyaw: clock binding => dose-response appears; both park => clock refuted, BC-turn-clone next.

**verdict**: Dose twin of tip50-clockyaw (90% vs 50% turn-in-place exposure) -- ALSO fires FAIL-park, closing the dose question too. eval_yaw (already generated, this cycle's triage): turn_wz_err_med 0.271 rad/s (gate <=0.1), hold_wz_med 0.0025 (clean), 0 falls all scenarios. Matches tip50-clockyaw's 0.268 almost exactly -- more turn-in-place exposure (90% vs 50% of episodes) does not help once the clock-timing hypothesis itself is refuted; this isn't a dose problem, the mechanism (clock running during turns) simply doesn't supply what's missing. Confirms the same root cause named for tip50-clockyaw: the BC-init policy has no turning motor pattern in its repertoire at all -- clock rate and turn-exposure fraction were both red herrings. hardware-ready: no (no turning). CLOSES the entire clock-timing + turn-exposure-dose lever family for yaw command-following on this substrate; next untried lever per the shared gate text is BC-turn-clone (an actual turning demonstration), not further reward/exposure tuning.

