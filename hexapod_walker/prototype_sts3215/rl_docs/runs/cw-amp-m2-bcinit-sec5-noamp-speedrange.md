# cw-amp-m2-bcinit-sec5-noamp-speedrange

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INFORMATIVE

**created**: 2026-08-22T22:34:54+00:00

**pod**: hexapod-mjx-train-3

**steps**: 2000000

**parent**: cw-amp-m2-bcinit-sec5-noamp-headingsfull

**wandb_id**: xr8dp13c

**hypothesis**: Plain English: task-only twin of cw-amp-m2-bcinit-sec5-style05-speedrange (VERDICTED INFORMATIVE/partial: gait stays clean 6/6 det+sto at the 0.05-0.25 m/s speed range but achieved speed barely modulates, clustering 0.08-0.14 m/s regardless of command -- a fixed-cadence cap). Does the noamp BC-init walker show the SAME compressed-speed signature (confirming the cap is cadence/actor-general, not an AMP-style artifact) or does it modulate speed noticeably better/worse without the style channel's pull toward the teacher's fixed-cadence clips? Continues from noamp-headingsfull (--init-from-source), single lever: goal.walk_speed_min/max_m_s 0.08/0.08 -> 0.05/0.25, identical to the style05 sibling. Prediction-if-true (matches style05): gait_valid stays >=5/6 det+sto, speed_mean again clusters ~0.08-0.14 m/s regardless of command -- confirms the cap is architecture/cadence-general, not style-induced (independent evidence alongside the same-cycle -fastphase-nostyle arm). Prediction-if-false: noamp modulates speed meaningfully better (wider realized range) or the gait degrades where style05 didn't -- either result would make style/no-style a real functional fork on this axis for the first time.

**gate**: Discovery continuation (2M, DR-0, judged against the style05-speedrange sibling). INFORMATIVE-PASS = gait_valid stays >=5/6 det+sto, no new sacrificed legs/terminations, height_err stays in the walking band. Read speed_mean's realized spread vs command against the style05 sibling's ~0.08-0.14m/s band: MATCH = cap confirmed style-independent; WIDER/NARROWER = names a real style-vs-noamp fork on speed tracking. FAIL-collapse = terminations/sacrificed legs/statue at the wider speed demand.

**verdict**: Task-only twin of style05-speedrange -- lands in the SAME partial/compressed-speed regime, confirming the speed cap is style-INDEPENDENT. DR-0 gate: gait_valid 6/6 det+sto, zero terminations/sacrificed legs, real travel 0.03-1.36m/15s; speed_mean_m_s clusters 0.085-0.13 across the sample (style05 sibling: 0.084-0.136 det/0.092-0.126 sto) despite the same 5x commanded speed range (0.05-0.25 m/s) -- essentially an exact match to the style05 sibling's band, not wider or narrower. Fires the pre-registered MATCH branch: the ~0.14 m/s realized-speed ceiling is a property of the fixed walk_phase_hz=1.333 cadence (or the BC-init gait's stride length), not something AMP style helps or hurts. Frame-strips clean six-leg cycling throughout, no drag/flag-leg/statue. hardware-ready: no (2M continuation, DR-0). Closes the style-vs-noamp speed-tracking question: matches every other axis tested this cycle (heading 0/25/90/full, now speed) -- style has shown no measured functional necessity anywhere in the M2 command envelope so far. Next real lever per both siblings' shared conclusion: cadence/stride-length or an explicit speed-tracking reward term, not more discovery budget on this config.

