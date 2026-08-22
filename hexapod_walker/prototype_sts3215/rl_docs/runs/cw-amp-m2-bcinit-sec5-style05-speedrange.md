# cw-amp-m2-bcinit-sec5-style05-speedrange

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-22T22:03:16+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-amp-m2-bcinit-sec5-style05-headingsfull

**wandb_id**: dehcrq5s

**hypothesis**: Plain English: the AMP walker now follows commands in EVERY direction at one fixed speed (0.08 m/s) -- can it also modulate SPEED on command, from a slow 0.05 m/s crawl up to 0.25 m/s (3x anything the BC tripod teacher ever demonstrated)? Continues from the headingsfull checkpoint (full-circle headings retained, same sec5 minimal reward + amp 0.5/0.5, fresh disc per stage protocol), single lever: goal.walk_speed_min/max_m_s 0.08/0.08 -> 0.05/0.25. Stops are NOT new (walk_park_start_frac=0.25 already trains them); the new demand is speed VARIETY. Prediction-if-true: gait_valid stays >=5/6 det+sto, height_err in the 18-31mm band, and achieved speed correlates with commanded speed (vel_err does not collapse to a fixed-0.08 fingerprint). Prediction-if-false: high-speed commands destabilize the gait (terminations/sacrificed legs return) or the policy ignores speed and walks 0.08 regardless (vel_err ~ |cmd-0.08|). Strongest alternative: partial -- speeds up to ~0.15 tracked, 0.2+ physically out of reach at this gait frequency; per-episode speed_mean vs command spread will show the ceiling.

**gate**: Discovery continuation (2M, DR-0). INFORMATIVE-PASS = gait_valid >=5/6 det+sto at own cfg, no new sacrificed legs, height_err 18-31mm band, real travel on move segments, and evidence of speed MODULATION (achieved speed varies with command instead of pinning at 0.08). FAIL-collapse = statue/drag/sacrificed legs/terminations return. PARTIAL (informative) = low band tracked, top band capped -- names the speed ceiling for the envelope schedule.

