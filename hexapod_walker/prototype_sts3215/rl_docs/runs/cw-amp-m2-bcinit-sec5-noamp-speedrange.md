# cw-amp-m2-bcinit-sec5-noamp-speedrange

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-22T22:34:54+00:00

**pod**: hexapod-mjx-train-3

**steps**: 2000000

**parent**: cw-amp-m2-bcinit-sec5-noamp-headingsfull

**hypothesis**: Plain English: task-only twin of cw-amp-m2-bcinit-sec5-style05-speedrange (VERDICTED INFORMATIVE/partial: gait stays clean 6/6 det+sto at the 0.05-0.25 m/s speed range but achieved speed barely modulates, clustering 0.08-0.14 m/s regardless of command -- a fixed-cadence cap). Does the noamp BC-init walker show the SAME compressed-speed signature (confirming the cap is cadence/actor-general, not an AMP-style artifact) or does it modulate speed noticeably better/worse without the style channel's pull toward the teacher's fixed-cadence clips? Continues from noamp-headingsfull (--init-from-source), single lever: goal.walk_speed_min/max_m_s 0.08/0.08 -> 0.05/0.25, identical to the style05 sibling. Prediction-if-true (matches style05): gait_valid stays >=5/6 det+sto, speed_mean again clusters ~0.08-0.14 m/s regardless of command -- confirms the cap is architecture/cadence-general, not style-induced (independent evidence alongside the same-cycle -fastphase-nostyle arm). Prediction-if-false: noamp modulates speed meaningfully better (wider realized range) or the gait degrades where style05 didn't -- either result would make style/no-style a real functional fork on this axis for the first time.

**gate**: Discovery continuation (2M, DR-0, judged against the style05-speedrange sibling). INFORMATIVE-PASS = gait_valid stays >=5/6 det+sto, no new sacrificed legs/terminations, height_err stays in the walking band. Read speed_mean's realized spread vs command against the style05 sibling's ~0.08-0.14m/s band: MATCH = cap confirmed style-independent; WIDER/NARROWER = names a real style-vs-noamp fork on speed tracking. FAIL-collapse = terminations/sacrificed legs/statue at the wider speed demand.

