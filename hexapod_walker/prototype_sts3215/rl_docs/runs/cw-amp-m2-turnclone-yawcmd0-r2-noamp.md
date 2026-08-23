# cw-amp-m2-turnclone-yawcmd0-r2-noamp

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T01:01:37+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-amp-m2-turnclone-yawcmd0-r2

**wandb_id**: 9fim4cwy

**hypothesis**: Plain English: cw-amp-m2-turnclone-yawcmd0-r2 (style05) just showed a turn-taught BC init plus the full heading/backward/lateral command mix produces ONE checkpoint that clears the standard DR-0 walk gate (gait_valid 6/6 det+sto, slip med 2.24, dir med 40deg -- as good as or better than the plain headingsfull lineage) AND passes eval_yaw's turn-in-place bar (tip err 0.15/0.16) with zero dedicated tip exposure -- the first candidate that plausibly combines M2's turning-and-translating requirements in one policy. Every prior M2 axis (headings, speed, push, fault) found AMP style functionally neutral vs a task-only twin; does that hold on this COMBINED turn+heading substrate too, or does style matter once both axes stack? Single lever vs the style05 run: --amp-style-weight 0.5 -> 0.0 (task-only), same turn-clone init, same full cfg (heading -1, yaw_cmd on, stress_mix, 2M).

**gate**: Discovery (2M, DR-0). Compare against cw-amp-m2-turnclone-yawcmd0-r2's own DR-0 gate (gait_valid 6/6 det+sto, det prog med 0.9455/slip med 2.235/dir med 40.0) and its eval_yaw turn read (tip err 0.1525/0.1614): noamp within noise on all axes = style-neutral pattern extends to the combined substrate; noamp clearly worse (either walk gate axes or turn tip err) = first real positive style finding on this substrate; noamp clearly better = a negative style finding worth a dig-in. Zero falls / gait_valid required either way; run eval_yaw manually on the resulting checkpoint (same cfg as the style05 sibling) since the standard gate does not cover turn tracking.

