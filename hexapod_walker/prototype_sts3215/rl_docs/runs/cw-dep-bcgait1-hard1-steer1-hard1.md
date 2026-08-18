# cw-dep-bcgait1-hard1-steer1-hard1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: KILLED_DUPLICATE

**created**: 2026-08-18T17:57:56+00:00

**pod**: hexapod-mjx-train-4

**steps**: 20000000

**parent**: cw-dep-bcgait1-hard1-steer1c

**wandb_id**: 84yg96dc

**hypothesis**: Plain sentence: teach the tall walker to keep its gait and steering healthy through MANY more abrupt joystick direction changes, for long enough to actually harden the behavior (not just survive a 2M sanity check). This continues the steer1c canary's exact recipe (120s episodes, all six stress_mix command families, instant no-blend switches, irregular 2-20s dwells) for the full ~20M hardening budget from steer1c's own weights, with --best-ckpt so a mid-run regression can't silently ship as the final checkpoint. Prediction-if-true: by 20M the direction-switch panels (det+sto) show zero falls/tangles, all six legs still cycling within ~1s of every command change, no accumulating yaw-limit saturation, height/slip back in the fixed-command hard1 champion's own band (height>=-20mm, slip<=1.8/m), roll_class mostly 'recovered' not 'leaning'. Prediction-if-false: the direction-switch cost keeps trading off against the tall gait even at 20M (slip/height stay elevated, leaning persists, or a new exploit like leg-sacrifice generalizes) — meaning the fix needs a staged dwell-time curriculum instead of the full stress_mix from step 0.

**gate**: Operator admission panel (hw/STATUS.md WAITING-ON): original fixed-command hard1 quality retained (height>=-20mm, six-leg gait-valid, zero falls, slip<=1.8/m) AND long det+sto direction-switch panels show zero falls/tangles, all legs cycling after every change, no accumulating yaw saturation, prompt tracking recovery after a switch. FAIL if any bar misses at the full budget; PASS is a deployable stance-continuation candidate for the DOWNLOAD_ANSWER walk half.

**verdict**: KILLED at ~2.9M/20M steps: this is a byte-for-byte duplicate of a CONCURRENT cycle's already-COMPLETED hardening continuation (cw-dep-bcgait1-hard1-steer1-hard20m1, W&B w3fbxfj7 — same parent steer1c checkpoint, same seed=11, same --best-ckpt, same full recipe, launched 16:39:50, ran its full 20M budget and finished before I checked pod status at 17:58). I triaged cw-dep-bcgait1-hard1-steer1c as CANARY PASS independently and launched this continuation without first checking whether a peer cycle had already acted on the same WAITING-ON entry; w3fbxfj7 existed but its ledger row (mechanically re-verified RUNNING by the launcher's self-repair loop) was stale by the time I read status, so train-4 looked free. Killed immediately on discovering the duplicate (no useful signal lost — it was 2.9M steps into reproducing a run that already has a full 20M-step answer). The real verdict belongs to w3fbxfj7; see its ledger entry.

