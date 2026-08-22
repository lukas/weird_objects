# cw-amp-m2-bcinit-sec5-noamp-yawcmd-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-22T22:41:19+00:00

**pod**: hexapod-mjx-train-4

**steps**: 2000000

**parent**: cw-amp-m2-bcinit-sec5-noamp-headingsfull

**hypothesis**: Plain English: task-only twin of cw-amp-m2-bcinit-sec5-style05-yawcmd -- can the noamp BC-init walker also learn to TURN on command (yaw-rate +/-0.3 rad/s) with the full bank-verified turn pricing set, or is the AMP style channel's real turn-in-place clips (teacher_v2) specifically necessary to organize turning motion the way it was NOT necessary for heading/direction generalization (8/8 PASS with or without style)? Continues from noamp-headingsfull (--init-from-source), identical single-lever addition to the style05 sibling: obs 73->74 via --obs-pad-transplant 1, full turn pricing set (k_walk_yaw/walk_yaw_kernel_gate/k_yaw_prog/k_yaw_still/walk_kernel_yaw_gate/walk_yaw_hold_prog_gate/yaw_still_avg_s), no AMP flags. (-r1 suffix: first attempt's run name collided with a tag pushed by a stale earlier retry of this same launch by a concurrent cycle.) Prediction-if-true (matches style05 whichever way it lands): turning either emerges or stays command-invariant on BOTH arms equally -- turning is a task-reward/architecture question, not a style question. Prediction-if-false: style05 turns but noamp doesn't (or vice versa) -- the first real functional style fork in this lineage, on exactly the axis the AMP brief predicts style should matter most for.

**gate**: Discovery continuation (2M, DR-0, judged jointly with the style05-yawcmd sibling/its already-landed verdict). INFORMATIVE-PASS = gait_valid stays >=5/6 det+sto, translation dir_err does not degrade >15deg vs noamp-headingsfull's ~33/50deg. Read wz tracking against the style05-yawcmd reading: MATCH = turning is style-independent; FORK = first real functional style benefit in the lineage. FAIL-collapse = terminations/sacrificed legs/statue from the turn demand.

**refused_reason**: hexapod-mjx-train-4 code marker 65c5df87c4f76aa4dcba6346def31dd270a6f460-dirty != local HEAD 93547f569af82bf164fb1ff34495fbc108a14583. Sync first: snapshot.sh --sync hexapod-mjx-train-4 (and snapshot/commit before that if the tree is dirty).

