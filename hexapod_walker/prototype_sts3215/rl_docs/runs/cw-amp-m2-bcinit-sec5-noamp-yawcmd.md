# cw-amp-m2-bcinit-sec5-noamp-yawcmd

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-22T22:38:51+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m2-bcinit-sec5-noamp-headingsfull

**hypothesis**: Plain English: task-only twin of cw-amp-m2-bcinit-sec5-style05-yawcmd (concurrently in flight, own cycle) -- can the noamp BC-init walker also learn to TURN on command (yaw-rate +/-0.3 rad/s) with the full bank-verified turn pricing set, or is the AMP style channel's real turn-in-place clips (teacher_v2) specifically necessary to organize turning motion the way it was NOT necessary for heading/direction generalization (8/8 PASS with or without style)? Continues from noamp-headingsfull (--init-from-source), identical single-lever addition to the style05 sibling: obs 73->74 via --obs-pad-transplant 1, full turn pricing set (k_walk_yaw/walk_yaw_kernel_gate/k_yaw_prog/k_yaw_still/walk_kernel_yaw_gate/walk_yaw_hold_prog_gate/yaw_still_avg_s), no AMP flags. Prediction-if-true (matches style05 whichever way it lands): turning either emerges or stays command-invariant on BOTH arms equally -- turning is a task-reward/architecture question, not a style question (consistent with the whole heading curriculum's finding). Prediction-if-false: style05 turns but noamp doesn't (or vice versa) -- the FIRST real functional style fork in this whole lineage, on exactly the axis (genuine turning motion) the AMP brief predicts style should matter most for.

**gate**: Discovery continuation (2M, DR-0, judged jointly with the style05-yawcmd sibling once both land). INFORMATIVE-PASS = gait_valid stays >=5/6 det+sto, translation dir_err does not degrade >15deg vs noamp-headingsfull's ~33/50deg. Read wz tracking (|wz| sign following wz_ref, wz_err vs the command-invariant ~0.24 fingerprint from the old walking-champion turn failures) AGAINST the style05 sibling: MATCH (both turn or both don't) = turning is style-independent; FORK = first real functional style benefit in the lineage. FAIL-collapse = terminations/sacrificed legs/statue from the turn demand.

**refused_reason**: hexapod-mjx-train-0 code marker c89cb77060460dfb163fd1c0e75fe90f4d6f9d79 != local HEAD 732566a42bc2cb62020600efe28949ef79354d14. Sync first: snapshot.sh --sync hexapod-mjx-train-0 (and snapshot/commit before that if the tree is dirty).

