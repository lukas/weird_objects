# cw-amp-m2-min5-noamp

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-22T18:53:23+00:00

**pod**: hexapod-mjx-train-4

**steps**: 2000000

**parent**: cw-amp-m2-pilot-noamp

**hypothesis**: Plain English: does the AMP brief's own literal section-5 minimal reward (weak Gaussian velocity/yaw/upright/height task kernel + modest existing action/current/gyro regularizers, dropping the whole SLIPWALK anti-slip/anchor/idle-charge/drag-stance/gait-gate apparatus every one of the 9 prior from-scratch M2 arms reused) let a from-scratch policy actually walk? This is the pure-task control: task reward ONLY (no AMP), reusing the ORIGINAL pre-freeprog pilot's plain K_WALK Gaussian-velocity kernel + linear progress + base tilt/height kernel (env.py's shared r_task, already = brief 5.1's upright+height terms verbatim) and default-modest action/action-delta/current/gyro regularizers (section 5.3, already the shared defaults, untouched) -- but fixes the TWO confounds the original pilot-noamp/-style05 pair had: goal.walk_pure=1.0 (pure walk-only diet -- the M2-c1 dig-in found the mixed goal-mix let a frozen half-tripod collect rise_finish/posture/height income from OTHER modes) and reward.term_penalty=400 (the already-validated anti-suicide fix from the freeprog-term400 lineage). Envelope matches the freeprog family exactly (speed 0-0.25 m/s, yaw +/-0.5 rad/s) so this is comparable head-to-head against all 9 prior SLIPWALK-stack FAILs on the identical envelope -- the ONE variable is the reward architecture itself.

**gate**: Discovery (2M steps, judged on det video (3+ episodes) + gait_valid/fwd-travel harness numbers, NOT the joystick DONE gate). INFORMATIVE-PASS = det video shows real net forward travel (>=0.10 m/15s, the freeprog family's own bar) with visibly cyclic multi-leg contact/swing (in-place cyclic tripod motion with near-zero net travel also counts as INFORMATIVE, must be named). FAIL-same-signature = the identical frozen/half-tripod statue (gait_valid 0/6, 1-3 legs sacrificed, fwd <0.03m) despite dropping the entire SLIPWALK stack -- this would show the base shared r_task/height kernel ALONE is enough free income to freeze regardless of the anti-slip apparatus, and redirects the next arm to shrinking reward.k_track/k_height for this task specifically (a new, narrower lever) rather than the SLIPWALK-vs-minimal question. FAIL-different-signature (catastrophic toppling, the freeprog-fix-pair fingerprint) = term_penalty=400 did not transfer cleanly onto walk_pure=1, check ep_len_mean trend first. Read jointly with its 3 AMP-weighted siblings (min5-styleA/B/C, brief 5.2's A/B/C sweep) launched in the same batch.

**refused_reason**: hexapod-mjx-train-4 code marker aa0cf7009d4d824339f8c5b20f018c79bafbe76b != local HEAD b4e5227fcb298aec98619a80896add7471448755. Sync first: snapshot.sh --sync hexapod-mjx-train-4 (and snapshot/commit before that if the tree is dirty).

