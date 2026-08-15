# cw-mt-b1-dualgru1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-15T18:05:16+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-mt-b1

**hypothesis**: Fix the wave-1 narrow-generalist's yaw/acquisition shortfall with an architecture change instead of another reward/coefficient sweep: does giving the multitask policy two mode-gated GRU cores (one for walking, one for standing-still), routed by the LIVE commanded mode rather than a fixed episode label, let it actually learn the b1/b2 command mix (walk 0-0.06 m/s, occasional +-0.15 rad/s yaw, 40% stop segments) instead of topping out with unreliable yaw and a weak leg? This is the arch track's own dual-core fix (already proven to remove shared-trunk cross-mode interference in cw-arch-gru-dual1/2) transplanted onto multitask's own recipe, unblocked now that the routing bug (obs.mode_onehot was episode-constant 'walk' for every tick of this always-walking-task recipe, so the dual-core gate never engaged) is fixed by the new obs.mode_onehot_cmd (derives the one-hot from the live blended command instead; default-off, bit-exact, tests/test_mode_onehot.py 18/18 green). Fresh init, same as every wave-1 arm (fresh init IS multitask's hypothesis) -- ONE lever vs cw-mt-b1: architecture (DualGruActorCriticPolicy + live command-routed mode_onehot) in place of the plain MLP. Prediction-if-true: by 2M, det video shows a real six-leg gait (not the wave-1 0/6 paddle) with early positive yaw separation (arc-left vs arc-right differ) and no march-in-place on stop segments -- a qualitatively healthier early signal than b1's own 2M read (partial gait, sto gait_valid 3/6). Prediction-if-false: same 0/6-or-paddle signature as b1 at 2M -- the routing fix alone doesn't help at this budget/architecture, and mode-gating is not the lever for multitask's acquisition shortfall (falls back to the other withdrawn-pause candidates: command-width curriculum or reward-geometry diagnosis).

**gate**: 2M discovery, video-first vs cw-mt-b1's own 2M read (own-DR0.2 det+sto): PASS(promising) if det walk gait_valid >= 3/6 (beats b1's 0/6) AND no leg pinned <0.10 duty in every episode AND stop segments hold still -- continue to the 20M budget as cw-mt-b2's dual-core successor. FAIL(no-benefit) if walk still 0/6 gait_valid or an identical paddle/park signature to b1 -- close the architecture-transplant option, the acquisition shortfall isn't a shared-trunk routing problem here. FAIL(bug) if mode_onehot never actually flips between hold/walk during a walk episode (routing not engaging) -- code bug, not a science verdict, fix and rerun once.

**refused_reason**: hexapod-mjx-train-1 code marker 568daaa18d9a8a3161ccce2860da5fdf0bb2cac2 != local HEAD 46b9c7bb2de19d034cb6baff660f48d38f7e365f. Sync first: snapshot.sh --sync hexapod-mjx-train-1 (and snapshot/commit before that if the tree is dirty).

