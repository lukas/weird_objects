# cw-dep-tall-rsi1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-11T22:24:16+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-dep-tall-gate1

**wandb_id**: kgt0atfv

**hypothesis**: TALL LADDER T6: RSI-for-walk (mid-stride tall spawns, code landed 08-11 eve: goal.walk_gait_start_frac, tests green). PRICING FAMILY CLOSED by probe_tall_wall: five arms (ref ladder, income gate sigma30, gate+6M budget, k_height 3x and 10x, speed-band relief) ALL leave the mid-gait body at -72..-75mm with leg yaw pinned at its 35deg limit - kh10 pays ~5.6/tick (more than its walk income) rather than stand up, so the optimizer cannot FIND the taller basin, it is not underpaying for it. Episodes already spawn tall STANDING; the missing states are tall mid-stride WALKING. This arm: warm from cw-dep-tall-gate1 (gate keeps tall income-favored so RSI states, once visited, are worth keeping), 50% of walk episodes spawn mid-stride in the scripted tripod gaits tall pose at the episodes own command, live from tick 0 (0.3s ramp, no hold). Same shape as the rise RSI that closed the rise exploration gap.

**gate**: Primary: probe_tall_wall.py steady-state walking height (all parents -72..-75mm). PASS: >= -55mm mid-gait with walk retained (speed >=0.028, survived 1, slip <=1.8, no park) - first arm to move the needle wins the campaign lead; next rung raises gait_start_frac ref upward. INFORMATIVE FAIL: height still -72..-75 = even direct state injection cannot hold tall walking under this income; then the wall is dynamic stability itself and the next lever is physics easing or a taller scripted reference gait. Check yaw: if spawned-tall episodes end splayed at the 35deg wall, watch the collapse trajectory in video.

