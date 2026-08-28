# cw-standwalk-unified1-mix-long-s0-cont1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-28T13:48:41+00:00

**pod**: hexapod-mjx-train-4

**steps**: 16000000

**parent**: cw-standwalk-unified1-mix-long-s0

**wandb_id**: 4eqojadc

**hypothesis**: Plain English: the 60s/mode_seq_max_segments=7 unified command-following recipe just PASSED its session-health gate (0 falls, complete_frac clean) on both seeds, but absolute direction/slip tracking (dir_err~65deg, slip~9/m) is still far from the joystick command band (dir_err<=40, slip<=2.9); reward rose monotonically through 16M with no plateau and dir_err/slip both improved 5-9deg/2-3x since the 8M parent -- does another 16M steps (32M total) from this exact checkpoint keep closing that gap under the 08-21 continue ruling, or does it plateau (pointing to a reward/eval alignment fix instead of more budget)?

**gate**: At 32M (paired same-seed continuation vs this 16M checkpoint's own session_verdict.json numbers): PASS if dir_err_med AND slip_per_m_med both improve materially (>=15%) with session terminations/completion staying >= the 16M level and gait_valid/sac not regressing; PARTIAL if only one of dir_err/slip improves materially or improvement is <15%; FAIL if reward is flat/down, or dir_err/slip regress/plateau (<5% delta either direction) while terminations do not improve -- signals a genuine mechanism/reward-shape ceiling, not a budget one, and should redirect to a command-tracking reward audit (e.g. anchor/course-tracking pricing) instead of a further budget continuation on this identical recipe.

