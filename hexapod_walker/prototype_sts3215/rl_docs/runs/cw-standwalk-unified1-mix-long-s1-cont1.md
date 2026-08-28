# cw-standwalk-unified1-mix-long-s1-cont1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-28T13:54:04+00:00

**pod**: hexapod-mjx-train-5

**steps**: 16000000

**parent**: cw-standwalk-unified1-mix-long-s1

**wandb_id**: k1xs1fjk

**hypothesis**: Plain English: seed-1 twin of cw-standwalk-unified1-mix-long-s0-cont1 -- the 60s/mode_seq_max_segments=7 unified command-following recipe just PASSED its session-health gate (0 falls, complete_frac 1.0) on this seed too, but absolute direction/slip tracking (dir_err~62deg, slip~14/m) is still far from the joystick command band (dir_err<=40, slip<=2.9); reward rose monotonically through 16M with no plateau -- does another 16M steps (32M total) from this exact checkpoint keep closing that gap under the 08-21 continue ruling on the previously-catastrophe-prone rescued seed, or does it plateau/relapse?

**gate**: At 32M (paired same-seed continuation vs this 16M checkpoint's own session_verdict.json numbers): PASS if dir_err_med AND slip_per_m_med both improve materially (>=15%) with session terminations/completion staying >= the 16M level (0 terms, complete_frac 1.0) and gait_valid/sac not regressing (no relapse into the seed's known leg-sacrifice failure mode); PARTIAL if only one axis improves materially; FAIL if reward flat/down, or dir_err/slip regress/plateau (<5% delta) or terminations reappear -- signals a mechanism/reward ceiling (or a seed relapse), not a budget gap, and should redirect to a command-tracking reward audit instead of a further identical-recipe continuation.

