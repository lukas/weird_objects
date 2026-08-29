# cw-standwalk-unified1-mix-long-s0-cont1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-28T13:48:41+00:00

**pod**: hexapod-mjx-train-4

**steps**: 16000000

**parent**: cw-standwalk-unified1-mix-long-s0

**wandb_id**: 4eqojadc

**hypothesis**: Plain English: the 60s/mode_seq_max_segments=7 unified command-following recipe just PASSED its session-health gate (0 falls, complete_frac clean) on both seeds, but absolute direction/slip tracking (dir_err~65deg, slip~9/m) is still far from the joystick command band (dir_err<=40, slip<=2.9); reward rose monotonically through 16M with no plateau and dir_err/slip both improved 5-9deg/2-3x since the 8M parent -- does another 16M steps (32M total) from this exact checkpoint keep closing that gap under the 08-21 continue ruling, or does it plateau (pointing to a reward/eval alignment fix instead of more budget)?

**gate**: At 32M (paired same-seed continuation vs this 16M checkpoint's own session_verdict.json numbers): PASS if dir_err_med AND slip_per_m_med both improve materially (>=15%) with session terminations/completion staying >= the 16M level and gait_valid/sac not regressing; PARTIAL if only one of dir_err/slip improves materially or improvement is <15%; FAIL if reward is flat/down, or dir_err/slip regress/plateau (<5% delta either direction) while terminations do not improve -- signals a genuine mechanism/reward-shape ceiling, not a budget one, and should redirect to a command-tracking reward audit (e.g. anchor/course-tracking pricing) instead of a further budget continuation on this identical recipe.

**verdict**: CANARY FAIL vs its own pre-registered continuation gate, matching the sibling long-s1-cont1's already-recorded FAIL -- JOINT read confirms budget-continuation (16M->32M, same recipe, no reward-shape change) is not the fix for command-tracking. Baseline (this seed's own 16M mixedsession, unified1-mix-long-s0 wave-3 table): terms 2/90, complete_frac 0.978, dir_err_med 64.6deg, slip/m_med 9.17, prog_med 0.131, gait_valid_frac 0.967, sac=[2]. This run (32M, session_verdict.json, 90-episode dr0+owndr+dr0_long panel): terms 5/90 (all over_current, walk(3)+rise(2) segments -- NEW, previously 2), complete_frac 0.944 (down), dir_err_med 62.65deg (+3.0%, a plateau under the gate's own <5%-delta bar), slip/m_med 8.283 (+9.67%, real but under the 15% material-improvement bar), gait_valid_frac 0.95 (down), sac=[0,2] (leg 0 newly appears, was clean [2] before). Neither target metric clears the >=15% PASS bar (both PARTIAL-range at best), AND terminations/completion/gait_valid/sac all regressed rather than holding at-or-above the 16M level -- worse than a plain plateau. Training reward itself is healthy and rising throughout (quarters -11.5/631.7/1412.5/2210.8, ep_rew_mean 2772 at 16.06M steps-this-run), so this is not an optimizer failure -- it is exactly the gate's own predicted mechanism/reward-shape ceiling: more budget on the identical recipe does not move command-tracking and mildly costs stance-termination robustness doing it. Consistent with (and superseded by) the course-tracking root-cause work already in flight on other lineages (k_walk_course was found inert/EMA-cancelled, k_walk_course_disp built as the fix, now itself being read via the coursedisp-w015/w035 sub-stride-window siblings) -- no further same-recipe budget continuation should be funded on this lineage. Evidence: logs/ckpt_eval/cw_standwalk_unified1_mix_long_s0_cont1_{gate,owncfg,mixedsession}/session_verdict.json, W&B 4eqojadc.

