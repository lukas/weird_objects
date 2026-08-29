# cw-standwalk-unified1-mix-long-s1-cont1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-28T13:54:04+00:00

**pod**: hexapod-mjx-train-5

**steps**: 16000000

**parent**: cw-standwalk-unified1-mix-long-s1

**wandb_id**: k1xs1fjk

**hypothesis**: Plain English: seed-1 twin of cw-standwalk-unified1-mix-long-s0-cont1 -- the 60s/mode_seq_max_segments=7 unified command-following recipe just PASSED its session-health gate (0 falls, complete_frac 1.0) on this seed too, but absolute direction/slip tracking (dir_err~62deg, slip~14/m) is still far from the joystick command band (dir_err<=40, slip<=2.9); reward rose monotonically through 16M with no plateau -- does another 16M steps (32M total) from this exact checkpoint keep closing that gap under the 08-21 continue ruling on the previously-catastrophe-prone rescued seed, or does it plateau/relapse?

**gate**: At 32M (paired same-seed continuation vs this 16M checkpoint's own session_verdict.json numbers): PASS if dir_err_med AND slip_per_m_med both improve materially (>=15%) with session terminations/completion staying >= the 16M level (0 terms, complete_frac 1.0) and gait_valid/sac not regressing (no relapse into the seed's known leg-sacrifice failure mode); PARTIAL if only one axis improves materially; FAIL if reward flat/down, or dir_err/slip regress/plateau (<5% delta) or terminations reappear -- signals a mechanism/reward ceiling (or a seed relapse), not a budget gap, and should redirect to a command-tracking reward audit instead of a further identical-recipe continuation.

**verdict**: FAIL vs its own pre-registered continuation gate (paired against this seed's own 16M mixedsession baseline: dir_err 62.1deg, slip/m 14.0, 0 terminations, complete_frac 1.0, gait_valid_frac 1.0 sac=[]). Evidence (32M mixedsession, 90-episode dr0+owndr+dr0_long panel, session_verdict.json): slip/m materially improved (14.0->10.47, -25%, clears the >=15% bar) but dir_err did NOT improve (62.1->64.15deg, +3.3%, a plateau not a gain) and -- the gate's explicit disqualifier -- terminations REAPPEARED: 4/90 (all over_current, in walk(2)/rise(2) segments, previously 0/90) with gait_valid_frac dropping 1.0->0.967 and two legs newly sacrificed (sac=[0,2], previously []) -- exactly 'the seed's known leg-sacrifice failure mode' the gate named as the relapse risk for this ('previously catastrophe-prone rescued') seed. DR-0/own-DR isolated-mode gate+owncfg reports stay clean (walk gait_valid 6/6 sac=[] 0 terms both DR; only known-benign hold_min_load terms on hold, unchanged from baseline) -- the relapse is a SEQUENCED-SESSION-SPECIFIC finding (mode transitions/segment starts under mode_seq), invisible to the isolated per-mode DR-0 panel, which is exactly why the mixedsession instrument exists. Reward kept rising monotonically the whole 16M (quarters -44.2/534.8/1418.9/2278.7, no plateau) so this is not a stalled-optimizer story. Per the gate's own prescribed next step: this is a mechanism/reward-ceiling-or-seed-relapse signal, not a budget gap -- do not fund a further identical-recipe continuation on this seed; the open lever is a command-tracking reward audit (why slip improves while dir_err and segment-transition robustness do not, and why more budget reopened an over_current failure mode the 16M checkpoint had closed). Sibling long-s0-cont1 not yet read (own-DR mixedsession still computing) -- this is a self-contained verdict against this seed's own named gate, not a joint call.

