# cw-amp-m4-pushfault1-noamp

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-23T01:58:11+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-amp-m4-faultobs2-headingsfull-noamp

**hypothesis**: Plain English: M3 (push recovery) and M4 (fault tolerance) have each been solved SEPARATELY on the full-heading substrate, and turn+push / turn+fault composition both landed 3x worse than solo-axis at matched budget -- but nobody has tested push+fault together WITHOUT turn in the mix, which isolates whether turn specifically is what makes composition hard or whether ANY two-axis stack degrades. Single lever vs cw-amp-m4-faultobs2-headingsfull-noamp (the PASSed fault-sighted full-heading champion, gait_valid 11/12, zero terminations): add dr.ext_push_prob=1.0 (the exact single-shove 10-25N base dose pushsmoke1/pushacq1 already proved safe on the non-fault substrate). Same init checkpoint, same fault wiring (dr.fault_prob=1.0, obs.fault_health=1, obs already padded so no further transplant needed), 2M discovery. Prediction-if-true (composes for free, like push-onto-plain-heading did): DR-0 own-cfg gate (both dr.fault_prob=1.0 and dr.ext_push_prob=1.0 sampled) stays gait_valid >=10/12, det+sto terminations from the added push stay <=3/6 det and <=4/6 sto (matching pushsmoke1-noamp-r4's own first-2M shape), faulted+pushed episodes still show compensation not statue on video. Prediction-if-false (degrades like the turn compositions): terminations spike well past the pushsmoke1 shape or gait_valid drops below 9/12 with reward still rising -- reads UNDERTRAINED per the 08-21 ruling, next lever is acquisition budget, not a broken combination. Strongest alternative: push and fault interact fine on their own but the STACKED randomization (one push AND one fault per episode, both active every eval episode here) is harder than either gate's own single-axis eval, which only samples one hazard at a time -- a genuinely new difficulty class distinct from turn-composition.

**gate**: Discovery mechanism-safety bar (2M, DR-0, own cfg, dr.fault_prob=1.0 AND dr.ext_push_prob=1.0 both active): gait_valid >=9/12 det+sto (the faultobs2/turnfault1 floor), no crouch (height stays in the walking band), video confirms limps/stumbles-and-recovers not frozen statues. Compare numerically against the two solo-axis parents: faultobs2-headingsfull-noamp (gait_valid 11/12, 0 terminations, det prog 1.14/slip 3.08) and pushsmoke1-noamp-r4 (1/6 det + 3/6 sto topples at matched 2M). If it clears with reward still rising: acquisition-budget continuation is next (matching pushacq1's 6M dose). If it misses the floor with reward flat: push+fault is a genuine harder joint skill needing sequential curriculum, same conclusion class as turnpush1/turnfault1 but for a different axis pair -- informs whether the M5 fault gap is turn-specific or general-composition.

