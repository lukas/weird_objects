# cw-amp-m4-pushfault1-noamp-r2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-23T02:09:26+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-amp-m4-pushfault1-noamp

**hypothesis**: Plain English: retry of the dead-at-birth cw-amp-m4-pushfault1-noamp with its one config bug removed — this tests whether push recovery and fault tolerance compose WITHOUT turn in the mix, isolating whether turn is what makes two-axis composition hard. Same single lever vs cw-amp-m4-faultobs2-headingsfull-noamp (fault-sighted full-heading champion): add dr.ext_push_prob=1.0 (the proven 10-25N single-shove base dose). The original launch died fail-closed because it inherited --obs-pad-transplant 18 from the parent's own launch args, but the parent checkpoint ALREADY has the widened 92-dim obs (fault_health included), so obs widened by 0 and the trainer refused; this retry drops the transplant flag and changes nothing else. Prediction-if-true: DR-0 own-cfg gate (both hazards active) stays gait_valid >=10/12, added-push terminations <=3/6 det and <=4/6 sto (pushsmoke1-noamp-r4's first-2M shape), video shows compensation not statues. Prediction-if-false: terminations spike past the pushsmoke1 shape or gait_valid <9/12 with reward still rising — reads UNDERTRAINED per the 08-21 ruling, next lever is acquisition budget. Strongest alternative: the stacked per-episode randomization (one push AND one fault every episode) is a genuinely new difficulty class distinct from turn-composition.

**gate**: Discovery mechanism-safety bar (2M, DR-0, own cfg, dr.fault_prob=1.0 AND dr.ext_push_prob=1.0 both active): gait_valid >=9/12 det+sto (the faultobs2/turnfault1 floor), no crouch (height stays in the walking band), video confirms limps/stumbles-and-recovers not frozen statues. Compare numerically against the two solo-axis parents: faultobs2-headingsfull-noamp (gait_valid 11/12, 0 terminations, det prog 1.14/slip 3.08) and pushsmoke1-noamp-r4 (1/6 det + 3/6 sto topples at matched 2M). Clears with reward rising => acquisition continuation next (pushacq1's 6M dose). Misses floor with reward flat => push+fault is a genuinely harder joint skill needing sequential curriculum — informs whether the M5 fault gap is turn-specific or general-composition.

