# cw-amp-m4-pushfault1-noamp-r2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-23T02:09:26+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-amp-m4-pushfault1-noamp

**wandb_id**: lz6hmcvp

**hypothesis**: Plain English: retry of the dead-at-birth cw-amp-m4-pushfault1-noamp with its one config bug removed — this tests whether push recovery and fault tolerance compose WITHOUT turn in the mix, isolating whether turn is what makes two-axis composition hard. Same single lever vs cw-amp-m4-faultobs2-headingsfull-noamp (fault-sighted full-heading champion): add dr.ext_push_prob=1.0 (the proven 10-25N single-shove base dose). The original launch died fail-closed because it inherited --obs-pad-transplant 18 from the parent's own launch args, but the parent checkpoint ALREADY has the widened 92-dim obs (fault_health included), so obs widened by 0 and the trainer refused; this retry drops the transplant flag and changes nothing else. Prediction-if-true: DR-0 own-cfg gate (both hazards active) stays gait_valid >=10/12, added-push terminations <=3/6 det and <=4/6 sto (pushsmoke1-noamp-r4's first-2M shape), video shows compensation not statues. Prediction-if-false: terminations spike past the pushsmoke1 shape or gait_valid <9/12 with reward still rising — reads UNDERTRAINED per the 08-21 ruling, next lever is acquisition budget. Strongest alternative: the stacked per-episode randomization (one push AND one fault every episode) is a genuinely new difficulty class distinct from turn-composition.

**gate**: Discovery mechanism-safety bar (2M, DR-0, own cfg, dr.fault_prob=1.0 AND dr.ext_push_prob=1.0 both active): gait_valid >=9/12 det+sto (the faultobs2/turnfault1 floor), no crouch (height stays in the walking band), video confirms limps/stumbles-and-recovers not frozen statues. Compare numerically against the two solo-axis parents: faultobs2-headingsfull-noamp (gait_valid 11/12, 0 terminations, det prog 1.14/slip 3.08) and pushsmoke1-noamp-r4 (1/6 det + 3/6 sto topples at matched 2M). Clears with reward rising => acquisition continuation next (pushacq1's 6M dose). Misses floor with reward flat => push+fault is a genuinely harder joint skill needing sequential curriculum — informs whether the M5 fault gap is turn-specific or general-composition.

**verdict**: Push and fault compose essentially FOR FREE — the two-axis stack (100% single-shove 10-25N + 100% single-fault per episode) clears the discovery safety bar at 2M, so the composition penalty seen in turn+push/turn+fault is TURN-SPECIFIC, not a general two-axis effect. Evidence (own-cfg gate, both hazards active every episode, dr=0.0): gait_valid 11/12 (bar >=9/12; the one gv=False det episode carries its DISABLED leg 4 while walking on five — the honest limp the fault axis trains for, flagged correctly by the detector, not a trained pathology), terminations 2/6 det + 0/6 sto = 2/12, BETTER than solo-push pushsmoke1-noamp-r4's own matched-2M shape (1/6 det + 3/6 sto = 4/12) and inside the prediction-if-true bars (<=3/6 det, <=4/6 sto, gait >=10/12). Strips watched: det_0 clean six-leg cycling at full walking height (no crouch/statue), det_5 walks on five with the faulted leg carried, det_3 upright all episode then a genuine full-flip knockdown at end-frame. Reward steeply rising at cutoff (quarters 23/114/212/281). Per the gate's own PASS branch: acquisition continuation (6M, pushacq1's dose) launched this cycle as cw-amp-m4-pushfault1-noamp-acq1. M5 implication: the fault gap is turn-composition specific — the walk+push+fault triple (sans turn) is a viable M5 candidate substrate while the turn-composition arms grind.

