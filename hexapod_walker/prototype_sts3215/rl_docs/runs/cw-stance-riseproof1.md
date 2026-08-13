# cw-stance-riseproof1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: STALE_INTENT

**created**: 2026-08-10T14:59:04+00:00

**pod**: hexapod-mjx-train-3

**steps**: 18000000

**parent**: none (from scratch; stance-line recipe control)

**hypothesis**: Control probe pre-registered in cw-uni-rfix-warm1/fresh1 if-false: BOTH rfix arms FAILED the posture-strict gate (warm1 rise 0/6 lower 6/6; fresh1 rise 0/6 lower 0/6, worst end pad clearance 273-313mm -- height criterion satisfied by bridge/flail, not standing), implicating the WALK-env rise/lower task construction rather than pricing (fixed) or init (varied). This arm: the stance-line joint_goal recipe -- same fresh-init field standard (log_std 0, ent 0.005, DR 0.2), same hold=.1 rise=.45 lower=.45 mix, 15s episodes -- on the post-273ebde sim, where the stance champion still executes a clean feet-down rise (verified 08-10: det flat-start rise ends 5mm off target, worst pad clear 4mm). If-true (joint_goal fresh learns posture-strict rise/lower where walk-env fresh gamed it): walk-env task/obs construction is the blocker -- unified deliverable goes Stage-II reference-tracking distillation (rise ref + reward.k_rise_ref_track + reward.rise_posture_gate land with the next code snapshot) or two-policy blend. If-false (joint_goal from scratch also fails posture-strict on todays sim): post-shin-collision sim contact near ground is implicated -- escalate to a sim-contact dig-in before any more rise arms.

**gate**: posture-strict harness (default end-posture gate): rise AND lower success >=5/6 det each by 18M; VIDEO no leg-through-floor; early call permitted if W&B rise/lower success flat 0 at 6M

**verdict**: no verdict — never launched (INTENT stub from 08-10, 2 prior REFUSED attempts). Question (walk-env vs joint_goal rise/lower task construction for the unified deliverable) superseded: stance rise/lower solved via BC anchoring (footlow2-hard1 12/12 rise+lower), unified-policy line moved to the multitask track wave-1 (a2/b2/c2, verdicted). Closed as stale bookkeeping by an 08-13 idle-kick cycle; do not drain.

