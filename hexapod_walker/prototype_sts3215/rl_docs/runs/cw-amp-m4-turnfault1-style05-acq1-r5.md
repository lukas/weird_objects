# cw-amp-m4-turnfault1-style05-acq1-r5

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-23T02:07:03+00:00

**pod**: hexapod-mjx-train-4

**steps**: 6000000

**parent**: cw-amp-m4-turnfault1-style05

**hypothesis**: Plain English: turnfault1's 2M discovery stacked three brand-new axes at once (full-heading translation + turn-in-place + faults) and missed its own gait_valid>=10/12 safety bar (9/12, 3 sto leg-statues) while training reward was STILL RISING (46->203/quarter, never flat) -- per the 08-21 ruling that reads UNDERTRAINED, not a proven-broken combination, since push (pushacq1) and fault (faultobs2-headingsfull) each needed the SAME 3x acquisition step-up past their own 2M discovery reads before they composed cleanly. Single lever vs turnfault1-style05: continue from its own 2M checkpoint (--init-from-source) for 6M more steps, identical recipe (heading+turn+fault cfg, amp 0.5/0.5) unchanged, matching pushacq1's own acquisition dose exactly. (r5: r4 LAUNCH_CRASHED on a stale --obs-pad-transplant 18 inherited unchanged from the parent's graft-onto-new-substrate launch -- this respec's own source checkpoint already has the fault pad baked into its 93-dim obs, so continuing it needs pad=0, exactly like pushacq1's own continuation never re-pads; fixed via --arg, everything else identical, r1-r4 were tag/marker/config-crash retries, zero steps trained on any of them.)

**gate**: Acquisition (6M total, DR-0, own cfg). Mechanism-safety bar unchanged from the discovery arm: gait_valid >=10/12 det+sto (was 9/12), faulted episodes limp not statue on video, no crouch. If it clears: compare faulted-episode prog_ratio/slip against faultobs2-headingsfull-style05's own numbers (det 1.09/3.08, sto 0.57/7.26) and eval_yaw turn tracking against turnclone-yawcmd0-r2's own numbers (tip errs ~0.15-0.19) to see whether the 3-way combination now matches solo-axis quality at matched budget. If it STILL misses the safety bar with reward now flat: the combination is genuinely harder than either pairwise composition and needs the sequential (turn+push solid first, then graft fault) route instead of more raw budget on a fresh 3-way stack.

