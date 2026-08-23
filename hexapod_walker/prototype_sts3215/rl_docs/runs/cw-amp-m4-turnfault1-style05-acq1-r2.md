# cw-amp-m4-turnfault1-style05-acq1-r2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-23T01:50:13+00:00

**pod**: hexapod-mjx-train-4

**steps**: 6000000

**parent**: cw-amp-m4-turnfault1-style05

**hypothesis**: Plain English: turnfault1's 2M discovery stacked three brand-new axes at once (full-heading translation + turn-in-place + faults) and missed its own gait_valid>=10/12 safety bar (9/12, 3 sto leg-statues) while training reward was STILL RISING (46->203/quarter, never flat) -- per the 08-21 ruling that reads UNDERTRAINED, not a proven-broken combination, since push (pushacq1) and fault (faultobs2-headingsfull) each needed the SAME 3x acquisition step-up past their own 2M discovery reads before they composed cleanly. Single lever vs turnfault1-style05: continue from its own 2M checkpoint (--init-from-source) for 6M more steps, identical recipe (heading+turn+fault cfg, amp 0.5/0.5) unchanged, matching pushacq1's own acquisition dose exactly. (r2: an earlier same-named attempt this cycle window was REFUSED before training started -- renamed, retrying, same recipe.)

**gate**: Acquisition (6M total, DR-0, own cfg). Mechanism-safety bar unchanged from the discovery arm: gait_valid >=10/12 det+sto (was 9/12), faulted episodes limp not statue on video, no crouch. If it clears: compare faulted-episode prog_ratio/slip against faultobs2-headingsfull-style05's own numbers (det 1.09/3.08, sto 0.57/7.26) and eval_yaw turn tracking against turnclone-yawcmd0-r2's own numbers (tip errs ~0.15-0.19) to see whether the 3-way combination now matches solo-axis quality at matched budget. If it STILL misses the safety bar with reward now flat: the combination is genuinely harder than either pairwise composition and needs the sequential (turn+push solid first, then graft fault) route instead of more raw budget on a fresh 3-way stack.

**refused_reason**: hexapod-mjx-train-4 code marker e97d29f45f826f8e15863508eaf8fb3a9842fcc0 != local HEAD 4ca9ba8813acc6ed8b942f5a420c8eca39560c6b. Sync first: snapshot.sh --sync hexapod-mjx-train-4 (and snapshot/commit before that if the tree is dirty).

