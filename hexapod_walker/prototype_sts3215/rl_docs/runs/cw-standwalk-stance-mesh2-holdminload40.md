# cw-standwalk-stance-mesh2-holdminload40

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-25T10:38:17+00:00

**pod**: hexapod-mjx-train-0

**steps**: 6000000

**parent**: cw-standwalk-stance-mesh2-holdterm40

**wandb_id**: jbnbyrdm

**hypothesis**: Plain English: does terminating hold episodes when a foot stays functionally UNLOADED (not just when the body sinks) finally get the mesh stance policy to keep all six feet planted, instead of learning to hover right at the height-drop line? holdterm40's own gate showed the exact predicted alternative cheat: every episode ends with height_err pinned at 40.0-40.3mm (right at the drop boundary) with hold_feet_factor flat 0.10-0.17 the whole 6M run -- a body-height proxy cannot see a foot that stays unloaded while the chassis itself stays inside the line. Single lever off holdterm40 (its scratch recipe + height-drop term untouched): safety.hold_min_load_terminate_s=1.0 + hold_min_load_terminate_n=0.3N + grace=1.0s terminates hold episodes whose WORST (min-over-feet) touch force stays below 0.3N for 1s straight (reason hold_min_load), bank-proved 4/4 green (test_hold_minload_* in test_task_semantics.py): the CURRENT live stack (height-drop already on) is blind to the scripted hover/hover1 crouch-park poses (they survive to truncation un-terminated), and the new lever catches both within a few seconds while leaving the honest quiet stand byte-identical. Prediction-if-true: hold panel >=10/12 valid six-foot plant by 6M, hold_feet_factor recovering and staying high, terminations/hold_min_load high early then falling. Prediction-if-false: the policy finds a THIRD basin the load floor also can't see (e.g. all feet loaded just above the 0.3N floor without real weight-bearing, or oscillating in/out of the floor faster than the 1s sustain) -- next lever would be a stricter floor/longer sustain dose, or a DR/entropy axis on top of this mechanism.

**gate**: Hold panel at 6M: pod_eval hold DR-0 det+sto n=6+6. PASS: >=10/12 survive 15s with zero over_current/tilt/hold_low_height/hold_min_load terminations AND det episodes show six-foot stance (valid_plant true, or all-leg duty>=0.9 with end_clear<5mm) AND cur_p95<=1.0A; own-DR(0.2) read alongside.

**verdict**: Rung-6 min-load termination lever does NOT produce an honest six-foot plant. Evidence: DR-0 det gate 0/6 valid_plant, all 6 episodes end TERM hold_low_height at height_err_end_mm pinned 40.0-40.4mm (right at the drop line) with cur_max_a pinned 2.64A (both height AND current constraints binding simultaneously) -- the SAME chassis-hover-at-the-boundary signature as rung-5's holdterm40, now also crossing the term line by episode end instead of surviving just inside it. Own-DR(0.2): 0/12, sto mode shows the new hold_min_load reason firing 3/6 times (mechanism verified live) plus 3/6 tilt_roll real falls -- min-load correctly catches the previously-invisible unloaded-foot cheat but the policy has nowhere honest to go once caught. Training reward WORSENED every quarter (-97.8/-223.6/-347.6/-355.1), never recovering -- genuine FAIL per the 08-21 ruling (bank-proven, aligned mechanism; task metric flat-zero the whole 6M run). Video: crouched crab-like stance, legs bent under a lowered chassis, symmetric but never a level six-foot plant. Rung-6 mechanism lever CLOSED; joint read with seed1 below.

