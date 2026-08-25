# cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref-tuckexempt0-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: CANARY FAIL - MECHANISM

**created**: 2026-08-25T19:14:37+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref-tuckfloor0-s1

**wandb_id**: zigatklw

**hypothesis**: Seed twin of tuckexempt0 (train.bc_anchor_min_h_ahead_mm restored 0->8 + new train.bc_anchor_min_h_tuck_exempt_i0=1, gating the height-floor off only inside the mesh ref's tuck segment < ramp_i0): does the tuck-exempt fix work robustly across seeds, or was tuckexempt0 seed luck? Same joint-pair discipline as every mechanism hedge this campaign (meshref-s1, flatmix70-s1, tuckfloor0-s1). Judged jointly with tuckexempt0 per its own pre-registered gate.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. Same flat-pinned probe + standard DR-0 gate as tuckexempt0; joint pair read against tuckfloor0/-s1 (0/12 flat valid, 0/12 oc, duty=0 freeze; standard gate 0/6+0/6) and the meshref parent (det 5/6 + sto 4/6, oc 3/12, no freezes). PASS if BOTH seeds hit flat-probe det>=4/6 AND sto>=4/6 valid_plant (genuine duty>0 tuck motion, not a freeze) AND standard-gate non-flat kinds at-or-above the meshref parent -> promote to an 8M acquisition grid + port into stancemix. PARTIAL if flat-probe improves over tuckfloor0's 0/12 while non-flat holds >= tuckfloor0's own floor -> extend budget. FAIL if flat stays 0-1/12 valid with either the original 2.64A press-up or the same duty=0 freeze, or non-flat regresses below the meshref parent -> tuck-segment start curriculum next, not more anchor plumbing.

**verdict**: CANARY FAIL - MECHANISM: seed twin cross-verifies tuckexempt0's read exactly. Scoping the BC-anchor height-floor OFF only inside the mesh ref's tuck segment (<ramp_i0) reproduces tuckfloor0's total-freeze failure, not a genuine tuck. Flat-pinned probe (det+sto n=6+6, DR-0, run this cycle): 0/12 valid_plant, ALL 12 duty_cycle=0.0 on all six legs (zero swings), Imax only 0.53-0.54A, height_err_end 80-87mm -- byte-for-byte the tuckfloor0/-s1 and tuckexempt0 signature (sto h_err values bit-identical to seed-0's: policy output is effectively a no-op regardless of seed, so passive spawn dynamics dominate). Standard DR-0 gate also collapsed: det 0/6 + sto 0/6 + own-DR(0.2) det 0/6 + sto 0/6 (mix of the same duty=0 freezes on flat/rsi and 2.62-2.64A over_current pins on bridge/crouch/rsi) -- WORSE than the meshref parent's own 5/6+4/6, matching tuckfloor0's floor and triggering the gate's ALSO-FAIL clause on top of the primary flat-freeze FAIL. Reward declined (quarters 14.7/2.8/-12.8/-37.2), a genuine FAIL per 08-21 (bad eval + falling reward). MECHANISM (cross-verified with tuckexempt0's independent read, W&B 46px3v47): the mesh ref's own tuck segment holds height pinned near 0mm for ~245 ticks, so the state-aligned lookahead target sits at ~0mm too -- removing the floor (globally or tuck-scoped) leaves near-zero pursuit gradient there either way, so the policy settles into the reward-cheapest freeze instead of tucking. This closes the anchor-floor-scoping axis entirely (full removal AND exempt-scoped removal both freeze, both seeds, both times): the fix must reshape what the tuck-segment reference actually targets (real foot-sweep motion, not a static pin) or add a direct tuck-phase reward/curriculum -- not more anchor-floor plumbing. Next: tuck-segment start curriculum / ref-content reshaping, per the pair's own pre-registered FAIL branch. Evidence: logs/ckpt_eval/cw_standwalk_stance_mesh2_riseonly_bcchain3_meshref_tuckexempt0_s1_{gate,owncfg,flatprobe_det,flatprobe_sto}/, W&B zigatklw.

