# cw-standwalk-stance-mesh2-riseonly-bcchain3-tuckrise45-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: CANARY FAIL - MECHANISM

**created**: 2026-08-25T20:02:18+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-riseonly-bcchain3-tuckrise45

**wandb_id**: wbhsu4st

**hypothesis**: Seed twin of tuckrise45 (only --seed 1->1 differs, same 45mm tuck-rise ref lever): does the ref-content height-shaping fix work robustly across seeds, or was tuckrise45 seed luck? Same joint-pair discipline as every mechanism hedge this campaign (meshref-s1, flatmix70-s1, tuckfloor0-s1, tuckexempt0-s1). Judged jointly with tuckrise45 per its own pre-registered gate.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. Same as tuckrise45: MECHANISM-HEALTH CANARY ONLY. Joint pair read: PASS if BOTH seeds hit flat-probe det>=4/6 AND sto>=4/6 valid_plant with genuine duty>0 tuck motion AND standard-gate non-flat kinds at-or-above the meshref parent (5/6+4/6) -> promote 8M + port into stancemix. PARTIAL if flat-probe shows any genuine duty>0 tuck motion while non-flat holds >= meshref parent -> extend/dose. FAIL if flat stays 0-1/12 valid (2.64A pin or duty=0 freeze) or non-flat regresses below meshref parent -> ref-content height-shaping refuted, next is a direct tuck-phase reward/curriculum term.

**verdict**: CANARY FAIL - MECHANISM (killed early, refuted by construction before completion): launched before reading the concurrent cycle's STATUS.md update (19:5x), which had ALREADY probed --tuck-rise-mm at 3 controlled CPU doses (15/20/45mm, no GPU spent) and found that on the 3.5kg mesh model NO feasible tuck-segment content places the ACHIEVED chassis height inside the tuck: swing-lift fights the rise, then a ~25-30mm compliance dead-band, then a ~1.5s settling lag -- achieved height always lands in the press segment regardless of dose/split. This run's own ref (rise_ref_mesh_tuckrise45.npz) confirms it directly: h_rel_m stays 0.0 through tick ~245 and only starts climbing at tick ~250-258 (achieved ramp_i0=258, barely later than meshref's own 245) -- the '45mm across the second half of the tuck' never actually lands IN the tuck, it lands at the tuck/press boundary, so the BC-anchor floor sees exactly the same height-flat approach it always did. Killed both seeds (train-0/train-1) at ~1.0-1.6M/2M steps to stop wasted GPU spend rather than let a doomed canary run to completion -- ref-content height-shaping is refuted by the same mechanism as tuckexempt, not a fresh open question. Per the concurrent cycle's own next-step call: the only surviving design is changing the anchor floor's PROGRESS METRIC itself (script-index or commanded-height keyed, not achieved-height keyed) -- that is bc_anchor hot-path code + bank rows + a canary, correctly DIG-IN flagged for the deep-model cycle rather than rushed here. No further ref-content-height-shaping arm should be funded on this axis.

