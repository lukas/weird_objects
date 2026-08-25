# cw-standwalk-stance-mesh2-holdminload40-bcanchor0p5

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-25T12:19:04+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-holdminload40

**wandb_id**: vggrigsq

**hypothesis**: Rung-7 dose floor sibling of bcanchor1/bcanchor3: completes a 3-point dose curve (0.5/1.0/3.0) for the joint-space imitation anchor in one wave rather than serializing across cycles (operator 08-22 batching guidance). Same single-lever-off-holdminload40 base.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. Same as bcanchor1/3: 2M canary, hold DR-0 det+sto n=6+6, >=4/6 det valid_plant + cur_p95<=1.5A + rising hold_feet_factor/hold_load_factor = PASS; 1-3/6 = PARTIAL/continue; 0/6 pinned-signature = FAIL. Joint 3-arm dose read decides which dose (if any) funds an 8M acquisition arm.

**verdict**: CANARY FAIL - MECHANISM (below the pre-registered PASS bar, but a genuinely NEW and informative signature, not a repeat of rung5/6's 40mm/2.64A collapse). At coef=0.5, det gate is 0/6 by the letter of the metric, but the underlying pose is nearly honest: the reported final-frame state has height_err_end_mm=1.2, cur_max_a=1.26A, end_clear<=1mm on all six feet, valid_plant=True, plant_fail=[] -- yet term_reason=hold_min_load fires anyway because leg 0 fidgets (duty_cycle[0]=0.29, swing_count[0]=11 in a single ~15s episode) enough to trip the sustained-low-load EMA even though it re-plants between lifts. This is a DOSE-FLOOR finding: 0.5x anchor weight is not enough to fully suppress one leg's residual fidget, while 1.0x and 3.0x (siblings bcanchor1/bcanchor3, both verdicted CANARY PASS this cycle) suppress it completely (6/6 det, zero terminations, clean video). sto mode fails the same way as the other two doses (0/6, current-pinned 2.64A). No separate continuation needed: the two higher doses already supersede this one outright, and the std-anneal follow-up (bcanchor3-stdanneal) is built on the winning dose.

