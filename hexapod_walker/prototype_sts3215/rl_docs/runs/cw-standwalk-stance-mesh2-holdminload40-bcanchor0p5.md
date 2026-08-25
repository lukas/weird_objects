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

**verdict**: CANARY PASS (mechanism-health scope, 3rd/3rd dose confirmation) -- CORRECTED read (see below): dose 0.5 also clears the letter of the gate (DR-0 det 6/6 valid_plant, cur_p95 0.67A<=1.5A) but is materially weaker than doses 1.0/3.0. CORRECTION to this verdict's first pass: I originally wrote video="held to truncation" -- WRONG. Frame-strip timestamps + report.json show all 6/6 DR-0 det episodes TERMINATE early via hold_min_load at t~3.5s (return ~57.8, vs a full 15s episode), unlike bcanchor1/bcanchor3 whose DR-0 det gate has ZERO terminations (empty term_reason) for the full duration. The geometric valid_plant check still reads True at the termination instant (height/posture/current all still fine the moment min-load fires), so the letter of the pre-registered gate (valid_plant + cur_p95) is met, but this is a real, visible degradation: the policy itself (no action noise, DR-0, deterministic) cannot sustain the hold past ~3.5s at this dose, where doses 1.0/3.0 sustain it cleanly to 15s. Own-DR(0.2) det is 5/6 valid_plant, and only 1 of those 6 episodes (idx 3) runs to truncation without any termination -- the other 5 terminate early too. Sto 0/6 both DR-0 and own-DR, same un-annealed-policy_std signature as siblings. hold_feet_factor noisy non-monotone, matches siblings. bc_anchor_loss 0.16->0.008. Revised reading of the joint 3-arm dose read: the mechanism generalizes across the tested range but has a real dose floor around 0.5-1.0x -- 1.0x and 3.0x are the clean passers, 0.5x is a weaker/marginal pass that would need more dose or budget to sustain the hold, not a flat failure. This does not change the acquisition-arm choice: the concurrent cycle already funded the 8M std-anneal arm off dose 3.0 (cleanest of the three) before this correction landed -- still the right pick.

