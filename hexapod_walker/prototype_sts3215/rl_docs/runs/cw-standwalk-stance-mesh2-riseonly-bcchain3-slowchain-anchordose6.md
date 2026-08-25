# cw-standwalk-stance-mesh2-riseonly-bcchain3-slowchain-anchordose6

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-25T16:02:18+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-riseonly-bcchain3-slowchain

**wandb_id**: r0htou5e

**hypothesis**: Quarterchain/eighthchain (halving the BC-anchor chain pace further, twice) came back a REGRESSION not an improvement: det valid_plant fell 3/6(slowchain)->2/6(quarterchain), the 'cooler' deep-start cur_p95 median is a measurement artifact of MORE non-attempts (flat/rsi starts stall at height_err 76-86mm, video-confirmed same splayed press-up as ever, near-zero current because the leg never engages) rather than genuinely cooler landings (only 1/9 deep starts actually landed, at 1.46A). This suggests chain PACE has hit its floor between slowchain(1/2) and quarterchain(1/4). This arm tests the OTHER lever the gate names: is chain-tracking SUPERVISION STRENGTH (not speed) the blocker? Keep slowchain's already-working half-pace (lookahead 0.25s, min_h 8mm) fixed and double the anchor dose 3.0->6.0 (rise has never been dosed above the hold/lower-inherited 3.0 default). Prediction-if-true: deep-start (flat/bridge/rsi) valid_plant count rises above slowchain's own 3/6 det and/or bc_anchor_loss_rise drops below its ~0.05 plateau. Prediction-if-false: same plateau/regression signature (stalled splay, anchor loss unmoved) -- dose is not the remaining lever either, tightening the case for rung-9 (mesh-native IK rise ref).

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. 2M mechanism canary, DR-0 rise det+sto n=6+6 (same harness as slowchain/quarterchain/eighthchain). PASS: det>=4/6 AND sto>=4/6 valid_plant with cur_p95<=1.5A in valid episodes and zero over_current -- promote to 8M acquisition. PARTIAL: deep-start valid_plant count exceeds slowchain's 3/6 det and/or bc_anchor_loss_rise drops visibly below its ~0.05-0.07 plateau, even short of the bar -- dose has room, grid the next step up. FAIL: same stalled-splay signature (height_err 70-90mm on flat/rsi, anchor loss flat at the plateau) -- anchor dose is not the lever either; with pace also refuted (quarterchain/eighthchain/cont8), escalate to rung-9 (mint a mesh-native rise ref from scripted IK, since the primitive-extracted ref's flat/rsi segment may itself encode an infeasible-on-mesh posture) or attack the splayed-posture directly with a new reward term.

**verdict**: CANARY FAIL - MECHANISM: doubling the BC-anchor coef (3.0->6.0) on slowchain's own working half-pace makes things WORSE, not better, and via a DIFFERENT failure mode than the pace-dose siblings. DR-0 gate det 1/6 valid_plant (slowchain: 3/6), sto 2/6 (slowchain: 2/6), 9/12 over_current terms (slowchain: 3/12); own-DR(0.2) det 1/6/sto 0/6, 11/12 over_current. Unlike quarterchain/eighthchain's 'stalled/frozen splay' pathology (near-zero current, never engaging), THIS run's video (rise_det_0) shows a genuine, more aggressive rise attempt -- body visibly lifting/tucking across the clip -- that then trips over_current mid-motion; env/max_current_a climbs to 1.7-2.1A range through training and terminations/over_current SPIKES in the final report window (125 in the last chunk) rather than settling. bc_anchor_loss_rise also ends WORSE (0.098) than the established ~0.05-0.06 plateau every coef=3.0 sibling reaches, after briefly dipping to 0.072 mid-run -- the higher dose does not converge tracking better, it destabilizes it. Reward quarters fall every quarter (8.2/-23.0/-219.5/-420.6), a flat-to-falling aligned signal consistent with the bad eval (08-21 rule: not a 'needs more budget' case). CONCLUSION: raising anchor supervision strength ABOVE the hold/lower-inherited 3.0 default overshoots into a hotter, more over_current-prone tracking regime on this rung -- the opposite direction from what helped on hold/lower at lower doses. Read jointly with the paired -anchordose10 arm (same cycle): both land in the same worse-than-slowchain over_current signature, closing the anchor-dose-up axis for rise. Evidence: logs/ckpt_eval/cw_standwalk_stance_mesh2_riseonly_bcchain3_slowchain_anchordose6_{gate,owncfg}/, W&B r0htou5e.

