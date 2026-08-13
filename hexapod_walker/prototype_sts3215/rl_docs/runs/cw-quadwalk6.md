# cw-quadwalk6

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-13T17:28:56+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-quadwalk5

**wandb_id**: 60eu6ogs

**hardware_ready**: False

**hypothesis**: Stop paying the robot for moving unless every one of its four support legs is genuinely stepping — if any leg is parked (in the air or pinned to the floor), the walking money drops to zero instead of a payable fine; this arm tests whether that structural change finally makes the policy step with all four legs. Five arms (quadwalk1-5) measured that additive charges cannot do this: quadwalk3 verifiably PAID a -575/ep lift-contact charge (~40% of return) and kept six-legging, and quadwalk5's 6x k_park_duty reprice changed the mid-leg-park scoot not at all (identical legs [1,4] sacrificed 12/12) - the anchor gate never sees an air-parked leg because its fraction spans loaded feet only. One lever vs quadwalk5: reward.walk_gait_gate=1.0 (new, bank-proven: velocity income kernel+positive-progress+quadwalk clear/plant is multiplied by the MIN over commanded support legs of a completed-a-real-swing-recently score, 2s window + 2s fade on commanded ticks, swing = >=2 ticks airborne + >=10mm XY stride; lift legs exempt; penalties never shrink; semantics tests measure the scripted mid-pin scoot losing 54% of return and 72% of its income while the honest six-leg walk gait keeps walk_gait_min=1.0 and ~99% of income). Prediction-if-true: by 2M the det video shows all four support legs cycling contact/swing (gait_valid >0/6), positive forward travel toward 0.05 m/s, fronts lifted. Prediction-if-false: (a) policy freezes/creeps in the quad stance to dodge the zeroed income (park-duty + still charges make that negative - known one-line STOP, would mean the gate is right but 2M exploration from the spawn cannot find four-leg stepping, next lever is exploration/curriculum or a feedback-reference discussion, NOT pricing); (b) token-hop cheat - mids hop >=10mm every <=2s but duty stays <0.1 (park duty still charges it; would mean stride/window bars need coupling to duty, a gate parameter fix, not a new mechanism); (c) balance failure - genuine stepping attempts but falls (balance curriculum next, matches quadwalk1 class (c)). Strongest alternative: closed-loop four-leg balance is beyond a reactive MLP at any reward - shows as (c) persisting.

**gate**: Harness quadwalk det 6 eps @2M with the run's own cfg: >=4/6 eps net forward displacement >= +0.05m AND fronts lifted (post-grace tail lift duty <0.15 both lift legs) AND gait_valid >=3/6 (all four support legs cycling contact/swing, no sacrificed support leg) AND no episode net backward < -0.02m AND 0 falls AND roll_tail <= 3deg med. Retention: quad-hold survived 6/6, fronts lifted, creep no worse than the 0.43-0.50m lineage band. Known cheats = one-line STOP: freeze/creep-in-stance, backward shuffle, six-leg replant, sacrificed-support-leg variant (incl. token-hop with support duty <0.1). PASS does NOT make it the bank reference (needs full QUADWALK_REF_GATE.md).

**verdict**: STOP — pricing PROVEN (matched A/B vs quadwalk5, identical cheat: sac legs [1,4], near-identical per-leg swing_count fingerprint, return flips +722..+1146 -> -65..-290 for the SAME behavior) but the policy is STUCK in that now-unprofitable basin after 2M fresh steps from the shared quad_hold2 init (gait_valid 0/6 det+sto unchanged). Secondary: front lift-leg dragged into support service (duty_tail up to 0.44, fronts_lifted fails 9/12) despite k_quad_lift_contact=3.0 firing -- an additive fine, paid, same lesson as quadwalk3 but on the lift-leg side. Quad-hold retention clean (fwd creep 0.41m det, in-band vs 0.43-0.50m lineage; roll_tail 0.6deg; height_err 6.3mm; 0 falls). Verdict: exploration-blocked local optimum, not a pricing problem -- next lever is entropy/exploration (pre-registered branch a), not another coefficient.

