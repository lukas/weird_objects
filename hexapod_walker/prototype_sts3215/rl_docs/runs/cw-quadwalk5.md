# cw-quadwalk5

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: STOPPED

**created**: 2026-08-13T16:33:09+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-quadwalk4

**wandb_id**: cufgjfk2

**hardware_ready**: False

**hypothesis**: Make hanging a support leg in the air cost about as much as dragging it on the floor, so the robot's cheapest option becomes actually stepping with all four support legs; this arm re-runs the quad-spawn walker with that single price fixed. cw-quadwalk4 proved the quad spawn kills the fronts-down cheat (fronts lifted 11/12, first time in four arms) but the policy then held its MID legs up too and scooted on the rear pair: k_drag_loaded=10 prices a mid foot touching down and scuffing ~100x higher than k_park_duty=1.0 prices holding it airborne (~-0.12/tick, ~7% of the 1243/ep return). One lever vs quadwalk4: reward.k_park_duty 1.0->6.0 (existing bank-verified term, lift legs already exempt), making the mid-flag posture cost ~-0.9/tick (~55% of return) while a real gait (duty 0.1-0.9) still pays zero. Unlike quadwalk3's paid charge, putting the mids down is not a basin crossing — quadwalk2 already showed contact duty moves with price, and the spawn starts the mids ON the ground; the policy actively lifts them. Prediction-if-true: det video by 2M shows all four support legs cycling contact/swing (gait_valid >0/6), speed toward the commanded 0.05 m/s, slip well below quadwalk4's 13/m. Prediction-if-false: charge paid again (mids stay up — pricing is then exhausted for support-leg use too and the next lever class is a stepping curriculum/reference, likely an operator discussion), or mids plant but pin at duty ~1.0 as drag anchors (park ceiling should also charge that — would mean the scoot itself, not the leg economy, is the attractor), or it freezes to dodge every charge (known one-line STOP).

**gate**: Harness quadwalk det 6 eps @2M with the run's own cfg: >=4/6 eps net forward displacement >= +0.05m AND fronts lifted (post-grace tail lift duty <0.15 both lift legs) AND gait_valid >=3/6 (all four support legs cycling contact/swing, no sacrificed support leg) AND no episode net backward < -0.02m AND 0 falls AND roll_tail <= 3deg med. Retention: quad-hold survived 6/6, fronts lifted, drift creep no worse than the 0.43-0.50m lineage band. Known cheats = one-line STOP: freeze/creep-in-stance, backward shuffle, six-leg replant, NEW sacrificed-support-leg variant. PASS does NOT make it the bank reference (needs full QUADWALK_REF_GATE.md).

**verdict**: STOP — known exploit (sacrificed-support-leg variant, pre-registered): 6x k_park_duty price hike (1.0->6.0) did NOT change the cheat from quadwalk4 — det+sto both sacrifice the exact same mid legs [1,4] in all 12/12 episodes (gait_valid 0/6, slip 13.25 med det / 11.31 sto, prog_ratio ~0.10, fwd med 0.17m det vs 0.05-0.06 commanded), fronts_lifted actually regressed to 3/6 det (was 11/12 in quadwalk4). Quad-hold retention ok (6/6 survived, fwd creep 0.49m det / 0.40m sto, in the 0.43-0.50m lineage band; roll_tail 2.5deg det / 0.7deg sto, within the 3deg cap). Pricing-only is now measured-exhausted for BOTH quadwalk cheat families found so far (fronts-down: quadwalk1-3; mid-leg-park: quadwalk4-5) — a 6x charge that should dominate return produced zero behavior change, so the next lever is a structural CODE fix (something that closes the sacrifice-any-subset loophole generically), not another coefficient. No further reprice-only quadwalk arm until that lever exists.

