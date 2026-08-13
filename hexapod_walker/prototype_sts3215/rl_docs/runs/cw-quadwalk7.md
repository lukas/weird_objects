# cw-quadwalk7

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: STOP

**created**: 2026-08-13T17:50:21+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-quadwalk6

**wandb_id**: tw2ui4kc

**hardware_ready**: False

**hypothesis**: Teach the quadruped to genuinely walk on four legs (front pair free as hands) instead of faking it by parking two middle legs. This arm tests whether the quadwalk6 failure was an EXPLORATION problem rather than a pricing problem. Evidence for that read: quadwalk6 added a structural income gate that verifiably flips the known mid-leg-sacrifice cheat from strongly profitable (matched-behavior comparison vs quadwalk5: same two legs [1,4] idle, same footwork fingerprint, return +722..+1146) to strongly unprofitable (-65..-290 for the identical behavior) -- proof the pricing now works -- yet a FRESH 2M-step run from the same quad_hold2 init, with the fix active from step 0, converged on the exact same now-losing behavior anyway. One lever vs quadwalk6: raise --ent-coef 0.001 -> 0.02 (10-20x), everything else identical (same gate, same init, same 2M discovery budget) -- this is the arm's own pre-registered branch (a) remedy: if the gate is right but exploration from the spawn can't find real four-leg stepping, the next lever is exploration, not pricing. Prediction-if-true: by 2M the det video shows all four support legs cycling contact/swing (gait_valid >0/6) and net forward progress, even if rough. Prediction-if-false: (a) the policy still converges to the same mid-leg-park shuffle despite the extra noise -- exploration alone isn't enough either, next question becomes whether a reactive MLP can balance a moving four-leg stance at all (architecture/curriculum question, not reward); (b) higher entropy destabilizes quad-hold retention (falls/roll blowup) -- would mean this entropy level is too hot for this warm start, try a smaller bump instead of abandoning the exploration hypothesis.

**gate**: Harness quadwalk det 6 eps @2M with the run's own cfg: >=4/6 eps net forward displacement >= +0.05m AND fronts lifted (post-grace tail lift duty <0.15 both lift legs) AND gait_valid >=3/6 (all four support legs cycling contact/swing, no sacrificed support leg) AND no episode net backward < -0.02m AND 0 falls AND roll_tail <= 3deg med. Retention: quad-hold survived 6/6, fronts lifted, creep no worse than the 0.41-0.50m lineage band, roll_tail <=3deg. Known cheats = one-line STOP: freeze/creep-in-stance, backward shuffle, six-leg replant, sacrificed-support-leg variant (incl. token-hop with support duty <0.1), lift-leg-as-prop (tail duty >=0.15 on a nominal lift leg). If gait_valid stays 0/6 AND the same legs [1,4] are sacrificed with the same footwork fingerprint as quadwalk5/6: exploration-only is CLOSED for this family, next needs an architecture/curriculum discussion, not another entropy scan.

**verdict**: STOP — pre-registered close: 20x entropy (ent-coef 0.001->0.02) did not escape the [1,4] mid-leg-sacrifice local optimum (gait_valid 0/6 det+sto, identical fingerprint to quadwalk5/6) even though quadwalk6 proved the walk_gait_gate makes that behavior lose -65..-290/ep. Exploration-only is CLOSED for the quadwalk family; next lever is architecture/curriculum (operator discussion). Quad-hold retention clean: 6/6, roll_tail 0.8deg, creep 0.36m in-band, 0 falls.

