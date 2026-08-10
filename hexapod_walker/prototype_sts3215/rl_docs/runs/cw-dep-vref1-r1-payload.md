# cw-dep-vref1-r1-payload

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T07:23:59+00:00

**pod**: hexapod-mjx-train-2

**steps**: 20000000

**parent**: cw-dep-vref1-r1

**wandb_id**: nkw2r49j

**hardware_ready**: False

**hypothesis**: PROTECT THE NAMED HARDWARE CANDIDATE: vref1-r1 is the leading checkpoint for tonight's hardware attempt #2, but has never carried the validated payload range (mass_scale 1.0-1.5x, from cw-walk-payload50) -- directly relevant since the real robot carries a battery/harness whose exact mass is uncertain. Per P0 rule 3, k_current=0 (do not price current on dep-line arms). If-true: own-cfg det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band; payload composes free like it does on every other lineage. If-false: contract-exact obs + added mass interact badly (velocity obs=ref no longer matches the now-heavier body's true dynamics as well) -- flag before hardware.

**gate**: Own-cfg (DR0.35+mass1.0-1.5x) det+sto 6/6 @15s: gait_valid 6/6, 0 term, slip/m within vref1-r1's own band (~0.89-1.13 det, ~1.13-1.36 sto); DR0 no-payload retention clean; frames watched det

**verdict**: FAIL -- first non-free axis on the contract-exact hardware-candidate line. Chassis payload/mass DR (1.0-1.5x) composed onto vref1-r1 per P0 rule 3 (k_current=0). Training reward declined steadily through the back half of training, unlike every sibling compose tonight (fric/comshift/deadband/encnoise/groundtilt5/gyronoise/imumount/latency/placement/zerobias all flat ~630-680 in the final reward quarter): peaked ~665 (quarter 2) then dropped monotonically to 342 (quarter 4), env/reward_walk_prog falling 0.74->0.44 over the same span, mean_current unchanged (not an instability/current spike). Own-cfg gate (DR0.35+mass1.0-1.5x) at the final checkpoint fails outright: det gait_valid 5/6 with a genuine sacrificed_legs=[5] episode (not the lineage's usual march-in-place crater), det prog med 0.57 (parent band ~0.9-1.0), det slip/m med 2.34 (parent ceiling ~1.13), fwd med 0.43m (parent ~0.74-0.78m); sto also degraded (prog med 0.81, slip med 1.38 vs parent's sto band ~1.13-1.36 at the top end). Frames watched across all 6 det + 6 sto episodes: mix of slow shuffling and the known crater pattern, but episode 3's leg-5 sacrifice is a genuinely new failure mode for this line, not the familiar inherited stall. Flat (no-payload) retention pass launched to check whether the checkpoint itself degraded outside the payload condition or only under load; result pending, will not block this verdict since the own-cfg gate already fails cleanly. RULING: unlike every other single DR axis tried on this checkpoint tonight, 1.0-1.5x mass/payload variation is NOT free -- flag for hardware attempt #2 if the real robot's battery/payload mass varies meaningfully from the training nominal; if payload turns out to matter physically, this checkpoint needs payload-aware retraining (lighter DR range or dedicated payload curriculum) before deployment, not a blanket assumption that DR axes compose for free.

