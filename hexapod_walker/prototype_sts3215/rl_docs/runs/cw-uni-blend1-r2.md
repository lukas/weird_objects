# cw-uni-blend1-r2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T22:05:53+00:00

**pod**: hexapod-mjx-train-4

**steps**: 18000000

**parent**: cw-uni-blend1

**wandb_id**: 4bwtkbhx

**hardware_ready**: no

**hypothesis**: Same as cw-uni-blend1 (unified joystick policy line: driving champion + goal-mix walk=0.7/hold=0.1/rise=0.1/lower=0.1) but on sim >=273ebde where femur/tibia/knee-servo now collide with the floor. blend1 was killed 25min in because its rise/lower could exploit shins passing through the ground (operator caught it in MuJoCo). If-true: rise/lower learned here are floor-respecting and hardware-plausible.

**gate**: JOYSTICK GATE retention AND rise/lower >= 5/6 AND quiet hold; VIDEO: no leg-through-floor in rise/lower

**verdict**: FAIL (rise/lower 0/6 vs gate >=5/6; everything else clean). OBSERVATIONS: JOYSTICK GATE PASS (0 falls all 12 scenarios incl. flip-stress, tracking err 0.025-0.052); DR0 walk retention gv 12/12, 0 term, det prog med 0.95; own-DR0.5 walk gv 12/12; hold det 6/6 succ (h_err <=6.5mm; sto 1/6 by the letter but h_err 0.1-9.5mm = drift criterion, visually quiet). rise 0/12 det+sto: robot STAYS CROUCHED churning legs (h_err_end 33-62mm, returns -131..-6, posture_ok false; strips show no upward motion across the full 15s). lower 0/12: robot KEEPS WALKING at standing height (fwd 0.2-0.4m during lower mode, h_err_end 22-52mm) and earns POSITIVE returns (+14..+153) for never descending. No leg-through-floor anywhere (end_clear >=-1mm noise, strips clean) -- sim fix 273ebde verified working in anger. W&B: eval rise_crouch/bridge/flat_frac and lower_success_frac FLAT ZERO from 1M to 18M; env/reward_rise_finish DECAYED 0.95->0.10 while total reward climbed 600->702 then plateaued (682/681). INTERPRETATION (root-cause chain): behavior <- incentive: 70% walk share with ~600-700/ep income dominates the shared actor's gradient; rise/lower at 10% each with tiny-to-negative returns never produce competitive advantage -- the policy actively traded rise-finish income away for walk income. <- pricing: in lower mode NOT descending still earns positive return (descending is not worth more than standing BY CONSTRUCTION -- violates the parking-vs-stepping design rule applied to height); in rise mode progress nets ~-0.014/step so crouch-parking is the local optimum. <- sim defect: none. NOT under-training: success flat 0 for 18M with plateaued reward -- more steps of this config buys nothing. VERDICT: FAIL, hardware_ready no (walk itself is champion-band but the deliverable is the unified policy). HYPOTHESIS STATUS: half-confirmed/half-refuted -- the floor fix did remove the shin exploit, but 'floor-respecting rise/lower learned at 10% mix' is FALSE: they never train at all. The plan's mix-ladder fallback (toward walk=0.9) targets walk EROSION, the opposite failure; adopted inverse ladder instead (raise rise/lower share: mix40 walk=0.4/rise=.25/lower=.25 and mix20 walk=0.2/rise=.35/lower=.35). If both stay pinned at 0 success, mix share is refuted as the lever and the next move is a rise/lower income scale audit (make not-lowering unprofitable by construction), not more ladder.

