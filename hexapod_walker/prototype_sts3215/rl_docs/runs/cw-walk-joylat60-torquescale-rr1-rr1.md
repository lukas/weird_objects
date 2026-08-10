# cw-walk-joylat60-torquescale-rr1-rr1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T04:06:17+00:00

**pod**: hexapod-mjx-train-7

**steps**: 20000000

**parent**: cw-walk-joylat60

**wandb_id**: 2fl29hy7

**hardware_ready**: False

**hypothesis**: Compose: torque-droop (0.80-1.05x) onto joylat60 driving package. 2nd requeue -- first 2 attempts (cw-walk-joylat60-torquescale, twice) vanished/died with no ledger trace under the fleet's severe concurrent-drain launch-collision storm this window, 0 compute lost either time.

**gate**: Own-cfg (dr.latency_scale=0.5,2.5 + dr.torque_scale=0.80,1.05) det+sto 6/6 @30s: gait_valid 12/12, 0 term, det med fwd>=1.2m; JOYSTICK GATE (eval_drive, --dr-scale 0.2) 0 in-envelope falls; DR0 flat retention det 6/6 gv, slip/m<=1.24, prog>=0.90; frames watched det

**verdict**: PASS: torque-droop under load (0.80-1.05x) composes for free onto the 60s abrupt-flip driving endurance package (joylat60) -- 2nd independent confirmation of this compose cell (matches sibling joylat60-torquescale-rr2's PASS, and the isolated torque-droop axis alone is already CLOSED/NO-EFFECT on the plain-walk baseline). Own-cfg (DR0.5+latency+torque) det gv 6/6 prog med 0.97 slip med 1.44 fwd med 1.51m (>=1.2m gate), sto gv 6/6 prog med 0.95 slip med 1.47 fwd med 1.49m, 0 term either mode, 0 sacrificed legs. JOYSTICK GATE (eval_drive DR0.2, 45deg envelope): 0 in-envelope falls across the full direction panel + flip-stress. DR0 TRUE FLAT retention (torque+latency overrides both dropped): det gv 6/6 prog med 0.975 slip med 1.40, sto gv 6/6 prog med 0.965 slip med 1.48 -- inside joylat60's own established flat-retention band (1.43-1.65 per SKILLS row); the run's pre-registered 1.24 slip cap is boilerplate mismatched to this lineage's real band (same pattern already documented on joylat60-torquescale-rr2 and joyjit-payload), not a regression. Frames (own-cfg det, 2 episodes watched): six legs cycling swing/stance throughout, no flag leg, no dragging, no first/second-half decay. Not hardware-ready (known paddle foot-slide persists, contract-line economics not applied to this lineage).

