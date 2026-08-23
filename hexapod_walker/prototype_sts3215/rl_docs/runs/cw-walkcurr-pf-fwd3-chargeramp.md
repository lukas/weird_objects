# cw-walkcurr-pf-fwd3-chargeramp

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T18:15:04+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-walkcurr-pf-fwd1

**wandb_id**: 0whmb97m

**hypothesis**: Plain English: three rung-1 arms froze because the dense walk charges (~-4.7/step) make freezing the best policy a random-init learner can reach and punish the flailing exploration must pass through — this arm starts those four charges (loadslip/park/idle/heading) at 40% dose and anneals them to the full bank-proven dose over the first 1M steps (new reward.walk_charge_ramp_steps mechanism, mirror of the term-penalty/drag-allow ramp contracts, tests 6/6). At the measured 0.40 floor the landscape is a monotone positive-sum ladder toward walking (gait +439.7 > creep +327.7 > stall +230.1 > park +98.5 > shuffle +27.6 > sideways -142 > reverse -219 > skate -302 > topple -1164 — full operator ranking intact, floor-bank 3/3 green; 0.15 was measured to invert skate/shuffle and rejected). Exact fwd1 recipe otherwise (no swing bonus — measured inert; term 1200), fresh init. Prediction-if-true: walk_prog leaves 0.0 and step-event rate leaves 0.02/step within the first 1M (while charges are loose), and behavior survives the anneal to full pricing in the second 1M. Prediction-if-false: (a) freeze again even at 40% charges = the charge flow was not the binding constraint, exploration fix moves to init/noise (log_std) or rung-0 curriculum; (b) walks early then collapses as charges anneal in = ramp too fast, pre-registered follow-up is longer ramp/higher floor, not a new mechanism. Strongest alternative: positive-sum park (+98.5 at floor) becomes a sticky camp PPO never leaves even though stall pays +132 more — watch reward_park_duty vs step-event rate.

**gate**: Rung-1 gate (same as fwd1): C-env det fixed-forward panel — zero tilt terms, cmd_prog_frac >= 0.35, direction_err <= 30 deg, slip/m <= 3.0, six legs cycling on >=4/6 episodes, video shows real stepping (eval judges FULL bank-proven pricing by ramp construction). Discovery-health (08-21 ruling): walk_prog > 0 and rising at 2M with panel short = continue; walk_prog still identically 0.0 at 2M = charge-flow hypothesis refuted, rung-1 dig-in on init/exploration, no same-recipe continuation.

