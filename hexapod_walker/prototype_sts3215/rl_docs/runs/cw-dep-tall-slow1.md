# cw-dep-tall-slow1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: DONE

**created**: 2026-08-11T22:10:53+00:00

**pod**: hexapod-mjx-train-4

**steps**: 2000000

**parent**: cw-dep-tall30

**wandb_id**: usjcemku

**hardware_ready**: False

**hypothesis**: TALL LADDER T4: speed trade. Shrink the commanded band 0.05-0.06 -> 0.03-0.04 at ref -15, warm from tall30. T5 probe says the crouch buys STABILITY (yaw-splay at limit) for gait speed; if speed demand drops, the stability budget needed drops, and posture may rise without any new pricing. Watch the yaw-limit signature: if mean leg yaw comes off the 35deg wall, the mechanism is confirmed.

**gate**: Primary metric: probe_tall_wall.py steady-state walking height (parent = -75mm). PASS: walking height >= -50mm, speed in the 0.03-0.04 band, survived 1, slip <=1.8. If PASS: rung speed back up with height held. FAIL: height flat = speed pressure is not the pin.

**verdict**: INFORMATIVE FAIL (independently confirmed, matches concurrent cycle's T2/T4 verdict logged 22:27). probe_tall_wall.py steady-state walking height -73.8mm (mean over 3 seeds), flat vs parent tall30's -75mm wall despite easing the commanded speed band 0.05-0.06->0.03-0.04 -- the policy didn't even adopt the easier band (harness measured speed 0.048-0.051 m/s, still above the new 0.04 ceiling), so speed pressure is refuted as the pin twice over: the crouch didn't rise AND the speed didn't drop. Leg-yaw pinned at the 35deg limit (margin -0.24 to -0.48deg), same fingerprint as every other tall arm. Honest six-leg gait, no park/cheat (gait_valid 5/6). Speed-relief lever CLOSED.

