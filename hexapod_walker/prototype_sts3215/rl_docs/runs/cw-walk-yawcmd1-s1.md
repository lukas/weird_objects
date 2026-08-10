# cw-walk-yawcmd1-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T06:42:10+00:00

**pod**: hexapod-mjx-train-7

**steps**: 12000000

**parent**: cw-walk-yawcmd1-rr1

**wandb_id**: ycemc5zk

**hardware_ready**: False

**hypothesis**: Seed twin of cw-walk-yawcmd1-rr1 (P0/turning: yaw-rate command channel on the jittered driving package) -- yawcmd1-rr1 finished 12M/12M unverdicted (own-cfg gv 12/12 0 term, JOYSTICK GATE PASS 0 falls, forward slip 1.21/1.22<=1.25 gate -- but the yaw-tracking-specific metric (|wz_err| on commanded-turn segments vs |wz| on zero segments) needs custom analysis beyond the standard harness table, flagged DIG-IN). This seed twin gets a second data point ready in parallel rather than waiting serially, per the plan's 'idle pods during deliberation is a failure mode' rule.

**gate**: Own-cfg eval: commanded-turn segments |wz_err| med <= 0.10 rad/s AND wz_ref=0 segments |wz| med <= 0.05 rad/s (heading hold) AND JOYSTICK GATE retained (0 falls incl. flips) AND forward det med within parent band, slip <= 1.25 -- same gate as rr1, seed-robustness check

**verdict**: FAIL the yaw-tracking gate -- the run's actual purpose -- via the NEW eval_yaw panel (tool landed this cycle, rl_move/sim/eval_yaw.py; reusable for parent rr1). OBSERVATIONS: commanded-turn |wz_err| med 0.242 rad/s vs gate 0.10; right turns essentially untracked (err 0.378 at cmd -0.3 = |cmd|+drift), left only partially (0.098 at +0.15); turn-in-place both directions fails (0.22/0.35); heading-hold |wz| med 0.087 vs gate 0.05 (persistent ~5 deg/s drift); 0 falls anywhere incl yaw-flip stress. Independent confirmation from training telemetry: env/walk_yaw_err FLAT 0.138->0.130 rad/s first-to-last quarter (same in seed-0 rr1) with reward_walk_yaw comfortable at 0.66 throughout -- the channel was never learned. Everything else held: JOYSTICK GATE PASS (0 falls, trk_err 0.029), own-DR0.5 gv 12/12 prog med 0.98/0.99 slip 1.49/1.64 (lineage band), DR0 gate gv 12/12 (slip 1.28 vs 1.25 boilerplate cap -- moot). INTERPRETATION: the pre-registered if-false branch fired exactly as written: parked-yaw/heading-hold free income dominates -- with walk_yaw_zero_frac=0.5 half the ticks pay near-full kernel for doing nothing, and yaw_sigma 0.15 still pays ~0.28 at err 0.24, so ignoring wz_ref costs little. Not a seed issue (both seeds identical). VERDICT: FAIL; escalate per the hypothesis's own plan to yaw income gating (WISHLIST item 3 risk note) -- e.g. progress-gate the yaw kernel on signed wz toward wz_ref, the same worth-less-by-construction mechanism as walk_kernel_prog_gate. Linear driving is intact; the checkpoint is NOT a turning policy. HYPOTHESIS STATUS: refuted on the if-false branch, cleanly.

