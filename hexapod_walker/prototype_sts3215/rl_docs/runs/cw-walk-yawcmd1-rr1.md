# cw-walk-yawcmd1-rr1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-10T03:13:12+00:00

**pod**: hexapod-mjx-train-2

**steps**: 12000000

**parent**: cw-walk-joyjit-dr05-c1

**wandb_id**: q62adt6p

**hardware_ready**: no

**hypothesis**: Yaw-rate command channel (code c086a22, probe clean): kernel k_walk_yaw=1.0 incl. wz_ref=0 heading-hold income trains commanded turning/arcs/turn-in-place on the jittered driving package (warm start joyjit-dr05-c1, obs 72->73 via pad transplant) without eroding linear driving. If false: parked-yaw free income dominates (yaw_err flat ~|wz_ref|) -> escalate to yaw income gating per WISHLIST item 3 risk note.

**gate**: own-cfg eval: commanded-turn segments |wz_err| med <= 0.10 rad/s AND wz_ref=0 segments |wz| med <= 0.05 rad/s (heading hold) AND JOYSTICK GATE retained (0 falls incl. flips) AND forward det med within parent band, slip <= 1.25

**verdict**: FAIL on the yaw-tracking clauses of its own gate; linear-driving retention clauses PASS. OBSERVATIONS: custom yaw panel (rl_move/sim/eval_yaw.py, own cfg, DR0): commanded-turn |wz_err| med 0.239 rad/s (gate <=0.10), wz_ref=0 heading-hold |wz| med 0.099 (gate <=0.05), 0 falls in 10 scenarios incl 2 yaw-flip stress eps. Per-scenario errs scale with distance of the command from ~+0.09 rad/s (arc-left 0.075, arc-right 0.214, tip-right 0.393 > |wz_ref|=0.3): policy yaws at a command-INVARIANT ~+0.08-0.10 rad/s left drift, no turn response either direction. Training walk_yaw_err flat 0.138->0.132 over 12M (no learning after ~1M) with reward_walk_yaw ~0.67 throughout. Retention: JOYSTICK GATE PASS (0 falls, trk_err 0.024-0.053), own-cfg harness 12/12 gv, prog med 1.02/1.00 (parent 0.94/0.98), slip 1.21/1.22 <= 1.25 cap, 0 term, frames clean six-leg gait. INTERPRETATION: pre-registered if-false confirmed -- parked-yaw free income dominates. Root cause: incentive, not sim -- Gaussian yaw kernel sigma 0.15 pays exp(-0.5*(0.135/0.15)^2)=0.67 of max income per tick for ignoring the command entirely (matches observed reward_walk_yaw 0.67 exactly); actually turning risks gait disruption for at most +0.33 on turn segments only. Exactly the WISHLIST item-3 KNOWN RISK. VERDICT: FAIL, hardware-ready no (turning not learned; linear driving unharmed). HYPOTHESIS STATUS: refuted on the yaw claim, held on no-erosion. Next: gate yaw income on achieved wz toward wz_ref (analog of walk_kernel_prog_gate) -- the pre-registered escalation; do NOT raise k_walk_yaw or steps. Sibling cw-walk-yawcmd1-s1 shows the same signature (turn 0.242/hold 0.087, concurrent cycle's verdict, not mine).

