# cw-walk-joyheaddeadband-s1-r4

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-10T02:30:16+00:00

**pod**: hexapod-mjx-train-3

**steps**: 16000000

**parent**: cw-walk-joyheaddeadband-s1-r3-c1

**hardware_ready**: False

**hypothesis**: 5th launch attempt of the joyheaddeadband seed-1 twin (s1, s1-r1, s1-r2 died to gotcha-13b EOFError; s1-r3 succeeded then got node-rebalanced for host starvation; the rebalanced s1-r3-c1 then ALSO died to the same EOFError, colliding with a concurrent cycle's own launch onto the same just-freed pod). Extremely heavy multi-cycle drain-storm window this whole cycle (02:00-02:25) -- every pod handoff this window has drawn 2+ simultaneous claimants. Same spec unchanged: seed-1 twin of cw-walk-joyheaddeadband. If this also fails, this joins the DIG-IN flag already raised for cw-walk-joyheaddeadband-s1's sibling pattern (c67 joyhead90-lat25-s1 precedent) rather than more blind retries.

**gate**: JOYSTICK GATE @90deg 0 in-envelope falls; own-cfg (DR0.5+lat+deadband1-3x) det+sto 6/6 @15s: gait_valid 6/6, 0 term, prog_ratio med>=0.80; DR0 retention gv 6/6; frames watched det.

**verdict**: INFRA FAILURE (not science): launch never appeared running in W&B within 240s (fleet launch-collision storm, gotcha 13b) -- 0 steps. Redundant with cw-walk-joyheaddeadband-s1-r3-c3-rr1, which is the SAME seed-1 spec and DID complete + PASS this cycle (seed-1 twin, panel closed 2/2). No further retry needed for this hypothesis.

