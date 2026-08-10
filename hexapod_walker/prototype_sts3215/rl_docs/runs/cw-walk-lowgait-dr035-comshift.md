# cw-walk-lowgait-dr035-comshift

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-10T03:11:15+00:00

**pod**: hexapod-mjx-train-1

**steps**: 20000000

**parent**: cw-walk-lowgait-dr035

**hardware_ready**: False

**hypothesis**: NEW compose, untried pairing: crouch height (-50mm, DR0.35, PASSed) x off-center payload (CoM offset 0.03m, the comshift_dr05 PASS envelope). CoM offset composes cleanly onto plain-walk and driving packages and now onto sloped-floor (groundtilt5-comshift, in flight) but never onto a CROUCHED stance -- a lower CoM height changes the lever arm for an off-center load, a distinct failure mode from CoM-offset at nominal height. One variable off lowgait-dr035: add dr.com_offset_m=0.03.

**gate**: own-cfg (DR0.35 crouch -50mm + com_offset 0.03) det+sto 6/6 @15s: gait_valid 12/12, 0 term, height err<=10mm, slip/m med<=1.6; DR0 nominal-height no-offset retention det 6/6 gv, height err<=8mm, slip/m<=1.15; frames watched det

**verdict**: FAILED-infra: 0-step launch-collision death (pid started, /tmp log never grew past 4074 bytes, matches this session's fleet-wide EOFError storm pattern) -- no science on the crouch x comshift compose. Retry landed clean as cw-walk-lowgait-dr035-comshift-r1 (separately triaged).

**refused_reason**: W&B already has a run named cw-walk-lowgait-dr035-comshift (names are append-only; pick a new one)

