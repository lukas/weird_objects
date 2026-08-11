# cw-stand-crouchrise2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-11T17:43:53+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-stand-holdbc1-hard1

**wandb_id**: 1gg41uez

**hardware_ready**: no

**hypothesis**: DISCOVERY (2M, warm from cw-stand-holdbc1-hard1, NOT from crouchrise1 — its hold cheat may be baked in): crouchrise1 proved the crouch-60% start mix fixes the lineage last defect (det crouch 5/5, RSI-off 8/8 vs hard1 0/8) but its second delta — goal-mix skew rise 0.45->0.6 / lower 0.45->0.3 — starved lower and let the flag-leg hold cheat back in (det hold feet 1+4 duty 0.07/0.01; valid_plant 7/12). crouchrise2 = ONE variable vs crouchrise1: restore hard1 exact goal mix (hold=0.1,rise=0.45,lower=0.45), keep goal.rise_flat_frac=0.10 / rise_partial_frac=0.30. Same holdbc1 stack, seed 13.

**gate**: PASS if det crouch-start rise valid >= 3/4 with zero tilt falls AND hold retention at hard1 level: hold det+sto valid_plant >= 10/12 AND det-hold per-foot contact duty >= 0.8 on ALL SIX feet (explicit) AND det flat/bridge rise not worse than hard1, no flag-leg/tripod cheat on video. FAIL if crouch <= 2/4 or hold/duty regress. PASS -> replaces hard1 as the stance deploy candidate; the robot export MUST ship WITH its goal-ramp profile, and the stand lineage closes.

**verdict**: FAIL (mixed, lever re-proven, NOT promoted -- same pattern as crouchrise1): the crouch-start rise fix REPRODUCES cleanly, restoring hard1 exact goal-mix on top of the crouch-60% start bias -- det rise 6/6 valid_plant across all start kinds incl. crouch 4/4 (>= the 3/4 gate bar), zero tilt falls, video-clean six-foot stands. But hold-retention FAILS its own explicit gate clause: det-hold per-foot duty_cycle is [0.99,0.03,0.96,0.98,0.03,0.96] IDENTICALLY across all 6 det episodes -- legs 1 and 4 at 0.03 duty, far under the required >=0.8 on all six feet (valid_plant reads True per-episode, exactly the blind spot the gate was written to catch with duty telemetry instead). This is the SAME flag-leg fingerprint and even the SAME two legs as crouchrise1 (1+4 duty 0.07/0.01) -- restoring the goal-mix did NOT fix it, so the goal-mix skew was not (solely) the cause. sto-hold duty is spread/reasonable (no near-zero leg), so the cheat is det-policy-specific. Additionally: det-lower is worse than its matched parent -- this eval 0/6 success, 2/6 hard tilt_pitch falls, other 4/6 end_posture_ok=false despite small height error; a same-eval matched-parent control on hard1 itself (never gated on lower before) gets 3/6 success, 1/6 falls -- so lower has a REAL pre-existing partial gap in the lineage that crouchrise2 makes measurably worse (3/6->0/6, +1 fall), not purely inherited unchanged. Do NOT promote/deploy this checkpoint; do not warm from it for a hold-focused follow-up (cheat may be baked in, per the crouchrise1 precedent). Banked as a usable crouch-rise-only reference (md5 55f8e80d) if a future arm isolates the rise-side fix from whatever destabilizes hold/lower.

**note**: created via `update --create`

