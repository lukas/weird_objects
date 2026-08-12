# cw-stand-riserock4

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-12T03:21:44+00:00

**pod**: hexapod-mjx-train-4

**steps**: 2000000

**parent**: cw-stand-holdbc1-hard1

**wandb_id**: 1444a1nt

**hypothesis**: Teach the stand specialist to survive the roll the real robot's belly-curl actually produces, using the mechanism-corrected RAMP-GATED rise-rock axis (a fold bias that grows with height-ramp progress, matching the tape-measured flat-then-accelerating-roll shape) instead of the old persistent/flat bias that turned out to test a rock shape no hardware tape ever shows. One change vs hard1: dr.rise_rock_prob=0.5, dr.rise_rock_deg=8,18 (the calibrated ramp-gated default, target ~18deg dose at ramp end). Prediction-if-true: under the exact bench trip dose (10deg, guaranteed) this checkpoint clears the rise gate that both hard1 and the old flat-bias riserock arms failed (matched-parent separation for the first time in this family). Prediction-if-false: still zero separation -- the whole command-bias/fold-DR family is closed for good regardless of shape, and only true contact/pinning geometry modeling (belly/tucked-shank collision) remains as a lever.

**gate**: PASS if injected det rise (dr.rise_rock_prob=1.0, dr.rise_rock_deg=10,10 fixed, the exact bench trip angle) is >=5/6 valid_plant with ZERO tilt falls AND frozen hard1 under the IDENTICAL injection still fails (matched-parent control, >=2/6 falls) AND nominal own-mix det rise/hold/LOWER retention matches hard1's own probe (12/12 valid_plant incl. flat 4/4, hold 11/12) with NO flag-leg/outrigger cheat on lower video (the riserock3 lesson -- a lower regression under this axis is disqualifying regardless of the rise-rock result in isolation). FAIL on zero separation OR any known-exploit cheat in the retention check.

