# cw-standwalk-stance-mesh2-riseonly-bcchain3-decouple-b

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-25T16:15:48+00:00

**pod**: hexapod-mjx-train-3

**steps**: 8000000

**parent**: cw-standwalk-stance-mesh2-riseonly-bcchain3-slowchain

**wandb_id**: z36jhugx

**hypothesis**: Sibling of decouple-a (same cycle, same isolation): tests quarterchain's SHORTER lookahead_s=0.125s (which already showed a NEW freeze failure at quarterchain's paired floor of 4mm) but with the floor reverted to the strong original 15mm. If a strong floor rescues quarterchain's freeze while keeping its gentler (shorter) pursuit jump, this decouples the two mechanisms cleanly: floor strength prevents freezing, lookahead length controls current aggressiveness, and they can be dosed independently instead of as one scalar. Prediction-if-true: det/sto valid_plant >= quarterchain's own 2/6+2/6, freeze-signature episodes (all-six-leg duty=0) disappear even at this short lookahead. Prediction-if-false: freeze reappears despite the strong floor -- short lookahead itself (not the floor) is what starves the forward pull.

**gate**: DR-0 gate rise, det+sto n=6+6, dr-scale 0.0 (same harness as quarterchain/slowchain/eighthchain). Read as a 2D grid against the existing lookahead-only dose curve (slowchain 1/2=3/6+2/6 peak, quarterchain 1/4=2/6+2/6 w/ new freeze, eighthchain 1/8=0/6+0/6 total collapse). PASS: det>=4/6 AND sto>=4/6 valid_plant AND cur_p95<=1.5A in every valid episode AND zero over_current terms AND zero all-six-leg-duty=0 freeze episodes. PARTIAL: valid_plant count beats the SAME-lookahead paired-dose sibling (e.g. decouple-b beats quarterchain, decouple-c beats eighthchain) even short of PASS, confirming floor-strength as an independent lever. FAIL: valid_plant count and freeze-episode count are flat or worse than the same-lookahead paired-dose sibling -- floor strength is not the fix, lookahead length itself is the blocker; close this decoupling direction and fund rung-9 (mesh-native rise ref / flat-segment edit) instead.

