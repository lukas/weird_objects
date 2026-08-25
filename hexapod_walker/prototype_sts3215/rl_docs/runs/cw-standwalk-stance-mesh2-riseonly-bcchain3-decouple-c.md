# cw-standwalk-stance-mesh2-riseonly-bcchain3-decouple-c

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-25T16:17:57+00:00

**pod**: hexapod-mjx-train-4

**steps**: 8000000

**parent**: cw-standwalk-stance-mesh2-riseonly-bcchain3-slowchain

**wandb_id**: yz8pe501

**hypothesis**: Sibling of decouple-a/b (same cycle): the most extreme test, eighthchain's own collapsed lookahead_s=0.0625s but with the floor reverted to 15mm (vs eighthchain's paired 2mm that produced total 0/6+0/6 collapse with multiple all-six-leg duty=0 freeze episodes). If a strong floor alone rescues even this most-aggressive pace from total collapse, floor strength is confirmed as THE anti-freeze lever, independent of lookahead length, and pace-dosing should be re-run as a 2D grid (lookahead x floor) instead of the 1D scalar tried so far. Prediction-if-false: still collapses like eighthchain -- 0.0625s lookahead is simply too short for the anchor to supervise anything useful regardless of floor, closing this direction and confirming rung-9 (mesh-native ref / flat-segment fix) as the only remaining lever.

**gate**: DR-0 gate rise, det+sto n=6+6, dr-scale 0.0 (same harness as quarterchain/slowchain/eighthchain). Read as a 2D grid against the existing lookahead-only dose curve (slowchain 1/2=3/6+2/6 peak, quarterchain 1/4=2/6+2/6 w/ new freeze, eighthchain 1/8=0/6+0/6 total collapse). PASS: det>=4/6 AND sto>=4/6 valid_plant AND cur_p95<=1.5A in every valid episode AND zero over_current terms AND zero all-six-leg-duty=0 freeze episodes. PARTIAL: valid_plant count beats the SAME-lookahead paired-dose sibling (e.g. decouple-b beats quarterchain, decouple-c beats eighthchain) even short of PASS, confirming floor-strength as an independent lever. FAIL: valid_plant count and freeze-episode count are flat or worse than the same-lookahead paired-dose sibling -- floor strength is not the fix, lookahead length itself is the blocker; close this decoupling direction and fund rung-9 (mesh-native rise ref / flat-segment edit) instead.

