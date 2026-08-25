# cw-standwalk-stance-mesh2-riseonly-bcchain3-decouple-c

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PARTIAL

**created**: 2026-08-25T16:17:57+00:00

**pod**: hexapod-mjx-train-4

**steps**: 8000000

**parent**: cw-standwalk-stance-mesh2-riseonly-bcchain3-slowchain

**wandb_id**: yz8pe501

**hypothesis**: Sibling of decouple-a/b (same cycle): the most extreme test, eighthchain's own collapsed lookahead_s=0.0625s but with the floor reverted to 15mm (vs eighthchain's paired 2mm that produced total 0/6+0/6 collapse with multiple all-six-leg duty=0 freeze episodes). If a strong floor alone rescues even this most-aggressive pace from total collapse, floor strength is confirmed as THE anti-freeze lever, independent of lookahead length, and pace-dosing should be re-run as a 2D grid (lookahead x floor) instead of the 1D scalar tried so far. Prediction-if-false: still collapses like eighthchain -- 0.0625s lookahead is simply too short for the anchor to supervise anything useful regardless of floor, closing this direction and confirming rung-9 (mesh-native ref / flat-segment fix) as the only remaining lever.

**gate**: DR-0 gate rise, det+sto n=6+6, dr-scale 0.0 (same harness as quarterchain/slowchain/eighthchain). Read as a 2D grid against the existing lookahead-only dose curve (slowchain 1/2=3/6+2/6 peak, quarterchain 1/4=2/6+2/6 w/ new freeze, eighthchain 1/8=0/6+0/6 total collapse). PASS: det>=4/6 AND sto>=4/6 valid_plant AND cur_p95<=1.5A in every valid episode AND zero over_current terms AND zero all-six-leg-duty=0 freeze episodes. PARTIAL: valid_plant count beats the SAME-lookahead paired-dose sibling (e.g. decouple-b beats quarterchain, decouple-c beats eighthchain) even short of PASS, confirming floor-strength as an independent lever. FAIL: valid_plant count and freeze-episode count are flat or worse than the same-lookahead paired-dose sibling -- floor strength is not the fix, lookahead length itself is the blocker; close this decoupling direction and fund rung-9 (mesh-native rise ref / flat-segment edit) instead.

**verdict**: Floor strength IS an independent anti-freeze lever: reverting min_h_ahead to 15mm at eighthchain's own 0.0625s lookahead rescues its TOTAL collapse (0/6+0/6, freezes) back to det 3/6 + sto 3/6 valid_plant with ZERO freeze episodes and even a clean bridge-start rise (det_1, hend 0.2mm, video-confirmed full stand). PARTIAL per pre-registered criteria (beats eighthchain on both valid_plant and freeze count). Key synthesis with decouple-a: once floor >=15mm the lookahead axis is flat (6/12 at 0.25s = 6/12 at 0.0625s), so the 'proper 2D grid' the gate contemplated is NOT worth funding -- both axes are now bracketed and the ceiling is invariant at ~5-6/12. Every remaining failure is the SAME wall: rsi/bridge deep starts tripping over_current at 2.64A mid-rise (video det_3 shows genuine progress -- legs tucking, body lifting -- then hot), across pace x floor x anchor-dose x budget. The reference trajectory itself doesn't offer a current-feasible deep-start path on the 3.5kg mesh model; rung-9 mesh-native ref (meshref/meshref-s1, in flight, other cycle) is the right and only funded lever. Next: no further pursuit-shaping arms; read meshref.

