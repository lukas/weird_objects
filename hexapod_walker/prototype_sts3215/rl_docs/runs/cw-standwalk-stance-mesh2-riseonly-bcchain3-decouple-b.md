# cw-standwalk-stance-mesh2-riseonly-bcchain3-decouple-b

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PARTIAL

**created**: 2026-08-25T16:15:48+00:00

**pod**: hexapod-mjx-train-3

**steps**: 8000000

**parent**: cw-standwalk-stance-mesh2-riseonly-bcchain3-slowchain

**wandb_id**: z36jhugx

**hypothesis**: Sibling of decouple-a (same cycle, same isolation): tests quarterchain's SHORTER lookahead_s=0.125s (which already showed a NEW freeze failure at quarterchain's paired floor of 4mm) but with the floor reverted to the strong original 15mm. If a strong floor rescues quarterchain's freeze while keeping its gentler (shorter) pursuit jump, this decouples the two mechanisms cleanly: floor strength prevents freezing, lookahead length controls current aggressiveness, and they can be dosed independently instead of as one scalar. Prediction-if-true: det/sto valid_plant >= quarterchain's own 2/6+2/6, freeze-signature episodes (all-six-leg duty=0) disappear even at this short lookahead. Prediction-if-false: freeze reappears despite the strong floor -- short lookahead itself (not the floor) is what starves the forward pull.

**gate**: DR-0 gate rise, det+sto n=6+6, dr-scale 0.0 (same harness as quarterchain/slowchain/eighthchain). Read as a 2D grid against the existing lookahead-only dose curve (slowchain 1/2=3/6+2/6 peak, quarterchain 1/4=2/6+2/6 w/ new freeze, eighthchain 1/8=0/6+0/6 total collapse). PASS: det>=4/6 AND sto>=4/6 valid_plant AND cur_p95<=1.5A in every valid episode AND zero over_current terms AND zero all-six-leg-duty=0 freeze episodes. PARTIAL: valid_plant count beats the SAME-lookahead paired-dose sibling (e.g. decouple-b beats quarterchain, decouple-c beats eighthchain) even short of PASS, confirming floor-strength as an independent lever. FAIL: valid_plant count and freeze-episode count are flat or worse than the same-lookahead paired-dose sibling -- floor strength is not the fix, lookahead length itself is the blocker; close this decoupling direction and fund rung-9 (mesh-native rise ref / flat-segment edit) instead.

**verdict**: Floor 4->15mm at quarterchain's own 0.125s lookahead: floor-strength lever confirmed AGAIN (3rd of 3 decouple arms) but ceiling is flat and low. DR-0 gate det 2/6 [bridge1/2 crouch1/1 flat0/1 rsi0/2] + sto 3/6 [crouch2/2 rsi1/4] valid_plant (5/12, beats quarterchain's 4/12) with ZERO freeze episodes (quarterchain: 6/12 froze) -- same anti-freeze rescue decouple-c showed at 0.0625s. Own-DR(0.2) same shape: det 3/6 + sto 2/6 (5/12, beats quarterchain's 3/12), zero freeze (quarterchain: 4/12). Cost of the rescue: over_current terms rise to 6/12 gate + 7/12 owncfg (quarterchain: 1/12 + 5/12) -- freeze converts to hot failure, not clean success; even the 'valid' bridge/rsi episodes run cur_p95 1.52-2.05A, over the 1.5A PASS bar. Video: det_0 (flat) genuinely presses up to a raised splayed stance before tripping hot; det_5 (bridge, usually a clean starter) instead sinks/splays and trips -- the strong floor pushes harder starts into the SAME over_current wall other starts already hit. Combined with decouple-a (FAIL, floor doesn't help at 0.25s lookahead) and decouple-c (PARTIAL, floor rescues 0.0625s) verdicted this window: floor>=15mm gives a FLAT ~5-6/12 valid_plant ceiling across all three tested lookaheads (0.25/0.125/0.0625s all cluster 5-6/12) with the identical over_current wall on rsi/bridge deep starts at the 2.64A cap -- the 2D floor x lookahead grid is now fully bracketed and closed, no further pursuit-shaping arm is worth funding. rung-9 (mesh-native rise ref, meshref/meshref-s1, in flight on another cycle's pods) is confirmed as the sole remaining lever for this rung.

