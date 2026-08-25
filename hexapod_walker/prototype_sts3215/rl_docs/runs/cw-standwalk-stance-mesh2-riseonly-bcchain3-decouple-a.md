# cw-standwalk-stance-mesh2-riseonly-bcchain3-decouple-a

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-25T16:12:22+00:00

**pod**: hexapod-mjx-train-2

**steps**: 8000000

**parent**: cw-standwalk-stance-mesh2-riseonly-bcchain3-slowchain

**wandb_id**: ahz9mrwq

**hypothesis**: Root-cause decoupling of the pace dose (this cycle's eighthchain/slowchain-cont8 triage): every pace arm so far (quarterchain/slowchain/eighthchain) moved bc_anchor_lookahead_s AND bc_anchor_min_h_ahead_mm TOGETHER as one 'pace' scalar, conflating two different mechanisms -- lookahead_s sets how far ahead the pursuit target sits in a genuinely-climbing region (torque aggressiveness), while min_h_ahead_mm sets how hard the anchor escapes the reference's ~5s flat dead-zone (anti-freeze; sim_env's height-floor search literally jumps forward until h clears the floor). eighthchain's total-freeze collapse (0/6+0/6, several deep starts at exactly 0 duty on all six legs) landed at the SMALLEST floor (2mm) as well as the smallest lookahead -- this arm isolates whether the freeze was actually caused by the weak floor, not the short lookahead. Reverts min_h_ahead_mm to the ORIGINAL bcanchor3-stdanneal value (15mm, known to produce genuine partial rises, never a total freeze) while independently testing slowchain's already-working lookahead_s=0.25s. Prediction-if-true: matches or beats slowchain's 3/6 det + 2/6 sto valid_plant with a floor strong enough to prevent any freeze episodes. Prediction-if-false: unmoved vs slowchain (floor alone doesn't help) -- lookahead_s is the real lever after all, not the floor.

**gate**: DR-0 gate rise, det+sto n=6+6, dr-scale 0.0 (same harness as quarterchain/slowchain/eighthchain). Read as a 2D grid against the existing lookahead-only dose curve (slowchain 1/2=3/6+2/6 peak, quarterchain 1/4=2/6+2/6 w/ new freeze, eighthchain 1/8=0/6+0/6 total collapse). PASS: det>=4/6 AND sto>=4/6 valid_plant AND cur_p95<=1.5A in every valid episode AND zero over_current terms AND zero all-six-leg-duty=0 freeze episodes. PARTIAL: valid_plant count beats the SAME-lookahead paired-dose sibling (e.g. decouple-b beats quarterchain, decouple-c beats eighthchain) even short of PASS, confirming floor-strength as an independent lever. FAIL: valid_plant count and freeze-episode count are flat or worse than the same-lookahead paired-dose sibling -- floor strength is not the fix, lookahead length itself is the blocker; close this decoupling direction and fund rung-9 (mesh-native rise ref / flat-segment edit) instead.

**verdict**: Floor 8->15mm at slowchain's own 0.25s lookahead does NOT improve rise: DR-0 gate det 2/6 + sto 4/6 valid_plant (6/12) vs slowchain 5/12 -- +1 episode, inside eval noise -- while over_current terms DOUBLE (6/12 vs slowchain 3/12; all failures are rsi/bridge deep starts pinned at 2.64A, hend 20-77mm; video det_1 bridge shows straining without lifting). Zero freezes both, so no freeze-rescue signal at this lookahead. Per pre-registered criteria this is flat/worse vs the same-lookahead sibling: at an already-working lookahead, a stronger height-floor only pushes deep starts hotter, mirroring the anchor-dose overshoot. Next: floor axis at long lookahead closed; see decouple-c for the real finding (floor rescues short-lookahead collapse); rung-9 meshref arms (in flight, other cycle) remain the live lever for the deep-start over_current wall.

