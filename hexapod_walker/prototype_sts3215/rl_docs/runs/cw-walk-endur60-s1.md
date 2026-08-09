# cw-walk-endur60-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T14:35:01+00:00

**pod**: hexapod-mjx-train-4

**steps**: 20000000

**parent**: cw-walk-anchorgate

**hardware_ready**: no

**hypothesis**: Seed twin of cw-walk-endur60 per operator ruling 7 (promotion needs multi-seed panels). endur60 PASSed its 60s endurance gate with the best DR0 det slip/m yet measured (med 0.887 vs champion longdist-r2's 0.96) and 3.17m median @60s, gv 12/12, 0 term — a champion-path candidate. Identical config except --seed 1. If both seeds land the same result the 60s-horizon effect is seed-robust and the endur60 line goes to a DR1.0/corner panel against longdist-r2 for possible promotion; if they diverge, the slip gain was seed luck and the champion stands.

**gate**: DR0 det+sto 6/6: median fwd distance >=2.4 m @ 60 s, zero terminations, gait_valid 12/12, det slip/m median <=1.0; frames watched det

**verdict**: FAIL on the det slip clause; endurance itself CONFIRMED seed-robust. DR0 60s panel: det med fwd 3.08m / sto 2.89m (gate >=2.4), gv 12/12, 0 term, no late-episode decay — matches seed-0 endur60 (3.17m). But det slip/m med 1.132 vs gate <=1.0 and vs seed-0's 0.887; per-ep ranges non-overlapping (1.07-1.18 vs 0.84-0.94) = real seed divergence, so endur60's best-ever slip was SEED LUCK per pre-registration; no promotion panel, champion longdist-r2 stands. One sto draw-stall ep (prog 0.17) = lineage trait. Frames det watched: level six-leg cycling for the full 60s, still paddling-class transport. hardware-ready: no.

