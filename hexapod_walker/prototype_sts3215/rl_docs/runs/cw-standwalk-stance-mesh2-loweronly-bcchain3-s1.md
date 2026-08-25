# cw-standwalk-stance-mesh2-loweronly-bcchain3-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-25T14:48:09+00:00

**pod**: hexapod-mjx-train-4

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-loweronly-bcchain3

**hypothesis**: Is the lower-rung IK-descent BC-anchor chain's escape from the mesh crouch (loweronly-bcchain3 CANARY PASS: DR-0 det 6/6 honest full-drop descents, 0 terminations, then its stdanneal acquisition just closed FULL PASS 24/24 det+sto DR-0+own-DR, cur_max down to 0.7-1.24A from the hot 2.17-2.26A canary) seed-robust, or was seed 0 a fluke -- mirroring the exact hold-rung seed hedge (bcanchor3-s1) that confirmed dose/seed robustness there? Identical recipe, coef 3.0, from-scratch, only the seed changes (0->1).

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. 2M mechanism canary, DR-0 det panel (n=6, --modes lower). PASS = det >=5/6 valid_plant with full commanded drop (height_err_end_mm within the canary's 0.1-3.7mm band) and zero over_current terms, cur_max reported vs the seed-0 canary's 2.17-2.26A (mechanism-health only, sto un-annealed-std failure is EXPECTED and does not fail this canary -- stdanneal is the proven fix if funded). FAIL = det collapses below 3/6 valid_plant or over_current terms appear -- seed 0 was a fluke, the mechanism needs a robustness fix before being trusted as the stage-1 lower answer.

