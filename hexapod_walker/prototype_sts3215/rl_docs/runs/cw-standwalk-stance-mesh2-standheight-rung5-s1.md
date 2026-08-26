# cw-standwalk-stance-mesh2-standheight-rung5-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-26T04:43:39+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stancemix-tuckclock-scratch8m

**wandb_id**: z103ioia

**hypothesis**: Seed-1 twin of cw-standwalk-stance-mesh2-standheight-rung5 (identical recipe, only seed changed) -- same plain-English question: can the promoted mesh stance policy learn to raise/lower its stand height on command mid rise->hold->lower sequence without breaking rise/lower. Pairs with the seed-0 canary for a joint mechanism-health read instead of a single-seed lottery ticket.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. Same as cw-standwalk-stance-mesh2-standheight-rung5 (MECHANISM-HEALTH CANARY ONLY, 2M); judged jointly with the seed-0 twin -- PASS needs both seeds to clear (reward rises + probe shows real height tracking + rise/lower not majority-collapsed), PARTIAL if seeds disagree, FAIL if both show flat reward or majority over_current/fall.

