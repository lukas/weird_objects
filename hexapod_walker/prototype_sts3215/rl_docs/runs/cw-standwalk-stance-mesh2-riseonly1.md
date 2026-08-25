# cw-standwalk-stance-mesh2-riseonly1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-25T06:55:48+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-cur1

**wandb_id**: myzj5aoo

**hypothesis**: Plain English: cur1/cur1-seed1's rise mode topples onto its side within the first couple seconds instead of pushing up, despite a demonstrated 25Hz reference (rise_ref_belly2plant.npz, k_rise_ref_track=2.0) that bank-checks as followable on mesh (replay ends valid plant 3/3). This arm isolates rise: same pricing/ref-tracking recipe, goal-mix forced to rise=1.0 (100% of training instead of 45%), 2M discovery. Prediction-if-true (rise was undertrained/diluted by the mix): rise ends closer to a valid plant (PLANT_SPEC height/attitude) on a majority of DR-0 episodes, tilt_pitch/over_current terminations become rare. Prediction-if-false: rise-only ALSO topples the same way -- the k_rise_ref_track=2.0 weight is simply too weak against the other shaping terms regardless of curriculum share (matches the mesh1-seed1 dig-in's own suspicion), and the next lever is a k_rise_ref_track dose increase (10-20x), not more curriculum share. Strongest alternative: 2M is too short regardless.

**gate**: Discovery/canary read at 2M: pod_eval stance panel, rise mode only, n=6 det DR-0. PASS-qualitative: majority (>=4/6) end in a PLANT_SPEC-valid or near-valid plant (height_err <30mm, no immediate topple), video shows a genuine push-up motion not a sideways roll. FAIL: same immediate-topple signature as cur1 -- escalate to a k_rise_ref_track dose sweep instead.

**verdict**: Diet isolation answer: rise does NOT emerge even with 100% of the goal mix. 11/12 DR-0 rise episodes terminate (det 6/6 over_current); video shows the familiar sprawled press-up — outrigger legs, feet skating, no valid plant, same signature as seed1-rr1. Training reward declined all 2M (7.3/-36.8/-147/-233) with zero task success. Read with refgain15's 20M FAIL (k_rise_ref_track=15, 0/36): rise-side failure is neither diet share nor ref-tracking gain. From-scratch rise on the 3.5kg mesh body at 100Hz needs a stronger prior — the rung-3 plan is hold-first (balance proven learnable) then rise from a hold-capable warm start. Discovery arm; question answered, lineage closed.

