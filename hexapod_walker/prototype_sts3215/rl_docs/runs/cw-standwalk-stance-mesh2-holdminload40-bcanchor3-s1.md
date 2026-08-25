# cw-standwalk-stance-mesh2-holdminload40-bcanchor3-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED-PASS

**created**: 2026-08-25T12:46:12+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-holdminload40-bcanchor3

**wandb_id**: q1xfw1ik

**hypothesis**: Is the BC pose-anchor's escape from the 40mm hover basin seed-robust, or does it depend on the one shared init? All three rung-7 dose arms (0.5/1.0/3.0, all CANARY PASS with 6/6 det valid_plant) used the DEFAULT seed — identical network init — so dose-insensitivity is proven but seed-robustness is not. Seed twin (seed 1) of the dose-3.0 canary, identical recipe otherwise. Prediction-if-true: >=4/6 det valid_plant at 2M with the same clean-plant signature. Prediction-if-false: pinned 40mm/2.64A hover reappears — the anchor mechanism rides a lucky init, which matters if the running stdanneal 8M acquisition disappoints. Strongest alternative: partial plant (1-3/6), anchor helps but escape is slower from other inits.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY: 2M, hold DR-0 det+sto n=6+6; >=4/6 det valid_plant + cur_p95<=1.5A = seed-robust PASS; 1-3/6 = PARTIAL; 0/6 pinned 40mm/2.64A signature = seed-dependence evidence. Hedge read for the already-funded bcanchor3-stdanneal acquisition; does not itself fund anything.

**verdict**: CANARY PASS. The BC pose-anchor mechanism is seed-robust: a second network init reproduces the clean six-foot plant, so the rung-7 result did not ride one lucky seed. Seed-1 2M mechanism-health canary of the bcanchor3 recipe: DR-0 det 6/6 valid_plant at cur_p95 0.46A with zero terminations (canary PASS bar >=4/6), own-DR(0.2) det also 6/6 zero terms; det video = level six-foot plant held motionless, same as seed 0. sto 0/6 (hold_min_load) at both DRs -- the exact un-annealed policy_std~1.02 signature seed 0 showed at 2M, now PROVEN benign by the stdanneal acquisition (same cycle: 6/6 sto after annealing). Reward quarters (-3.8/-57.7/-93.0/-81.8) track seed 0 (-2.9/-57.9/-91.2/-86.2) closely. Hedge read complete: mechanism generalizes across inits AND doses (0.5-3.0 det-level; 1.0-3.0 sustained). No follow-up needed from this run -- the stdanneal champion supersedes it.

