# cw-standwalk-stance-mesh2-holdminload40-bcanchor3-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-25T12:46:12+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-holdminload40-bcanchor3

**wandb_id**: q1xfw1ik

**hypothesis**: Is the BC pose-anchor's escape from the 40mm hover basin seed-robust, or does it depend on the one shared init? All three rung-7 dose arms (0.5/1.0/3.0, all CANARY PASS with 6/6 det valid_plant) used the DEFAULT seed — identical network init — so dose-insensitivity is proven but seed-robustness is not. Seed twin (seed 1) of the dose-3.0 canary, identical recipe otherwise. Prediction-if-true: >=4/6 det valid_plant at 2M with the same clean-plant signature. Prediction-if-false: pinned 40mm/2.64A hover reappears — the anchor mechanism rides a lucky init, which matters if the running stdanneal 8M acquisition disappoints. Strongest alternative: partial plant (1-3/6), anchor helps but escape is slower from other inits.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY: 2M, hold DR-0 det+sto n=6+6; >=4/6 det valid_plant + cur_p95<=1.5A = seed-robust PASS; 1-3/6 = PARTIAL; 0/6 pinned 40mm/2.64A signature = seed-dependence evidence. Hedge read for the already-funded bcanchor3-stdanneal acquisition; does not itself fund anything.

