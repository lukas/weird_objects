# cw-standwalk-stance-mesh2-holdminload40-bcanchor0p5

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-25T12:19:04+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-holdminload40

**wandb_id**: vggrigsq

**hypothesis**: Rung-7 dose floor sibling of bcanchor1/bcanchor3: completes a 3-point dose curve (0.5/1.0/3.0) for the joint-space imitation anchor in one wave rather than serializing across cycles (operator 08-22 batching guidance). Same single-lever-off-holdminload40 base.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. Same as bcanchor1/3: 2M canary, hold DR-0 det+sto n=6+6, >=4/6 det valid_plant + cur_p95<=1.5A + rising hold_feet_factor/hold_load_factor = PASS; 1-3/6 = PARTIAL/continue; 0/6 pinned-signature = FAIL. Joint 3-arm dose read decides which dose (if any) funds an 8M acquisition arm.

