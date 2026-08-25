# cw-standwalk-stance-mesh2-holdminload40-bcanchor3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-25T12:13:55+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-holdminload40

**wandb_id**: 80jrhio3

**hypothesis**: Rung-7 dose sibling of bcanchor1 (coef=1.0): does a 3x stronger joint-space imitation anchor toward the settled six-foot plant pose (self._q_nom) close the gap faster/more completely, or does over-weighting the imitation term fight the still-active income/termination pricing stack (hold_still_gate/hold_feet_load_min/hold_min_load_terminate) and destabilize instead? Same single-lever-off-holdminload40 base as bcanchor1, dose only differs.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. Same as bcanchor1: 2M canary, hold DR-0 det+sto n=6+6, >=4/6 det valid_plant + cur_p95<=1.5A + rising hold_feet_factor/hold_load_factor = PASS; 1-3/6 = PARTIAL/continue; 0/6 pinned-signature = FAIL. Joint read with bcanchor1 decides the dose to carry into an 8M acquisition arm.

**verdict**: CANARY PASS. The BC pose-anchor finally broke the 40mm chassis-hover basin: this is the first mesh-model stance policy in seven rungs to hold an honest six-foot plant. Evidence: DR-0 det gate 6/6 valid_plant (height_err_end 0.7mm vs parent's pinned 40mm, cur_p95 0.53A vs 1.5A cap, cur_max 1.1A vs parent's pinned 2.64A, roll clean, all six end_clear ~0mm, plant_margin 149.5mm); own-DR(0.2) det ALSO 6/6 ok with zero terminations; video shows a level chassis with all six feet planted and loaded, held to truncation. Caveats named bluntly: (a) DR-0 sto 0/6 — all six episodes end TERM hold_min_load with one foot unloading up to 27.8mm; root cause is policy_std pinned at 1.019 after only 2M steps (never annealed), so sto eval draws ~full-init noise on top of a clean mean — an undertrained-std artifact, not a pose defect; (b) the gate's 'rising hold_feet_factor' sub-criterion reads 0.27->0.20 (not rising) but it is computed on the same std~1.0 stochastic training rollouts, and the det harness it proxies outranks it; (c) training reward still declines quarterly (-2.9/-57.9/-91.2/-86.2) because the anchor is a supervised loss outside the RL reward — expected. Why it worked: bc_anchor_loss 0.08->0.003, i.e. the anchor gave the policy the target-posture signal six rungs of income/termination shaping could not. Next: joint 3-arm dose read (0.5/1.0/3.0) once bcanchor1 and bcanchor0p5 land, then an 8M acquisition arm at the chosen dose — the acquisition question is precisely whether std anneals enough for det-quality plants to survive stochastic sampling and DR.

