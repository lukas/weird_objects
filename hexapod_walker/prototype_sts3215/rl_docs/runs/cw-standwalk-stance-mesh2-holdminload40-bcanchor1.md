# cw-standwalk-stance-mesh2-holdminload40-bcanchor1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-25T12:11:03+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-holdminload40

**wandb_id**: 0q4hurrk

**hypothesis**: Rung-7: a direct joint-space imitation target (train.bc_anchor_coef, already-built HOLD/TRACK BC-anchor machinery targeting self._q_nom, the settled six-foot plant pose captured at episode reset) supplies the missing 'what does the honest target posture look like' signal that six rungs of pure income/termination shaping (rung1 total-collapse pricing grid, rung2-4 hold-only income variants, rung5 height-drop term, rung6 min-load term) could not -- does adding it, single lever, to the holdminload40 recipe finally produce an honest six-foot plant instead of the chassis-hover-at-the-40mm-boundary basin every prior arm converged to?

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. 2M canary, hold DR-0 det+sto n=6+6. PASS: >=4/6 det valid_plant (six-foot, cur_p95<=1.5A, no OC/tilt/hold_low_height/hold_min_load terms) AND W&B hold_feet_factor/hold_load_factor trending up (not flat-zero) by 2M. PARTIAL (continuation-worthy per 08-21): 1-3/6 valid_plant or a clearly-improving-but-incomplete trend -- fund an 8M continuation. FAIL: 0/6 valid_plant with the same height_err-pinned-at-40mm/cur_max_a-pinned-at-2.64A signature as rung5/rung6 -- bc_anchor mechanism itself refuted for this task, escalate to operator (from-scratch mesh hold discovery may need a stronger structural fix, e.g. RSI-anchored reset every N steps mid-episode, or accepting the primitive-family footlow2 champion as the interim teacher until a full from-scratch mesh recipe is found).

**verdict**: CANARY PASS (mechanism-health scope; no skill-acquisition claim at 2M). The BC pose anchor broke the seven-rung mesh-hold deadlock: with joint-space imitation of the reference stance (bc_anchor_coef=1.0) the mesh robot holds a LEVEL SIX-FOOT STANCE for the first time. Evidence: DR-0 det hold 6/6 valid_plant, cur_p95 0.75A, zero terminations, frame strips show a level upright plant held motionless (vs rung-5/6 crouched hover with height_err pinned 40mm / current pinned 2.64A). W&B: height_err fell 12.5->4.8mm over 2M, hold_feet/load_factor non-zero and recovering (0.12->0.19), bc_anchor_loss converged 0.164->0.005. Sto is 0/6, all hold_min_load: under action noise the stance creeps (~167mm drag, fwd 0.02-0.04m) until a foot unloads -- a robustness gap for the acquisition arm, NOT a mechanism fault; the min-load lever is doing exactly its job. Next: joint 3-arm dose read (0.5/1.0/3.0 -- siblings bcanchor0p5/bcanchor3 still landing) picks the dose that funds the 8M acquisition arm; 1.0 is the first confirmed passer, so acquisition proceeds even if the siblings fail.

