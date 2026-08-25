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

**verdict**: CANARY PASS -- reproduces bcanchor3's det-mode breakthrough at 3x lower dose (coef=1.0): DR-0 det gate 6/6 valid_plant, zero terminations, height_err_end_mm 0.0mm, cur_max_a 1.34A / cur_p95_a 0.75A, roll_class clean, return 1411 (matches bcanchor3's ~1379, both near the honest-quiet-stand bank value ~1471), duty_cycle ~0.98-1.0 six legs, video confirms a level six-foot stand. Confirms the bc_anchor mechanism is dose-robust across at least a 3x range (1.0-3.0), not a lucky single dose. Same residual gap as bcanchor3: sto mode 0/6, current-pinned 2.64A (unannealed std~1.0 exploration noise). No separate continuation needed -- the coef=3.0 sibling's std-anneal acquisition arm already tests the shared fix; if it PASSes, this dose (1.0, cheaper anchor weight) becomes the preferred base for composing rise/lower later since a lighter imitation weight leaves more room for other modes to shape the same policy.

