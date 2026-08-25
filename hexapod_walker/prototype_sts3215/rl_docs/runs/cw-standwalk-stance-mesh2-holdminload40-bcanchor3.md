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

**verdict**: CANARY PASS -- HISTORIC first-ever DET-mode PASS in the 7-rung standwalk mesh hold campaign. Evidence: DR-0 det gate 6/6 valid_plant, ZERO terminations, height_err_end_mm 0.0-0.7, cur_max_a 1.10A / cur_p95_a 0.53A (well under every prior rung's 2.64A pin), roll_class clean (peak 0.5deg), return 1379 (near the bank's own honest-quiet-stand reference value ~1471), duty_cycle ~0.98-1.0 on all six legs, video contact sheet shows a level symmetric six-foot stand held the whole episode -- qualitatively different from every prior rung's chassis-hover/crab-crouch/belly-flop basin. The mechanism (train.bc_anchor_coef=3.0, the already-built HOLD/TRACK BC-anchor supervising the policy's mean action toward self._q_nom every tick) WORKS: a direct joint-space imitation target succeeded where six rungs of pure income/termination pricing did not. Residual gap (not gate-blocking for this canary, but the next question): sto mode (stochastic action sampling) is still 0/6, current-pinned at 2.64A -- the policy's own unannealed exploration noise (log-std-init=0, std~1.0) appears to be what destabilizes an otherwise-solved plant under sampling. Own-DR(0.2) not yet read at this canary tier. Funding an 8M std-anneal continuation (single lever, --log-std-final=-4.0) to test whether the same fix that closed the analogous joystick-track sto gap (stotight45) also closes this one.

