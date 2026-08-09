# cw-walk-dr05-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T11:26:21+00:00

**pod**: hexapod-mjx-train-3

**steps**: 20000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_anchorgate.zip

**wandb_id**: zeban4qc

**hardware_ready**: no

**hypothesis**: RETRY of OPERATOR arm cw-walk-dr05 (DEAD at init, 0 steps: parent ckpt missing on pod — infra fault, not a result; retried once per the DEAD rule with the checkpoint pushed + md5-verified). Original hypothesis unchanged: HIGHER DR — the champion lineage trains at DR 0 and is only EVALED at DR 1.0; training under model-field DR 0.5 tests whether the gait is robust or brittle-tuned. One config change off champion ppo_goal_cw_walk_anchorgate.zip md5 35234ddc: --no-dr -> --dr-scale 0.5. If-true: DR0.5 det+sto zero terminations with gait retained and DR0 retention intact. If-false: gait degrades under DR (terminations/gait_valid failures) — robustness is a separate training rung to climb before hardware. Strongest alternative: DR training masks slip changes via physics variation — slip read only as observation vs champion 1.24.

**gate**: DR0.5 det+sto 6/6: zero terminations, gait_valid, det slip/m <= 1.24; DR0 det retention 6/6 fwd >= 0.55; frames watched det+sto

**verdict**: FAIL (gate): DR0.5 det slip/m 1.56 agg vs gate <=1.24 (one 3.66 blowout ep; median ~1.25) and sto gait_valid 5/6 (leg-5 sacrifice, slip/m 23). BUT 0 terminations at DR0.5 and 10/12 clean eps — gait largely survives DR0.5; DR0 retention PASS (det 6/6, prog_ratio 1.00, slip/m 1.06, along 0.74-0.79). Robustness mostly real, stochastically brittle; paddle transport persists. Frames watched (DR0.5 det): level body, all six feet cycling, champion-style creep. Not hardware-ready.

