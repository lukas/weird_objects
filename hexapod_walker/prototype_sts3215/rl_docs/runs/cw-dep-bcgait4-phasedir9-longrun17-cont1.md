# cw-dep-bcgait4-phasedir9-longrun17-cont1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-22T16:02:33+00:00

**pod**: hexapod-mjx-train-1

**steps**: 4000000

**parent**: cw-dep-bcgait4-phasedir9-longrun17

**wandb_id**: d3t26guk

**hypothesis**: Plain English: the only DONE-gate axis longrun17 still fails is the STOCHASTIC half of the 60s joystick session (sto slip 4.0 vs cap 2.9, dir_err 51-52deg vs allow 40; det PASSES every axis at both DR scales) -- and its training reward was still rising strongly at the 4M cutoff (quarters -245/-379/+104/+193), so the run is not converged. PPO's training objective IS the expected stochastic return under DR 0.35 with loadslip/drag pricing, i.e. exactly the sto behavior the gate measures, so a warm-started continuation (+4M from the longrun17 checkpoint, std HELD at the converged 0.041 via --warm-log-std-override=-3.2 so the old -2.0 override cannot reset it up; log-std-final -3.2 makes the anneal a constant) should keep improving sto slip/dir without disturbing the det pass. This is the 08-21-ruling continuation case: reward and eval (det gate 2M FAIL -> 4M PASS) improved together across budget on this exact seed/stack.

**gate**: Watcher's randomized 60s joystick DONE-gate (eval_joystick_gate, held-out stress_mix, n=12 det+sto, DR-0 + own-DR 0.35). PASS = det retains its full pass AND sto joins it: zero falls, sto slip/m <= 2.9, sto dir_err median <= 40deg, both DR scales. Prediction-if-true: sto slip falls from 4.0 toward det's 2.30 with dir_err under 40. Prediction-if-false: sto flat at ~4.0/51deg while training reward keeps rising -- a reward/eval divergence on the sto axis specifically, which redirects to a noise-robustness mechanism (train-time perturbation or sto-aware pricing) instead of more budget; also FAIL if det regresses below its own pass (continuation harmed the basin).

