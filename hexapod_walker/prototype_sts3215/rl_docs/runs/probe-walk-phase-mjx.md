# probe-walk-phase-mjx

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T07:29:12+00:00

**pod**: hexapod-mjx-train-1

**steps**: 1000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_parkstart_mjx.zip

**hypothesis**: Mechanism probe for the MJX obs-pad-transplant port + phase package on the GPU stack: warm start from the champion across a +2 obs widening (phase clock) trains without error; transplant log line present; no NaN/tracebacks; fps in band.

**gate**: survives 1M steps; '[mjx-train] warm start ... (+2 obs-pad transplant)' in log; periodic eval runs; no tracebacks

**verdict**: PASS: transplant line '72 -> 74 dims; zero-padded first-layer columns in [policy_net.0, value_net.0]' present; 1M steps completed in 207s; 0 tracebacks/NaN; periodic eval walk err 0.029 m/s (parent band). MJX obs-pad-transplant + phase package validated on the GPU stack; gates cw-walk-phaseprior.

