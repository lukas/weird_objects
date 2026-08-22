# cw-dep-bcgait4-phasedir9-longrun13

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-22T13:42:24+00:00

**pod**: hexapod-mjx-train-3

**steps**: 4000000

**parent**: cw-dep-bcgait4-phasedir9-stdanneal

**wandb_id**: jrvg7v6p

**hypothesis**: pd9's 2M near-pass (progress 0.873x clone, slip 1.08x, zero falls) read as UNDERTRAINED (reward + drag-charge still moving at cutoff) and did NOT reproduce on seed17 at the same 2M budget (fell to 0.727x/1.27x, at/below pd8's own level) -- this arm asks whether the SAME anneal schedule just needs more time in the converged low-noise regime, not a different mechanism or a banned continuation-of-a-converged-checkpoint. Identical stack+seed(13) as pd9-stdanneal, ONLY --steps 2M->4M and --log-std-anneal-frac 0.6->0.3 so log_std still finishes annealing at the SAME absolute step (~1.2M, unchanged) but the policy now trains 2.8M steps at converged std=0.041 instead of ~800k. Prediction-if-true: progress/slip keep improving past the old 2M mark and land >=0.9x/<=1.15x clone by 4M. Prediction-if-false: reward and gate metrics plateau flat by ~2.5-3M with no further movement -- budget is exonerated too, strengthening this cycle's DIG-IN flag on the BC-anchor/phase-lock family boundary (train/bc_anchor_loss_walk already near-zero/converged on both seed13 and seed17 while realized cadence still runs ~30% slower than the teacher -- a target-tracking-vs-realized-behavior gap, not a supervision-strength one).

**gate**: Same clone-relative forward panel as pd9 (logs/ckpt_eval/phasedir3_clone_control_gate, DR-0 det+sto). Report progress/slip/speed/falls at the run's own final checkpoint AND note the mid-run (~2M) eval for a trend read, not just cap comparison. PASS = zero falls, gait_valid 6/6, progress >=0.9x clone, slip <=1.15x clone, speed in [0.06,0.096]. Any other outcome: report whether metrics are STILL MOVING at 4M (continue is still available under 08-21) or FLAT (budget exonerated, hands off to the phase-lock dig-in).

