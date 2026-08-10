# cw-dep-vref1-r1-actnoise

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T17:01:10+00:00

**pod**: hexapod-mjx-train-8

**steps**: 20000000

**parent**: cw-dep-vref1-r1

**wandb_id**: jbttknkp

**hardware_ready**: False

**hypothesis**: PROTECT THE NAMED HARDWARE CANDIDATE: vref1-r1 has been hardened against SENSING noise (encoder, gyro, tilt) and COMMAND-path faults (cmddrop, deadband, latency) but never ACTUATOR OUTPUT noise (dr.action_noise -- random jitter applied to the commanded action itself, modeling servo-internal control-loop imprecision distinct from anything on the sensing or command-transport side). Per P0 rule 3, k_current=0. If-true: own-cfg det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band -- output-noise composes free like every other single axis so far. If-false: action-level noise compounds with the loaded servo model's settling dynamics in a way sensing/transport noise did not -- flag as a real pre-attempt-#2 actuator risk.

**gate**: own-cfg det+sto 6/6 @15s gait_valid 12/12, 0 term, slip/m within vref1-r1's own band (0.89-1.36); frames watched det for flag-leg/skate

**verdict**: FAIL as an axis, but NO hardware risk (dig-in resolved the gate-vs-video tension). OBSERVATIONS: own-cfg gate eval injected dr.action_noise=0.02 ABSOLUTE (dr.* overrides apply post-scaling, at eval AND training; parent trained at 0.02x0.35=0.007) while the gate band 0.89-1.36 was measured on the parent's NOISE-FREE eval -- apples-to-oranges. Control A (parent under the IDENTICAL 0.02-noise eval, logs/ckpt_eval/cw_dep_vref1_r1_noisectl): the SAME 4 episode indices degrade (det/5 0.76/1.74, sto/0 0.67/2.05, sto/1 0.63/2.08, sto/4 0.30/4.93; sto slip med 1.51) -- the lineage fixed-seed fingerprint tips under any perturbation. Control B (this ckpt on the parent's clean DR0 eval, .../cw_dep_vref1_r1_actnoise_dr0clean): gv 12/12, 0 term, slip med 0.89 det / 1.05 sto, in-band, only the known sto/4 crater -- clean retention. INTERPRETATION: the sto crater wave was the eval config, not policy erosion; but exposure bought NOTHING -- under identical noise this ckpt is uniformly slightly WORSE than the parent on all 4 hard episodes (sto med 2.05 vs 1.51), matching the champion-line ladder where action-noise 0.08 exposure was NO-EFFECT. VERDICT: do NOT add elevated action_noise to the dep-line default; keep parent vref1-r1 for attempt #2 (not blocked: no falls/terms in any of the 36 episodes across all 3 evals, frames all clean six-leg gait). HYPOTHESIS STATUS: if-true refuted (not in band under own-cfg); if-false's pre-attempt-risk ALSO refuted (parent degrades the same under matched injection). GATE LESSON for injection axes (also applies to the open loaded1 dig-in class): own-cfg bands must be compared against the parent measured under the SAME injection (parent-baseline control), never the parent's clean band.

