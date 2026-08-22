# cw-dep-bcgait4-phasedir9-seed17

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: DONE

**created**: 2026-08-22T12:49:14+00:00

**pod**: hexapod-mjx-train-3

**steps**: 2000000

**parent**: cw-dep-bcgait4-phasedir9-stdanneal

**wandb_id**: h9nrxse9

**hardware_ready**: False

**hypothesis**: phasedir9-stdanneal (fresh BC-clone restart + forced log_std anneal 0.135->0.041) was the best arm of the whole phasedir lineage (progress 0.873x clone, near-PASS, zero falls 24/24) but its own continuation (-cont1, +4M steps) came out WORSE on every clone-relative axis (progress 0.655-0.706x, slip 1.59-1.74x, speed below the 0.06 floor) despite reward partially recovering after a mid-run collapse -- an anomaly now flagged DIG-IN rather than snap-verdicted. Before spending more budget on that disputed continuation, this arm asks a simpler, orthogonal question: is pd9's 2M near-pass itself reproducible, or was it seed luck? Identical config (pd8 reward stack + --log-std-final -3.2/--log-std-anneal-frac 0.6 + overspeed ref floor), same 2M budget, ONLY --seed changed 13->17. Prediction-if-true (reproducible): a second seed also lands zero-falls/gait-6-6 with progress in the same 0.80-0.95x clone range -- pd9's recipe is a real, repeatable near-pass and the lineage is worth the cont1 dig-in's time. Prediction-if-false: seed17 lands far below pd9 (e.g. back near pd8's 0.77x or worse) or shows a different failure mode -- pd9 was partly seed luck and the dig-in must weight that in before trusting any single continuation's regression or progress as signal.

**gate**: At 2M, DR-0, same clone-relative forward panel as pd9 (logs/ckpt_eval/phasedir3_clone_control_gate). Report vs pd9-stdanneal's own numbers (progress 0.873x, slip 1.08x, speed 0.06-0.063, zero falls) on every axis, not just PASS/FAIL against the 0.9x cap. PASS-reproduced = zero falls, gait_valid 6/6, progress within ~0.1x of pd9's 0.873x (i.e. >=0.75x clone), slip <=1.2x clone. Either way this is evidence for the pd9-cont1 dig-in, not a standalone lineage promotion decision -- do not respec a 3rd continuation from this arm without the dig-in's root cause first.

**verdict**: FAIL-as-reproduction: pd9-stdanneal's 2M near-pass (progress 0.873x clone, slip 1.08x) does NOT reproduce on seed17 -- det gate progress 0.727x clone (0.56/0.77), slip 1.27x clone (2.40/1.89), i.e. BELOW pd8's own 0.766x/1.254x, matching the pre-registered prediction-if-false almost exactly (lands far below pd9, back near pd8 or worse). Zero falls, gait_valid 6/6 both det+sto, clean video (six legs cycling). Resolves STATUS Next-item-3's fork: pd9's near-pass reads as partly seed luck, not a repeatable recipe -- do not spend further budget assuming pd9 generalizes. Anomaly for dig-in: W&B train/bc_anchor_loss_walk is tiny (0.00005-0.0007, essentially converged) and env/walk_anchor_frac is already high (0.80-0.93) on BOTH seed13 and seed17, yet realized swing_s_mean is ~30% slower than the teacher/clone (0.34-0.36s vs clone's 0.25-0.27s) on both -- the BC-anchor+phase-lock supervision reads as tightly tracking its target, but realized cadence still drifts, which is the literal BC-anchor/phase-lock family boundary pd8/pd9 flagged and never traced. Needs an on-pod per-tick trace of bc_target cadence vs realized policy cadence vs raw (non-phase-locked) teacher cadence before any anchor-dose or phase_hz change.

