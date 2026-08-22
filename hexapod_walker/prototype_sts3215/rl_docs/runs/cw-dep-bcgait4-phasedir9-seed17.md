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

**verdict**: FAIL-as-reproduction (unchanged): pd9's 2M near-pass did not reproduce on seed17 (progress 0.727x clone, slip 1.27x; zero falls, gait 6/6). DIG-IN RESOLVED (08-22 per-tick trace, logs/ckpt_eval/pd9seed17_bc_cadence_trace + rl_move/sim/trace_bc_cadence.py): NO cadence gap exists. Realized coxa cycle period 0.76s == bc_target 0.76s == phase-obs clock 0.75s for the policy, the clone AND the raw un-phase-locked teacher (period is 0.75s by construction: TripodGait.period=0.75 == 1/walk_phase_hz=1.333). The '~30% slower swing_s_mean' premise was a CONTACT-SEGMENTATION ARTIFACT: the clone's 0.25s 'swing' comes from double lift-offs per cycle (24-36 lifts/leg per 15s vs ~19 gait cycles; lift-to-lift median 0.44s vs everyone else's 0.76s) -- split swings, not a faster gait. The policy single-swings cleanly (19-24 lifts/leg) at 0.372s vs the raw teacher's realized 0.345s (+8%, minor). Anchor supervision is honest in the eval regime too: policy-vs-bc_target action MSE 0.00136 (the clone's own is 4x worse, 0.00549), realized-vs-target xcorr ~1.00 at ~3-tick (0.12s) servo lag, same lag as the clone. VERDICT ON THE LEVER: bc_anchor_coef dose, walk_phase_hz, and the phase-lock plumbing are ALL exonerated -- do not spend arms there. The residual det-gate deficit is loaded-foot slip during stance at matched gait timing/stride/duty (gate slip med 2.85/m vs clone 1.89), i.e. the already-pinned slip-financed-progress family plus the seed-basin lottery (longrun17's det PASS vs longrun13's regression on the identical recipe). hardware-ready: no.

