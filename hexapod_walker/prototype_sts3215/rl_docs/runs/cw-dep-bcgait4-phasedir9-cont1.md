# cw-dep-bcgait4-phasedir9-cont1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: DONE

**created**: 2026-08-22T12:18:29+00:00

**pod**: hexapod-mjx-train-3

**steps**: 4000000

**parent**: cw-dep-bcgait4-phasedir9-stdanneal

**wandb_id**: q4aiwool

**hardware_ready**: False

**hypothesis**: The std-annealed pd9 arm was UNDERTRAINED, not misaligned: at 2M steps it already had zero falls (24/24 episodes), gait_valid 6/6, slip/dir_err/speed all inside the clone-relative gate, and only progress missed (0.873x clone vs 0.9x cap, best of the whole phasedir lineage vs pd8's 0.770x) while reward/tick was still climbing steeply (-0.82 -> -0.02 in the last 400K steps) and the drag-stance charge that financed the old cheat gait was falling (peak -3.9 -> -0.7 late), not flat. This arm continues training from that exact checkpoint with the annealed-low std held constant (log_std already at the -3.2/0.041 target, so the inherited anneal schedule is a no-op hold, verified: _LogStdAnnealCb reads its start value from the loaded checkpoint's own log_std at construction time) for 4M more steps, same reward/env code, zero new mechanisms. Prediction-if-true: progress closes to >=0.9x clone with slip staying <=1.15x and zero falls -- first PASS in the phasedir lineage. Prediction-if-false-(i): progress keeps climbing but plateaus below 0.9x with drag charge flat near its current ~-0.7 (not falling further) -- the std-anneal lever is exhausted, not wrong; report the plateau value and stop financing this exact config, dig at the BC-anchor/phase-lock family boundary next (pd8 branch ii). Prediction-if-false-(ii): slip or falls regress as progress rises -- the extra progress is being bought back with drag/slip, i.e. re-discovering the old cheat as std settles; report the specific tradeoff and treat as MISALIGNED again (the loadslip-band step-function finding would generalize past std annealing after all). Prediction-if-false-(iii): reward/progress go flat immediately (no further climb) -- the apparent late-run climb was noise, not learning; treat pd9 as a genuine near-miss, no further budget on this exact checkpoint.

**gate**: Same clone-relative forward-panel gate as pd9 (logs/ckpt_eval/phasedir3_clone_control_gate, DR-0 det+sto, PASS requires ALL: zero falls + gait_valid 6/6; progress >=0.9x clone; slip/m <=1.15x clone; dir_err med <= clone+5deg; speed_mean in [0.06,0.096]), evaluated on the final checkpoint after +4M steps. VERDICT MUST additionally report: final policy std (expect held ~0.041, log_std_anneal/frac should read 1.0 throughout since the checkpoint loads already at the anneal target), env/reward_drag_stance trend across the continuation (expect flat-near-zero-to-slightly-negative, NOT growing more negative -- growing more negative would mean a new/bigger drag regime), and reward-per-tick trend (expect continuing to rise toward 0 then plateau). PASS -> this is the first passing phasedir arm: promote per STATUS Next item 4 (heading curriculum / wider command envelope). Fail branches: see hypothesis predictions (i)-(iii), each names its own next action. NO DOWNLOAD_ANSWER change from this run.

**verdict**: FAIL as a continuation (pd9 ITSELF stays UNDERTRAINED/near-pass on its own 2M merits, unaffected). +4M steps regressed hard on every clone-relative axis vs the pd9 checkpoint it started from: progress 0.655x/0.706x (DR0/ownDR det, was 0.873x), slip 1.737x/1.587x (cap 1.15x, was 1.08x PASS), speed 0.051/0.0565 (was 0.06-0.063 in-band, now below the 0.06 floor); zero falls preserved, gait_valid 6/6 preserved. W&B: ep_rew_mean crashed 27->-670 by ~1.4M then partially recovered to 152-213 by 4M -- never cleanly rising, a collapse-then-partial-recovery pattern, not steady climbing. ROOT CAUSE, already established this cycle by the concurrent phasedir9b dig-in (INIT-BASIN SELECTION, NOT CHECKPOINT RECENCY, CURRENT_TRUTHS 08-22): this reward stack's optimization-regime surface is ~flat across the honest and drag/degraded basins at annealed-low std, so continued gradient steps are not guaranteed to stay in a good basin -- they can wander to a worse one even starting from the lineage-best checkpoint. This is the same mechanism from the opposite direction (pd9b: bad init stayed bad; cont1: good init drifted worse). Per the newly adopted binding rule (re-init from a pre-cheat/pre-drift checkpoint for regime repairs, never continue-train an already-converged one), further budget on THIS checkpoint's continuation is not warranted. NO DOWNLOAD_ANSWER change. Next: cw-dep-bcgait4-phasedir9-seed17 (launched this cycle, fresh BC-clone restart, seed 13->17, same recipe) tests whether pd9's 0.873x near-pass reproduces at all before any further lineage investment.

