# cw-standwalk-stance-mesh2-stage2-dualbc1-anchor14-walkretaincoef1-rescue-s1-acq8m

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-27T21:29:26+00:00

**pod**: hexapod-mjx-train-3

**steps**: 8000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor14-walkretaincoef1-rescue-s1

**wandb_id**: m4q7i54x

**hypothesis**: Plain English: seed1 is the catastrophe target that just got rescued at 2M (0/6->6/6 gait_valid, zero sacrificed legs) -- does the rescue keep improving with more training (progress_ratio climbing above the 0.10-0.16 canary band, slip/m falling) or does it plateau/relapse under a longer budget? Reward quarters showed the anchor4-class trough with only partial Q4 recovery, i.e. not flat, so per the 08-21 ruling this is continue-and-see. A relapse into the leg-sacrifice freeze under more steps would be new, important information that the coef=1.0 rescue is a fragile 2M-window effect, not a genuine fix.

**gate**: ACQUISITION READ (paired 2-seed call with the seed0 twin, not a fresh mechanism canary): compare 8M det walk DR-0 gate + own-DR against this run's own 2M canary snapshot (prog_ratio med 0.10 DR-0/0.16 own-DR, slip/m med 10.13 DR-0/6.81 own-DR, gait_valid 6/6 sac=[] both). PASS if BOTH seeds show gait_valid stays >=5/6 with zero/near-zero sacrificed legs AND progress_ratio improves over the 2M snapshot with slip/m flat-or-improving. PARTIAL if gait_valid holds but progress_ratio is flat. FAIL if gait_valid regresses below 5/6 or sacrificed legs reappear on either seed (the rescue relapses under more training) -- in that case do not re-fund this exact recipe, escalate to the reward/task-level audit.

**verdict**: ACQUISITION PASS (JOINT PASS with seed0): the 8M coef=1.0 walk-anchor continuation off the actual catastrophe-rescued seed keeps improving well past the 2M rescue canary, on the exact pre-registered clauses -- and does NOT relapse into the anchor4-class leg-sacrifice freeze under more budget, the specific risk this arm was funded to test. Evidence (logs/ckpt_eval/cw_standwalk_stance_mesh2_stage2_dualbc1_anchor14_walkretaincoef1_rescue_s1_acq8m_{gate,owncfg}/report.json, pulled this cycle after fixing the podeval self-match bug that had blocked the session pass): DR-0 det walk gait_valid 6/6, sacrificed_legs=[] all 6 episodes, progress_ratio med 0.541 (vs 2M canary 0.10), slip/m med 1.72 (vs 10.13); own-DR(0.5) det walk gait_valid 6/6, sac=[], progress_ratio med 0.544 (vs 0.16), slip/m med 1.92 (vs 6.81) -- both clauses clear with a huge margin, matching seed0 almost exactly (0.541 vs 0.555 DR-0, 0.544 vs 0.547 own-DR). walk_startjitter mirrors walk (prog ~0.52-0.55, gv 6/6 both DR). Video (walk_det contact sheet + per-episode strips) shows a genuine alternating six-leg gait, upright, no drag/persistent lift -- the SAME lineage that was a total 0/6 gait_valid freeze pre-rescue now sustains and improves the recovery at 4x the original budget. Why this matters: the joint call this gate was pre-registered for is now closed cleanly -- coef=1.0 is a genuine, budget-compounding fix, not a fragile 2M-window artifact, on BOTH the seed that was already healthy (seed0) and the seed that was the actual catastrophe (seed1). Caveat carried forward (pre-existing, not new, same as seed0s verdict): hold/lower terminations persist on stochastic passes matching the tracked flat-start-rise/lower residual; this acquisition gate was pre-registered walk-only, and the full session-level DONE gate is not re-attempted here (session pass returned INCOMPATIBLE, expected obs-contract mismatch for this dual-core-GRU checkpoint vs the deployed stance partner). Next: both anchor14-walkretaincoef1-rescue-acq8m checkpoints (seed0 + seed1) are now the leading stage-2 walk-quality candidates; the standing lower/hold residual (rung-5 segment-timing lineage) is the remaining lever before a real composed session attempt.

