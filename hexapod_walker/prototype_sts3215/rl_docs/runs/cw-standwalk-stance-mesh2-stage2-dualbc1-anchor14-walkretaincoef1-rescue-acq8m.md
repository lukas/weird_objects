# cw-standwalk-stance-mesh2-stage2-dualbc1-anchor14-walkretaincoef1-rescue-acq8m

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-27T21:27:13+00:00

**pod**: hexapod-mjx-train-1

**steps**: 8000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor14-walkretaincoef1-rescue

**wandb_id**: 5blr9syj

**hypothesis**: Plain English: now that the lighter walk-anchor dose (coef=1.0) is confirmed to rescue seed1's catastrophe AND keep seed0 healthy at 2M, does that rescue keep improving (progress_ratio up, slip down) with more training, or does it plateau at the ~0.10-0.18 prog_ratio band this canary showed? Both seeds' reward quarters still show the anchor4-class trough-then-partial-Q4-recovery shape (not flat), so per the 08-21 ruling this is a continue-and-see, not a re-launch-fresh. If prog_ratio rises well past 0.18-0.2 with slip staying low and gait_valid/sac stay clean at 8M, this recipe is promotable toward the campaign's real walking-quality bar. If it plateaus flat with reward also flat, that is new information the mechanism has a hard ceiling regardless of budget, and the reward/task-level audit (flagged by seed0's cross-recipe coincidence with anchor13's control) becomes the next lever, not more steps.

**gate**: ACQUISITION READ (paired 2-seed call, not a fresh mechanism canary): compare 8M det walk DR-0 gate + own-DR against this exact run's own 2M canary snapshot (prog_ratio med 0.10 DR-0/0.16 own-DR, slip/m med 10.13 DR-0/6.81 own-DR, gait_valid 6/6 sac=[] both). PASS if BOTH seeds show gait_valid stays >=5/6 with zero/near-zero sacrificed legs AND progress_ratio improves (not just noise) over the 2M snapshot with slip/m flat-or-improving. PARTIAL if gait_valid holds but progress_ratio is flat (mechanism stable, not yet a quality win -- budget question, consider a further continuation). FAIL only if gait_valid regresses below 5/6 or sacrificed legs reappear on either seed (the anchor4-class catastrophe returns under more training) -- in that case do not re-fund this exact recipe, escalate to the reward/task-level audit.

**verdict**: ACQUISITION PASS: the 8M coef=1.0 walk-anchor continuation keeps improving well past the 2M rescue canary, on the exact pre-registered clauses. Evidence (logs/ckpt_eval/cw_standwalk_stance_mesh2_stage2_dualbc1_anchor14_walkretaincoef1_rescue_acq8m_{gate,owncfg}/report.json, pulled this cycle after fixing the podeval self-match bug that had blocked the session pass): DR-0 det walk gait_valid 6/6, sacrificed_legs=[] all 6 episodes, progress_ratio med 0.555 (vs 2M canary 0.10), slip/m med 1.79 (vs 10.13); own-DR(0.5) det walk gait_valid 6/6, sac=[], progress_ratio med 0.547 (vs 0.16), slip/m med 1.94 (vs 6.81) -- both clauses (gait_valid held zero-sac AND progress_ratio improved with slip flat-or-better) clear with a huge margin, not noise. walk_startjitter mirrors walk (prog ~0.53-0.55, gv 6/6 both DR). Video (walk_det contact sheet + per-episode strips) shows a genuine alternating six-leg gait, upright, no drag/persistent lift. Why this matters: this is the best walk quality any stage-2 dual-BC-anchor arm on this mesh lineage has produced -- the lighter per-mode-decoupled coef=1.0 rescue dose is not just a 2M-window fluke, it keeps compounding with budget, refuting the "fragile short-window rescue" concern the launch hypothesis flagged. Caveat carried forward (pre-existing, not new): this same checkpoint still shows hold/lower terminations (hold_min_load, over_current) on stochastic passes at rates matching the known flat-start-rise/lower residual already tracked in STATUS -- the acquisition gate was pre-registered walk-only and this verdict stays in that scope; the full sit->rise->walk->lower session-level DONE gate is not yet re-attempted on this checkpoint (session pass returned INCOMPATIBLE, expected for this exotic dual-core-GRU obs contract vs the deployed stance partner). Next: promote this checkpoint as the leading stage-2 walk-quality candidate; the standing lower/hold residual (rung-5 segment-timing lineage) is the remaining lever before a real composed session attempt, not more walk-anchor budget.

