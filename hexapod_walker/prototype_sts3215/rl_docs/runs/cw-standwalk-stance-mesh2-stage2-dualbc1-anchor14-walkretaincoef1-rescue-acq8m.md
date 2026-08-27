# cw-standwalk-stance-mesh2-stage2-dualbc1-anchor14-walkretaincoef1-rescue-acq8m

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-27T21:27:13+00:00

**pod**: hexapod-mjx-train-1

**steps**: 8000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor14-walkretaincoef1-rescue

**wandb_id**: 5blr9syj

**hypothesis**: Plain English: now that the lighter walk-anchor dose (coef=1.0) is confirmed to rescue seed1's catastrophe AND keep seed0 healthy at 2M, does that rescue keep improving (progress_ratio up, slip down) with more training, or does it plateau at the ~0.10-0.18 prog_ratio band this canary showed? Both seeds' reward quarters still show the anchor4-class trough-then-partial-Q4-recovery shape (not flat), so per the 08-21 ruling this is a continue-and-see, not a re-launch-fresh. If prog_ratio rises well past 0.18-0.2 with slip staying low and gait_valid/sac stay clean at 8M, this recipe is promotable toward the campaign's real walking-quality bar. If it plateaus flat with reward also flat, that is new information the mechanism has a hard ceiling regardless of budget, and the reward/task-level audit (flagged by seed0's cross-recipe coincidence with anchor13's control) becomes the next lever, not more steps.

**gate**: ACQUISITION READ (paired 2-seed call, not a fresh mechanism canary): compare 8M det walk DR-0 gate + own-DR against this exact run's own 2M canary snapshot (prog_ratio med 0.10 DR-0/0.16 own-DR, slip/m med 10.13 DR-0/6.81 own-DR, gait_valid 6/6 sac=[] both). PASS if BOTH seeds show gait_valid stays >=5/6 with zero/near-zero sacrificed legs AND progress_ratio improves (not just noise) over the 2M snapshot with slip/m flat-or-improving. PARTIAL if gait_valid holds but progress_ratio is flat (mechanism stable, not yet a quality win -- budget question, consider a further continuation). FAIL only if gait_valid regresses below 5/6 or sacrificed legs reappear on either seed (the anchor4-class catastrophe returns under more training) -- in that case do not re-fund this exact recipe, escalate to the reward/task-level audit.

