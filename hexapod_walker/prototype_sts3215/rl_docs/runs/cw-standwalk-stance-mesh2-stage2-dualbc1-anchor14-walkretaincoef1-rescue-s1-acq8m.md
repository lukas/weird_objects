# cw-standwalk-stance-mesh2-stage2-dualbc1-anchor14-walkretaincoef1-rescue-s1-acq8m

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-27T21:29:26+00:00

**pod**: hexapod-mjx-train-3

**steps**: 8000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor14-walkretaincoef1-rescue-s1

**wandb_id**: m4q7i54x

**hypothesis**: Plain English: seed1 is the catastrophe target that just got rescued at 2M (0/6->6/6 gait_valid, zero sacrificed legs) -- does the rescue keep improving with more training (progress_ratio climbing above the 0.10-0.16 canary band, slip/m falling) or does it plateau/relapse under a longer budget? Reward quarters showed the anchor4-class trough with only partial Q4 recovery, i.e. not flat, so per the 08-21 ruling this is continue-and-see. A relapse into the leg-sacrifice freeze under more steps would be new, important information that the coef=1.0 rescue is a fragile 2M-window effect, not a genuine fix.

**gate**: ACQUISITION READ (paired 2-seed call with the seed0 twin, not a fresh mechanism canary): compare 8M det walk DR-0 gate + own-DR against this run's own 2M canary snapshot (prog_ratio med 0.10 DR-0/0.16 own-DR, slip/m med 10.13 DR-0/6.81 own-DR, gait_valid 6/6 sac=[] both). PASS if BOTH seeds show gait_valid stays >=5/6 with zero/near-zero sacrificed legs AND progress_ratio improves over the 2M snapshot with slip/m flat-or-improving. PARTIAL if gait_valid holds but progress_ratio is flat. FAIL if gait_valid regresses below 5/6 or sacrificed legs reappear on either seed (the rescue relapses under more training) -- in that case do not re-fund this exact recipe, escalate to the reward/task-level audit.

