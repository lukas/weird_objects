# cw-arch-tf-r1-hard3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-15T17:48:22+00:00

**pod**: hexapod-mjx-train-0

**steps**: 40000000

**parent**: cw-arch-tf-r1-hard2-r1

**wandb_id**: 3m2dn43f

**hypothesis**: Plain-English: keep training the causal-transformer walker for another 40M steps to see if it keeps closing the foot-slip gap to the older MLP champion, or whether it has already plateaued. hard1 (40M, fresh) matched the MLP champion (r7) at budget parity; hard2-r1 (an unplanned +40M continuation, backfilled to the ledger this cycle) then CONSOLIDATED the same walk -- DR0 slip/m med improved 1.60->1.21 det / 1.53->1.32 sto with prog_ratio held >=0.85 and zero falls/sacrificed legs across 48 episodes -- a real but partial gain (own-cfg DR0.5 slip did not improve the same way). This mirrors the sibling hist16-r7 MLP line's own history (r7->c1 improved, prompting further continuation before c4 finally plateaued and continuations stopped per the two-flat-continuations rule). hard3 is that same next check for the transformer line: does a SECOND continuation keep buying slip economy, or does it flatten like r7's c4 did? One variable: step count (pure continuation), same task/reward/DR/command recipe as hard1/hard2-r1.

**gate**: PASS = det+sto gait_valid 6/6, zero sacrificed legs, zero falls, prog_ratio med >=0.85 (matching hard2-r1's 1.23/1.09), AND slip/m med at or below hard2-r1's own DR0 numbers (1.21 det/1.32 sto) -- continued or held economy. FLAT (slip/m med within +/-10% of hard2-r1's, no clear direction) = the transformer line has plateaued like r7's c4 did; per the two-flat-continuations rule, do NOT queue a hard4 step-budget continuation -- the next lever would be contact/current pricing (CURRENT_TRUTHS open problem 1) or a full-skill (rise/hold/lower) extension, not more steps. REGRESSION (slip/m med >10% worse, or gv<6/6, or any fall/sacrificed leg) = STOP, do not extend further, root-cause before any next arm.

