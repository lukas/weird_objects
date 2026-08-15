# cw-arch-tf-r1-hard3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: VERDICTED

**created**: 2026-08-15T17:48:22+00:00

**pod**: hexapod-mjx-train-0

**steps**: 40000000

**parent**: cw-arch-tf-r1-hard2-r1

**wandb_id**: 3m2dn43f

**hardware_ready**: False

**hypothesis**: Plain-English: keep training the causal-transformer walker for another 40M steps to see if it keeps closing the foot-slip gap to the older MLP champion, or whether it has already plateaued. hard1 (40M, fresh) matched the MLP champion (r7) at budget parity; hard2-r1 (an unplanned +40M continuation, backfilled to the ledger this cycle) then CONSOLIDATED the same walk -- DR0 slip/m med improved 1.60->1.21 det / 1.53->1.32 sto with prog_ratio held >=0.85 and zero falls/sacrificed legs across 48 episodes -- a real but partial gain (own-cfg DR0.5 slip did not improve the same way). This mirrors the sibling hist16-r7 MLP line's own history (r7->c1 improved, prompting further continuation before c4 finally plateaued and continuations stopped per the two-flat-continuations rule). hard3 is that same next check for the transformer line: does a SECOND continuation keep buying slip economy, or does it flatten like r7's c4 did? One variable: step count (pure continuation), same task/reward/DR/command recipe as hard1/hard2-r1.

**gate**: PASS = det+sto gait_valid 6/6, zero sacrificed legs, zero falls, prog_ratio med >=0.85 (matching hard2-r1's 1.23/1.09), AND slip/m med at or below hard2-r1's own DR0 numbers (1.21 det/1.32 sto) -- continued or held economy. FLAT (slip/m med within +/-10% of hard2-r1's, no clear direction) = the transformer line has plateaued like r7's c4 did; per the two-flat-continuations rule, do NOT queue a hard4 step-budget continuation -- the next lever would be contact/current pricing (CURRENT_TRUTHS open problem 1) or a full-skill (rise/hold/lower) extension, not more steps. REGRESSION (slip/m med >10% worse, or gv<6/6, or any fall/sacrificed leg) = STOP, do not extend further, root-cause before any next arm.

**verdict**: PASS (2nd +40M continuation, same recipe): further step budget keeps buying slip economy for the transformer trunk, but the gain is now small -- DR0 gate det/sto gait_valid 6/6, 0 term/0 sacrificed legs both DR0 and own-cfg DR0.5, prog med 1.19/1.07 (bar >=0.85, matching hard2-r1's 1.19-1.23/1.07-1.08), slip/m med 1.17/1.28 (was hard2-r1's 1.21/1.32 -- ~3% further drop, both det+sto, both DR0 and own-cfg DR0.5 all moved the same direction); roll clean (peak 2deg/0.4-0.9deg tail DR0; 8deg/1.0-1.1deg tail under DR0.5 injection). Contact sheets (both DR0 gate and DR0.5 own-cfg, all 6 det episodes each) show clean six-leg cycling, no flag-leg/drag/parked-leg. PASS by the pre-registered gate's letter (slip at-or-below parent, clear direction not noise), but improvement dropped from hard1->hard2-r1's 14-27% to hard2-r1->hard3's ~3% -- diminishing returns, approaching the plateau shape the sibling hist16-r7 line eventually hit. Decision: do not chase a 4th walk-only step-budget rung purely for a shrinking slip percentage right now; deprioritizing further walk-only continuations in favor of the gate's own named next lever (full-skill rise/hold/lower extension) as the next real question for this architecture. Evidence: logs/ckpt_eval/cw_arch_tf_r1_hard3_{gate,owncfg}.

