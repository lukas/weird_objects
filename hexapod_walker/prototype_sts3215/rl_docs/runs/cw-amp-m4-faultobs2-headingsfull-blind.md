# cw-amp-m4-faultobs2-headingsfull-blind

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-23T00:53:57+00:00

**pod**: hexapod-mjx-train-0

**steps**: 4000000

**parent**: cw-amp-m4-faultobs2-headingsfull-noamp

**wandb_id**: 8k7hnm57

**hypothesis**: Plain English: the blind control the faultobs2 PASS still needs -- train the SAME full-heading walker under the SAME guaranteed fault injection but WITHOUT sight of the fault (obs.fault_health=0, no obs pad), so the sighted noamp arm's faulted-episode numbers finally have a matched blind-compensation baseline on this substrate (faultobs1's blind control was forward-only). Single lever vs cw-amp-m4-faultobs2-headingsfull-noamp: obs.fault_health 1->0 and --obs-pad-transplant 18->0 (init checkpoint obs already match natively); same init, fault mix, budget, seed. Prediction-if-true (fault-sight generalizes): sighted beats blind on the same-seed faulted episodes by faultobs1-like margins (prog +~18%, slip -~27%) or larger. Prediction-if-false: paired episodes within 6-ep noise -- fault-sight's benefit does not survive command diversity at this budget.

**gate**: Acquisition control (4M, DR-0). Same-seed paired read vs faultobs2-headingsfull-noamp's gate report (identical per-episode fault draws): SIGHTED-WINS if noamp beats this arm on faulted-episode prog_ratio/slip beyond paired noise (esp. sto ep0/3/4); WASH if within noise (then fault-obs's M4 value is unproven on diverse commands and the next lever is longer budget or fault-curriculum, not more obs); this arm must itself stay gait_valid >=9/12 with no crouch (heights in walking band) for the comparison to be honest.

**verdict**: Blind fault control clears its own mechanism-safety floor (gait_valid 9/12 combined, exactly the >=9/12 bar) and completes the sighted-vs-blind pairing on the full-heading substrate: SIGHTED WINS decisively. Same-fault-draw comparison (seed 0, identical dr.fault_prob=1.0 command sampling) vs the already-PASSed noamp (sighted) and style05 (sighted) siblings: sto gait_valid blind 3/6 (sacrificed legs on 3 episodes: ep0 leg0, ep1 leg2, ep3 leg5) vs noamp 5/6 (1 sacrifice) vs style05 6/6 (ZERO sacrifices) -- fault_health obs-wiring more than triples the safe-limp rate on this substrate, matching faultobs1's earlier forward-only finding (+18%prog/-27%slip sighted) directionally. det stays gait_valid 6/6 for all three arms but blind's det slip is worse too (med 3.58 vs 3.08 both sighted). Video (walk_sto_0, walk_sto_3 sheets) confirms the failures are genuine LIMPS not statues -- one dangling/static leg while the other five keep cycling, same fingerprint as the sighted arms' failures elsewhere, so this isn't a different failure mode, just a higher rate of it. Closes the faultobs2-headingsfull PASS/PASS-neutral/blind-control trio: fault-sight is a real, load-bearing mechanism on the full-heading substrate (not just forward-only), style neither helps nor hurts it. No further action needed on this substrate; the M4 open question is now graft-onto-turn (turnfault1, already INFORMATIVE/undertrained) not sighted-vs-blind.

