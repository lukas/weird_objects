# cw-standwalk-stance-mesh2-holdterm40-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-25T09:45:24+00:00

**pod**: hexapod-mjx-train-1

**steps**: 6000000

**parent**: cw-standwalk-stance-mesh2-holdterm40

**wandb_id**: jg6l3bf7

**hypothesis**: Plain English: seed replicate of holdterm40 -- with the belly absorbing state terminated+priced, does a SECOND seed also stay in (or return to) the six-foot stance, or does the fix only work for one optimization path? Seeds under this recipe family have found DIFFERENT basins every wave (seed0 rear-up OC, seed1 belly-freeze), so a single-seed pass would be weak evidence for the rung; byte-identical to holdterm40 except seed=1. Prediction-if-true: same PASS shape as seed 0 (>=10/12 valid plant at 6M). Prediction-if-false: seed 1 lands in a different (terminating) escape basin while seed 0 plants -- mechanism helps but the recipe is still seed-fragile; joint read decides whether rung-6 is a DR ramp or the min-load exit trigger.

**gate**: Same as holdterm40: hold panel at 6M, pod_eval hold DR-0 det+sto n=6+6, >=10/12 survive with zero OC/tilt/hold_low_height terms, six-foot stance (valid_plant or all-leg duty>=0.9), cur_p95<=1.0A; own-DR(0.2) alongside. Joint pass-rate read with seed 0 (n=2).

**verdict**: Ledger housekeeping: seed-1 twin of rung-5 holdterm40, deferred to 'another cycle's read' at launch (08-25) and never closed. Confirms the already-verdicted rung-5 FAIL cross-seed: 12/12 hold episodes (det+sto, own-DR 0.2) terminate hold_low_height, reward falling not rising across the run (-102/-365/-521/-405), matching seed-0's pinned-at-40mm-drop-line hover signature exactly. No new information -- rung-5 (height-drop lever alone) was already superseded by rung-6 (min-load termination, also FAIL) and then rung-7 (BC-anchor, PASS on hold, closed the hold rung 08-25 13:1x). Closing this stale ledger entry so it stops showing as untriaged; no track-story change.

