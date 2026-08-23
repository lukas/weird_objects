# cw-amp-m4-faultobs2-headingsfull-blind

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T00:53:57+00:00

**pod**: hexapod-mjx-train-0

**steps**: 4000000

**parent**: cw-amp-m4-faultobs2-headingsfull-noamp

**wandb_id**: 8k7hnm57

**hypothesis**: Plain English: the blind control the faultobs2 PASS still needs -- train the SAME full-heading walker under the SAME guaranteed fault injection but WITHOUT sight of the fault (obs.fault_health=0, no obs pad), so the sighted noamp arm's faulted-episode numbers finally have a matched blind-compensation baseline on this substrate (faultobs1's blind control was forward-only). Single lever vs cw-amp-m4-faultobs2-headingsfull-noamp: obs.fault_health 1->0 and --obs-pad-transplant 18->0 (init checkpoint obs already match natively); same init, fault mix, budget, seed. Prediction-if-true (fault-sight generalizes): sighted beats blind on the same-seed faulted episodes by faultobs1-like margins (prog +~18%, slip -~27%) or larger. Prediction-if-false: paired episodes within 6-ep noise -- fault-sight's benefit does not survive command diversity at this budget.

**gate**: Acquisition control (4M, DR-0). Same-seed paired read vs faultobs2-headingsfull-noamp's gate report (identical per-episode fault draws): SIGHTED-WINS if noamp beats this arm on faulted-episode prog_ratio/slip beyond paired noise (esp. sto ep0/3/4); WASH if within noise (then fault-obs's M4 value is unproven on diverse commands and the next lever is longer budget or fault-curriculum, not more obs); this arm must itself stay gait_valid >=9/12 with no crouch (heights in walking band) for the comparison to be honest.

