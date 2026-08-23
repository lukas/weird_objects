# cw-amp-m4-faultobs2-headingsfull-noamp

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-23T00:26:37+00:00

**pod**: hexapod-mjx-train-1

**steps**: 4000000

**parent**: cw-amp-m2-bcinit-sec5-noamp-headingsfull

**hypothesis**: Plain English: faultobs1 proved fault-sight helps (prog +18%/slip -27% vs blind) but only on a FORWARD-ONLY walker with a 2M smoke budget -- does the same fault_health-obs mechanism still help on the most command-diverse walker built so far (full-circle heading, incl. backward) at a real acquisition budget? Single lever vs faultobs1-noamp: --init-from swapped from the forward-only bcinit-sec5-noamp checkpoint to bcinit-sec5-noamp-headingsfull, goal.walk_heading_max_rad 0.0 -> -1 (matching that checkpoint's own trained envelope), steps 2M -> 4M (acquisition, not smoke -- the mechanism-safety question is already answered). Same dr.fault_prob=1.0 fault_mix, same obs.fault_health=1 + --obs-pad-transplant 18 wiring. Prediction-if-true: faulted-episode prog_ratio/slip improve over a blind-compensation control by a similar or larger margin than faultobs1's +18%/-27% (more command diversity in training should make the fault-sight signal more useful, not less). Prediction-if-false: no measurable delta or a regression -- heading diversity's own action-space demands crowd out the fault-compensation gradient at this budget.

**gate**: Acquisition (4M, DR-0). DR-0 gate on own cfg (dr.fault_prob still sampled at eval time via eval_checkpoint's existing dr.* override loop): gait_valid >=10/12 det+sto, faulted episodes show visible compensation on video (not sacrificed-leg statue), height_err stays in the walking band the whole run (no crouch). Compare numerically against the matched style05 sibling (same cycle) and, if time allows, a quick manual faulted-vs-healthy contrast the way faultobs1 did.

**refused_reason**: acquisition runs require --evidence: name the healthy canary and a comparable full-budget learning precedent.

