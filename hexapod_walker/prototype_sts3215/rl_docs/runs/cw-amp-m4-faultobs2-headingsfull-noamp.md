# cw-amp-m4-faultobs2-headingsfull-noamp

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-23T00:31:20+00:00

**pod**: hexapod-mjx-train-2

**steps**: 4000000

**parent**: cw-amp-m2-bcinit-sec5-noamp-headingsfull

**wandb_id**: 0l6x3z3w

**hypothesis**: Plain English: faultobs1 proved fault-sight helps (prog +18%/slip -27% vs blind) but only on a FORWARD-ONLY walker with a 2M smoke budget -- does the same fault_health-obs mechanism still help on the most command-diverse walker built so far (full-circle heading, incl. backward) at a real acquisition budget? Single lever vs faultobs1-noamp: --init-from swapped from the forward-only bcinit-sec5-noamp checkpoint to bcinit-sec5-noamp-headingsfull, goal.walk_heading_max_rad 0.0 -> -1 (matching that checkpoint's own trained envelope), steps 2M -> 4M (acquisition, not smoke -- the mechanism-safety question is already answered). Same dr.fault_prob=1.0 fault_mix, same obs.fault_health=1 + --obs-pad-transplant 18 wiring. Prediction-if-true: faulted-episode prog_ratio/slip improve over a blind-compensation control by a similar or larger margin than faultobs1's +18%/-27% (more command diversity in training should make the fault-sight signal more useful, not less). Prediction-if-false: no measurable delta or a regression -- heading diversity's own action-space demands crowd out the fault-compensation gradient at this budget.

**gate**: Acquisition (4M, DR-0). DR-0 gate on own cfg (dr.fault_prob still sampled at eval time via eval_checkpoint's existing dr.* override loop): gait_valid >=10/12 det+sto, faulted episodes show visible compensation on video (not sacrificed-leg statue), height_err stays in the walking band the whole run (no crouch). Compare numerically against the matched style05 sibling (same cycle) and, if time allows, a quick manual faulted-vs-healthy contrast the way faultobs1 did.

**verdict**: PASS (acquisition bar met): the fault-sighted walker survives the jump from forward-only to the full-heading substrate at 4M. DR-0 gate on own cfg (dr.fault_prob=1.0 sampled at eval): gait_valid 11/12 det+sto (bar >=10/12) -- det 6/6 prog med 1.14/slip 3.08, sto 5/6 prog 0.59/slip 6.03; the one invalid episode (sto ep0, sac=[0], leg0 duty 0.09) WALKS 0.47m with a visible 5-leg limp, and worst-fault ep4 (prog 0.35/slip 8.35) still advances upright -- compensation, not sacrificed-leg statue. Heights 0-22mm end-err, no crouch; zero terminations 12/12. Joint read vs the matched style05 sibling (same eval seed => same per-episode fault draws, its gate landed this cycle): det 1.09/3.08 vs 1.14/3.08, sto 0.57/7.26 vs 0.59/6.03, hard fault episodes ep0/3/4 bad in BOTH arms by the same shape -- style NEUTRAL again (no styleveto), disc unsaturated (d_real 0.78/d_fake -0.93, style_reward 0.121). CAVEAT the hypothesis's blind-vs-sighted delta is NOT yet measured on this substrate: faultobs1's blind control was forward-only; no blind fault-trained headingsfull twin exists. Launching it this cycle (cw-amp-m4-faultobs2-headingsfull-blind, obs.fault_health=0 single lever) -- paired same-seed contrast lands next.

