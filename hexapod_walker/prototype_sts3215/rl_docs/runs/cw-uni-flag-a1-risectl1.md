# cw-uni-flag-a1-risectl1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-11T18:05:14+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-uni-flag-a1-r1

**wandb_id**: 9n6d5qj3

**hardware_ready**: no

**hypothesis**: Does the one-brain joystick policy fail to learn stand-up because its sibling skills (hold still, sit down) are stealing network capacity, or because learning stand-up from scratch is just hard? This run trains the IDENTICAL from-scratch brain on stand-up ONLY — the single-task control cell the MoE fork decision is missing. One variable vs cw-uni-flag-a1-r1: goal-mix rise=1.0 (was hold=0.2/rise=0.4/lower=0.4); same seed, same hist16+256x256+mode-onehot arch, same stand-specialist reward + BC anchor, from scratch. Context: cw-uni-flag-a1-h2 (10M) left rise flat (all-crouch valid_plant 1/6 det, rise_feet_factor stuck 0.44-0.64 for 10M) while hold/lower stayed clean — the pre-registered MoE trigger fired, but no from-scratch rise-only arm has ever been run (every honest rise was warm-started), so interference is inferred, not measured. This 2M rise-only run gives 2M rise ticks ~= the flagship at 5M total steps (2.5x r1 exposure) with the full logged flagship trajectory as the matched-tick baseline. Prediction-if-true (interference real): rise-only clearly beats the flagship at matched rise-ticks — env/rise_feet_factor sustained >0.8 late-run (flagship never left 0.44-0.64 at ANY point through 12M) and/or all-crouch det valid_plant >=3/6 => MoE fork justified. Prediction-if-false: same plateau (feet factor <=0.65, valid_plant <=1/6) => interference refuted, MoE exonerated — the lever moves to rise-teaching (state-aligned BC anchor spec per crouchrise2 finding, crouch start-mix recipe, or specialist seeding). Strongest alternative: rise-only collapses into the classic flag-leg cheat WITHOUT hold/lower episodes stabilizing feet-down behavior — that would mean the siblings actually HELP, which also argues against isolated-expert MoE.

**gate**: 2M det+sto harness, own stack, rise (all-crouch) only: INTERFERENCE-CONFIRMED if env/rise_feet_factor last-quarter mean >0.8 OR det valid_plant >=3/6 (flagship matched-tick baseline: 0.44-0.64 band, 1/6); INTERFERENCE-REFUTED if both stay in the flagship band (feet <=0.65 AND valid_plant <=1/6); any stable known-exploit fingerprint (flag-leg/tripod/park/freeze) in video = separate STOP verdict. This gate decides the MoE fork.

**verdict**: OBSERVATIONS: 2M from-scratch, rise-only (goal-mix rise=1.0, identical arch/BC-anchor/loaded-servo stack as flagship a1). Harness: det rise 1/6 valid_plant (crouch 1/6, worst_clear 124mm), sto 1/6 (crouch 1/6, worst_clear 86mm) -- IDENTICAL band to flagship h2 (1/6 det). env/rise_feet_factor last-quarter mean 0.537 (settling 0.48-0.61 over the run), inside the flagship's 0.44-0.64 band, well below the 0.8 confirm threshold. Video (6 det + 6 sto, all checked): honest failures only -- crouch-start tip-overs (tilt_roll term, most det eps) or sprawl-stall short of height (1-2 legs near-zero duty), no stable flag-leg/tripod/park/freeze fingerprint; the one passing episode is a genuine six-foot plant, same shape as h2's single pass. INTERPRETATION: both pre-registered REFUTE conditions fire (feet<=0.65 AND valid_plant<=1/6). Isolating rise as the ONLY task, with 2.5x the flagship's rise-tick exposure, does not move the needle -- decisively answers the missing 2x2 cell h2 flagged: shared-capacity interference is NOT the cause of the flagship's rise plateau, because a network with ZERO sibling skills to fight for capacity hits the identical wall. VERDICT: MoE fork REFUTED by direct measurement, not inferred -- do not build a mixture-of-experts to fix unified-policy rise. The bottleneck is rise-from-scratch-on-this-stack itself (matches the campaign-wide crouch-start fragility the deployed BC-anchor specialist itself shows 0/6 RSI-off before its start-mix fix). Next lever for anyone reopening this line: specialist seeding/warm-start rather than another from-scratch reward/mix variant -- but this is architecture-curiosity, not a current hardware blocker (specialist-handoff composition already covers rise for attempt #2, deployed). hardware-ready: no.

