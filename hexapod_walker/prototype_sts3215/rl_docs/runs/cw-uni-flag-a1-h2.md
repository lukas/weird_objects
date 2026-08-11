# cw-uni-flag-a1-h2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-11T16:46:13+00:00

**pod**: hexapod-mjx-train-0

**steps**: 10000000

**parent**: cw-uni-flag-a1-r1

**wandb_id**: 22x0z3y9

**hardware_ready**: no

**hypothesis**: More practice, not a new brain: the one-network joystick policy (flagship stage A) already holds still and sits down cleanly and its stand-ups are honest but unfinished at 2M from scratch — this run tests whether a 10M hardening pass finishes stand-up the way it finished the specialist's. Hardening continuation of cw-uni-flag-a1-r1: identical config/stack (hist16 + 256x256 + mode one-hot, stand-specialist reward + BC anchor, hold/rise/lower mix, n-envs 3072 shm fix), warm-started from its checkpoint, budget is the only variable. Retry of cw-uni-flag-a1-h1(/-r1/-rr1), which all crashed at 0 steps on a trainer flag refusal (--net-arch vs warm start) — fixed at snapshot exp/cw-uni-flag-a1-h2, not a science result. Prediction-if-true: rise all-crouch valid_plant >=4/6 det with hold/lower retained by 10M (bc1-hard1 precedent: current/footprint tails resolve with budget). Prediction-if-false: rise valid_plant still <3/6 at 10M with flat rise_plant/feet factors despite budget = genuine shared-capacity interference -> fork to MoE per the flagship pre-registration. Strongest alternative: rise converges but hold/lower erode under shared capacity — that would also argue MoE.

**gate**: 10M det harness, own stack: rise (all-crouch) valid_plant >=4/6 AND hold valid_plant >=5/6 AND lower posture-strict >=5/6, zero STABLE known-exploit fingerprints (flag-leg/tripod/park/freeze) in video; MoE fork triggers if rise valid_plant <3/6 with flat rise factors.

**verdict**: OBSERVATIONS: 10M hardening, budget-only variable vs r1. Hold det 6/6 valid_plant (motionless level six-foot stand, video); lower det 6/6 + sto 6/6 posture-strict (IMPROVED vs parent sto 3/6). Rise (all-crouch) det 1/6 + sto 1/6 valid_plant — identical to the 2M parent, same passing episode; anatomy shifted stalls->falls (det 4/6 tilt_roll tip-overs AFTER honest six-foot rises; sto misses mostly current-tail); ZERO exploit fingerprints in 12 video-checked rise strips (sprawl-stall or rise-then-tip, no stable flag-leg/tripod/park/freeze). Training: env/rise_feet_factor noise-band 0.44-0.64 the FULL 10M (r1 end 0.62 "still climbing" was noise), rise_plant_factor flat 0.2-0.4, bc_anchor_loss converged ~0.009 throughout, hold/lower factors stable — no seesaw. INTERPRETATION: budget lever DEAD (5x steps, zero rise movement — the pre-registered if-false fires on the numbers). But its "flat = shared-capacity interference -> MoE" reading is confounded twice: (1) the all-crouch bar is failed by the deployed specialist champion itself (holdbc1-hard1 0/6 RSI-off crouch, same tilt_roll fingerprint; only the unpromoted crouchrise variants clear it) — this may be the campaign-wide crouch-start fragility, not a multitask effect; (2) the (from-scratch, rise-only, BC-anchor-stack) cell has NEVER been run (ledger-audited: every honest rise came from a warm start; the one rise=1.0 arm was warm on the pre-BC scoreref stack), so interference is inferred, not measured — CURRENT_TRUTHS requires demonstrated interference before MoE. Also relevant: crouchrise2 (concurrent, other cycle) just showed the clock-indexed BC anchor actively MIS-TEACHES on crouch starts (belly-phase reference vs near-plant state) — the flagship diet is 100% crouch starts, so its anchor may be partly poisoned the same way. VERDICT: FAIL on gate; hardening CLOSED for this lineage (no more budget/step variants). MoE fork NOT taken yet: queued cw-uni-flag-a1-risectl1 (2M discovery, identical from-scratch config, goal-mix rise=1.0 — the missing 2x2 cell). Rise-only clearly beating the flagship at matched rise-ticks => interference confirmed => MoE justified; same plateau => MoE exonerated, lever moves to rise-teaching (state-aligned anchor spec / crouchrise start-mix recipe / specialist seeding). hardware-ready: no.

