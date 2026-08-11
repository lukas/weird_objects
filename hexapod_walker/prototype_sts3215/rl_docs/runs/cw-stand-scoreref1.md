# cw-stand-scoreref1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: DONE

**created**: 2026-08-11T00:21:42+00:00

**pod**: hexapod-mjx-train-6

**steps**: 2000000

**parent**: cw-stand-score1

**wandb_id**: hxg739b8

**hardware_ready**: False

**hypothesis**: score1 isolated the remaining hole to EXPLORATION: with all cheat income removed, env/rise_score sat at ~0.01 for 2M steps -- the conjunction score is near-zero everywhere but the actual stand, so the ratchet never fired and the policy parked in the unpaid flag-leg attractor. This arm adds the one thing score1 lacked: the belly->plant reference as the exploration crutch (bank replay reaches the stand and earns +633 dominantly under this exact stack). The crutch is CHEAT-PROOFED, which plantgate1's was not: under score income the ref kernel pays only on grounded feet (feet-down^2 x no-flag; the bank showed the raw kernel re-funds flag-leg +419/ep) and the kernel tightens sigma 12->6deg (near-miss poses farmed +155/ep at 12). Two-stage per HumanUP/HoST: crutch finds the path, score income owns the endpoint; anneal k_rise_ref_track across arms if this works.

**gate**: posture-strict harness at plant height [108,114]mm: rise >=4/6 det valid_plant/end_posture_ok AND lower retains >=5/6 by 2M; W&B env/rise_score must LIFT off the 0.01 floor (score1's flatline is the early-stop trigger: flat at 1.5M = call it); VIDEO: feet-loaded rise, no flag-leg. If rise_score lifts but valid_plant stays 0, the gap is the score top factors (geometry), not exploration.

**verdict**: FAIL — same known exploit (flag-leg) recurs, now a 4th distinct mechanism beaten by it. Posture-strict harness: rise 0/6 det AND 0/6 sto at both DR0 gate and own-DR0.2 (worst-foot clearance 160-188mm, one leg held in the air the whole episode, video-confirmed on every sampled rise/det episode). env/rise_score stayed flat ~0.01-0.02 the entire 2M steps, matching this run's own pre-registered early-stop trigger; SCORE/rise_bridge/flat_success ~0 throughout, only crouch's looser training-time criterion ever fires. The cheat-proofed reference-tracking crutch (k_rise_ref_track=2.0, feet-gated kernel, sigma tightened 12->6deg) does NOT stop the cheat -- same pathology class as cw-stand-b2p1 (detect-discount), cw-stand-plantgate1 (multiplicative gate), cw-stand-score1 (income re-routing). hold/track also regressed vs prior stand arms (det 0/6 posture-ok at DR0, though sto still mostly passes) -- secondary, not the gating criterion. hold/lower legacy stack unaffected in kind.

