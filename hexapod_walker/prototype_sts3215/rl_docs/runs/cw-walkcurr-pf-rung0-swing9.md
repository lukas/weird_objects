# cw-walkcurr-pf-rung0-swing9

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-23T21:41:36+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**wandb_id**: rns9frs5

**hypothesis**: Plain English: same rung-0 'lift your feet and step, anywhere' diet as cw-walkcurr-pf-rung0-swing3, with the stepping paycheck tripled -- if the swing income at 1x is still too small against the remaining charges for a random policy to notice, 3x should show it. Single lever vs the swing3 twin: reward.k_walk_swing 0.06 -> 0.18 (bank test_walkcurr_rung0_*[x0.02-swing9] GREEN at this exact cfg: full ordering preserved, stall/shuffle margins over the freeze class grow with dose, walking stays strictly on top, skate/topple floor unchanged; snapshot exp/walkcurr-rung0-bank). Prediction-if-true: reward_swing trends up and feet cycle within 2M (possibly earlier/stronger than swing3). Prediction-if-false on BOTH arms (frozen crouch, reward_swing ~0, clip_fraction healthy): swing-income dose is not the blocker at any bank-safe level inside the rung-0 diet -- reward-landscape theory refuted, escalate to RND state-novelty per track STATUS. Strongest alternative: swing9 unfreezes but farms chaotic flail-stepping that never settles into a gait -- readable on video and by gait_valid; still a rung-0 PASS if six legs cycle without falls, and rung-1b's freeprog/step_event pricing does the refinement.

**gate**: Rung-0 certification gate (same as swing3): C-env det fixed-forward panel -- zero tilt terms, six legs cycling (gait_valid) on >=4/6 det episodes, video shows rhythmic stepping; travel/prog NOT required. Mechanism health at 2M: env/reward_swing per-step > 0 and rising, clip_fraction > 0.02. Joint read with swing3: either arm certifying = rung-0 PASS (pick the cleaner video for rung-1b warm-start); both frozen = rung-0 refuted -> RND escalation.

**verdict**: 3x swing-income dose does NOT certify rung-0 either -- gate FAIL, and it fails a NEW way (static airborne splay, not the prior frozen crouch). Evidence: own-cfg C-env det panel 0/6 gait_valid (need >=4/6) -- every det episode sacrifices ALL SIX legs (duty_cycle 0.02-0.03 uniformly), forward_dist ~0.006m over 25s, sto 1/6 term (tilt_pitch) + only 1/6 gait_valid; video (walk_det_0, walk_sto_3) shows the robot lifting all six legs off the ground into a static splayed hover held for the whole clip -- not rhythmic stepping, not the swing3-style crouch. W&B confirms this is a NEW failure mode, not the old optimizer crush: clip_fraction stayed healthy and rising (0.010->0.083), std crept up (0.368->0.373) -- optimizer is fine -- but env/reward_swing FELL monotonically (0.016->0.003, the opposite of the prediction-if-true) while ep_rew_mean quarters also fell (42/40/30/15). Why: reward falling + eval failing is an ALIGNED read per the 08-21 ruling (not misalignment to repair) -- tripling k_walk_swing did not make swing income durably profitable; the policy found it cheaper to plant nothing (near-zero duty on every leg, near-zero park_duty/loadslip exposure too since it is airborne) than to bank swing pay, and rides that to a flat/declining return. This refutes swing-income DOSE as the lever at bank-legal scale (swing3 was 1x, swing9 3x, both fail -- read jointly with swing3's own verdict, which a concurrent cycle owns). Next: if swing3 also fails, this closes the rung-0 sub-goal fork under the swing-income mechanism specifically; escalate to RND state-novelty per track STATUS's pre-registered order, or reconsider a stance-duty-priced variant (pay for TIME-ON-GROUND per leg, not completed swings, so an airborne hover cannot free-ride) before funding RND. hardware-ready: no.

