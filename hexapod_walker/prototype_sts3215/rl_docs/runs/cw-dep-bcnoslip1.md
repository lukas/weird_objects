# cw-dep-bcnoslip1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-12T14:44:49+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-dep-bcgait1

**wandb_id**: gr9c9lsk

**hypothesis**: Swap the BC-INIT teacher from the drag tripod to the scripted NO-SLIP step-then-shift gait (linux_control/noslip_gait.py, world-pinned stance feet) and pay the band that teacher can actually reach (walk speed band 0.05-0.06 -> 0.02-0.04, park-duty charge off so quasi-static dwells are not billed as parking): PPO fine-tune consolidates a low-slip tall walking gait instead of rediscovering the drag/paddle attractor. Evidence the init is viable (operator chat 08-12): the clone walks in closed loop on the band-matched stack (probe_walk_income return 562 vs teacher 588, progress 0.48, zero terminations) and is tall by probe_tall_wall (+7..+32 mm, 177 mm footprint). If true: the 2M ckpt holds walk_loadslip_factor >= the teacher~s 0.15 (the drag gait sits at 0.07-0.09) while lifting progress_ratio toward 0.7 in-band. If false: income pulls it back to the drag fingerprint (loadslip -> 0.09, faster cadence) or it stalls (progress -> 0), meaning band+park changes are still insufficient and the fitted servo clamp is the binding constraint.

**gate**: probe_walk_income on the vref1 base + goal.walk_speed_min/max=0.02/0.04 + reward.k_park_duty=0 at CMD_V 0.03, forward+crab_left x seeds 0,1,2: TOTAL_RETURN > 588 (the scripted teacher) AND walk_loadslip_factor >= 0.15 AND progress_ratio >= 0.55 with 0 terminations. Secondary: probe_tall_wall steady height >= -20 mm (stays tall).

**verdict**: FAIL on the pre-registered gate, fail mode (a) revert-to-crouch. Band-matched probe (vref1 base, band 0.02-0.04, park_duty 0, CMD 0.03): TOTAL_RETURN -67 vs scripted no-slip teacher 593, progress_ratio 0.14 vs 0.54, walk_loadslip_factor 0.08 (= the drag-gait level; teacher 0.19), anchor_frac 0.40. probe_tall_wall: -50 mm crouch, 261 mm splay, yaw/pitch margins pinned negative. The BC init itself was healthy (return 562, progress 0.48, tall +7..+32 mm), so 2M PPO steps ERASED the no-slip prior even with the band lowered to the teacher envelope and park-duty off: pitch penalty -51 and height -45 indicate it bought income with the lean-crouch attractor. Retry per the bcgait1 recipe: -r1 with lr 1e-4 + target-kl 0.01, one shot.

