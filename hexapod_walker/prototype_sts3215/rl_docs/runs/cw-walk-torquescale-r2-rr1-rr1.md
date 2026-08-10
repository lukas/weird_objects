# cw-walk-torquescale-r2-rr1-rr1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-10T03:28:36+00:00

**pod**: hexapod-mjx-train-5

**steps**: 6000000

**parent**: cw-walk-torquescale-r1

**wandb_id**: orwes2xw

**hardware_ready**: False

**hypothesis**: 3rd launch attempt: base + r1 both died 0-steps to launch-collision/stale-code-marker infra (gotcha 13b + a stale train-pod .code_sha this window, now synced). Same spec unchanged: OPERATOR WISHLIST 13b torque-droop-under-load axis off cw-walk-longdist-r2.

**gate**: own-cfg torque-droop-DR det+sto gv 6/6 @30s, 0 term, det med fwd within champion band; DR0 nominal-torque retention det 6/6 gv, slip<=1.24; frames watched det

**verdict**: FAIL -- duplicate of the already-CLOSED torque-droop axis, no new science. This is the SAME axis (dr.torque_scale=0.80,1.05) off the SAME parent (longdist-r2) as the already-FAILED sibling cw-walk-torquescale-rr1-rr1 (NO-EFFECT verdict), and it reproduces that verdict episode-for-episode: own-cfg det 4/6 healthy (med fwd 1.44m, letter-meets the >=1.2m text) but det/4-5 crater (prog 0.47/0.41, slip 3.40/3.42, fwd 0.61m) -- nearly identical numbers to the sibling's crater (prog 0.45/0.43, slip 3.66/3.81, fwd 0.66-0.67m) and to that sibling's own NAMED champion baseline under the identical torque spread (prog 0.45/0.41, fwd 0.69/0.69m, logs/ckpt_eval/cw_walk_longdist_r2_torquescale80_base) -- all three cluster in the same noise band, i.e. this training run does not beat the untrained champion on the worst draws either, same as the sibling's pre-registered gate failure. sto 6/6 clean (prog med 0.91). DR0 nominal retention det 6/6 gv perfectly clean (slip med 1.04, fwd 1.56m) -- matches the sibling's retention almost exactly (slip 1.04, fwd 1.59m). Video on the crater draws: six legs cycling, level, no fall/flag-leg -- slow paddle-shuffle, same signature as the sibling. Champion already tolerates 0.80-1.05x torque for free; this 6M-step exposure run (3rd launch attempt after 2 infra deaths) bought nothing new. Torque-droop exposure ladder stays CLOSED; no requeue.

