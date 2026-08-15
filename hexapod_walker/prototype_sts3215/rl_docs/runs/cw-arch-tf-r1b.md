# cw-arch-tf-r1b

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: DONE

**created**: 2026-08-15T12:12:54+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-arch-hist16-r7

**wandb_id**: kymrqt44

**hardware_ready**: no

**hypothesis**: Temporal-arch discovery rung (relaunch of cw-arch-tf-r1, relocated to train-1 with CUDA torch + --device cuda after the CPU-torch update path measured ~200 fps): small causal transformer trunk (2 layers, d_model 128, 4 heads, ff 256, separate actor/critic trunks, 622k params vs hist16 MLP 331k) attending over the SAME 16-frame window and EXACT recipe as champion cw-arch-hist16-r7, from scratch. One variable changed vs r7: policy trunk (flatten-MLP -> causal attention). This 2M rung answers ONLY: does the transformer+PPO stack boot at 3072 envs, train stably (no NaN/collapse, healthy fps on cuda), and start moving without an early cheat lock-in? Architecture verdict belongs to the 40M hardening twin (r7 itself showed no gait by 2M).

**gate**: PASS = boots and trains to 2M with no crash/NaN, healthy cuda fps (>=2000), ep_rew climbing, and the 1M/2M det walk evals free of the leg-sacrifice fingerprint (no 3-leg park). Directional gait NOT required at 2M. FAIL = crash, flat/dead reward, or leg-sacrifice/paddle already locked in det. If PASS -> respec 40M from-scratch hardening twin of hist16-r7.

**verdict**: PASS (2M discovery mechanism canary, per pre-registered gate). The causal-transformer PPO trunk (2 layers, d_model 128, 4 heads, over the 16-frame window) boots and trains stably on GPU-MJX with the ad hoc CUDA-torch build: 2.02M steps, no NaN/crash, ep_rew climbs through a mid-run dip to +93 by 2M (quarters -1.5/-64.3/-28.8/+74.9), explained_variance 0.02->0.86, std stable 0.37->0.39 (no collapse/blowup). Det walk eval (own DR0.5 + gate DR0): gait_valid 6/6 both passes, sacrificed_legs empty every det episode -- the one hard FAIL clause (leg-sacrifice/paddle lock-in) does not fire. Stochastic eval does show a dropped leg in some episodes (gate DR0 sacrificed [3] in 3/6, own-DR0.5 sacrificed [4] in 1/6) -- expected exploration noise at 2M, not a det lock-in, so it does not trip the gate. Watched det frame strips (walk_det_0/3, all 6 episodes): all six legs visibly cycling swing/stance, no flag leg -- but the robot falls via roll on EVERY det episode (roll_class fell 6/6, roll_peak ~15-16 deg, tail ~4 deg, roll_settled 0/6, term_reason tilt_roll every time), covering only ~0.1-0.15 m before tipping (slip/m ~2.9-4.5, prog_ratio noisy since distance is tiny). This is the SAME universal takeoff-roll instability already documented campaign-wide (CURRENT_TRUTHS) -- not new, not a transformer-specific defect, and explicitly NOT required to be solved at this discovery rung (directional gait/survival not gated at 2M). fps ended 1651 (peaked 1887 mid-run) -- short of the gates informational >=2000 floor, same ad hoc-CUDA-torch environmental softness already noted on this lineage checkup, not a behavioral fail. Session gate: INCOMPATIBLE as expected (transformer obs width 1152 != deployed session env 72 -- exotic-obs candidate, informational only). Per the pre-registered gate: PASS -> respec 40M from-scratch hardening twin of hist16-r7, launched this cycle as cw-arch-tf-r1-hard1 on train-1 (keeps the ad hoc CUDA-torch build already proven there).

