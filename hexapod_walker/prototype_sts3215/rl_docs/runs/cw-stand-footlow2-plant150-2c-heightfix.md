# cw-stand-footlow2-plant150-2c-heightfix

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-22T16:42:31+00:00

**pod**: hexapod-mjx-train-5

**steps**: 10000000

**parent**: cw-stand-footlow2-plant150-1

**wandb_id**: r3846sx5

**hypothesis**: Plain English: fix a stale-config bug found by triage, then see if it also helps the one real remaining defect. cw-stand-footlow2-plant150-1 (10M steps, warm-start fine-tune for the tibia-150 geometry) FAILED its gate (rise det 1/6, sto 0/6, vs parent's 5/6 det, 6/6 sto), but a same-checkpoint re-eval with the height target corrected to the bank's own 08-22 recalibration (actions.max_height_mm 115->137, goal.rise_height_mm [108,114]->[128,136] -- the values test_task_semantics.py's RISE_OVERRIDES/SCORE_OVERRIDES already use, which this run's copy-pasted launch recipe never picked up) shows the checkpoint was ALREADY GOOD on 3 of 4 start kinds: bridge/crouch/flat rise cleanly to within 1.4-3.9mm of the corrected target (det 1/6->3/6, sto 0/6->4/6, zero new training). Only RSI-reset starts (DeepMimic-style mid-ramp spawn) still fail every episode (5/5, ~22-29mm undershoot, unmoved by the cfg fix -- proving RSI's target already tracked the reference file's own tibia-150-correct height, not the buggy cfg -- and roll peaks up to 9deg vs 0.5-0.8deg on the clean starts, a real wobble). RSI passed fine on the 128mm parent (4/5). This run trains 10M MORE steps from the SAME checkpoint with ONLY the corrected cfg (matching what test_task_semantics.py already validated), so the reward's height income term stops fighting the already-correct bc_anchor reference on bridge/crouch/flat and any freed-up gradient can go toward the harder RSI case.

**gate**: PASS if, on the eval_checkpoint gate harness with the CORRECTED cfg (max_height_mm=137, rise_height_mm=[128,136]): rise det >= 5/6 AND sto >= 6/6 (matching or beating the 128mm parent's own numbers) with RSI-start height_err_end_mm improving materially (<=15mm, i.e. crossing the PLANT_SPEC window) on at least half of RSI episodes, while hold/lower stay at parent level and the session gate's tibia-150 sit-fall stays fixed. FAIL if RSI-start height_err stays pinned near 20-29mm regardless of budget (redirect to a dedicated audit of the RSI reference file's intermediate-ramp poses at tibia-150, not more training), or if bridge/crouch/flat regress from this run's own corrected-eval baseline (3/6 det, 4/6 sto).

**verdict**: FAIL exactly on the pre-registered branch: RSI-start rise stays pinned at 22-29mm height error (det rsi 0/3, sto rsi 0/2) after 10M MORE steps, zero movement; bridge/crouch/flat held steady at the run's own corrected baseline (det 3/6, sto 4/6), no regression. Reward flattened after Q1 (120.6->201.1->198.1->200.6) matching reward-flat+eval-flat = stuck mechanism, not undertraining. ROOT-CAUSED (not just re-verdicted): sim_env.py's RSI height-schedule rewrite anchored the ABSOLUTE target to the reference npz's OWN recorded final height (ref['h'][-1]=110.96mm), which is stale pre-tibia-150 data -- the same q_rad trajectory settles at 131.94mm on the current sim (CURRENT_TRUTHS rise_valid_plant finding), a ~21mm gap matching the observed error almost exactly. Every RSI episode was being trained toward a target ~21mm below the real goal.rise_height_mm window -- misalignment, not an unlearnable motion. FIXED: anchor to the episode's own live-cfg target height, use the reference only for fractional progress at the spawn point (rl_move/sim/sim_env.py); 2 new regression tests added (test_rise_rsi_height_target.py, both PASS) + full bank re-run clean (159 pass, only the pre-existing known-red fastprof fails, unrelated). Follow-up: cw-stand-footlow2-plant150-3-rsifix continues from this checkpoint with the fix live.

