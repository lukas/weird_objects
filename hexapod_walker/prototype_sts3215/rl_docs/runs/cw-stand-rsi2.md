# cw-stand-rsi2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: done

**created**: 2026-08-11T02:41:22+00:00

**pod**: hexapod-mjx-train-6

**steps**: 2000000

**parent**: cw-stand-rsi1

**wandb_id**: fofiof36

**hardware_ready**: False

**hypothesis**: ONE change vs cw-stand-rsi1: the warp pool-restore fix (SNAP_ATTRS += _score_best/_rise_ramp_i0/_end_posture_from/_rsi_pending/_rsi_ref_tick0, commit 65edba7). rsi1's env/rise_rsi decayed 0.58->0.15 with ZERO terminations -- impossible for a constant 0.5 spawn fraction -- exposing that pool-restored episodes inherited another episode's score ratchet high-water mark and ramp/RSI clocks, so score+ref income silently stopped paying as pooled generations took over (~20-30 updates), which IS the erosion signature of every stand arm since score1. With state restored correctly, the already-validated pricing (noisy-replay +357, bank green) plus RSI's state coverage should finally let the ratchet pay through training.

**gate**: env/rise_rsi must HOLD near 0.5 all run (the direct probe of the fix); env/reward_rise_ref >=0.3/tick sustained; env/rise_score off the 0.01-0.03 floor and climbing. Harness at 2M: rise valid_plant from flat starts (RSI off at eval); mechanism-health verdict as rsi1. If rise_rsi holds ~0.5 but score still flatlines, the pool bug is exonerated and RISE.md lever b (structural height<->contact coupling) is next.

**verdict**: FAIL -- KNOWN exploit (tripod/flag-leg): rise 0/6 det+sto, worst-foot clearance 146-161mm (one leg held in air the whole episode, video-confirmed, identical to score1/scoreref1). Mechanism-health check PASSED though: env/rise_rsi held ~0.48-0.58 all 2M steps (pool-restore fix confirmed working) but env/reward_rise_ref crashed 0.83->0.02-0.09/tick and env/rise_score never left the 0.01-0.02 floor, exactly like the corrupted runs. Per the pre-registered gate: pool bug is exonerated; RSI-with-this-stack is refuted on clean data; RISE.md lever (b) -- structural height<->contact coupling -- is next.

