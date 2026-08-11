# cw-stand-rsi1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: DONE

**created**: 2026-08-11T02:25:40+00:00

**pod**: hexapod-mjx-train-6

**steps**: 2000000

**parent**: cw-stand-scoreref1-dr0

**wandb_id**: zcmvyr1g

**hardware_ready**: False

**hypothesis**: RSI closes the rise state-distribution gap. The 08-10 operator forensic ladder exonerated every other suspect with one controlled run each: pricing (score bank + noisy-replay probe: honest path under FULL 0.198 action noise earns +357 and stands 2/3, ordering noisy-honest >> every cheat), DR (dr0), LR (dr0-lowlr), mode interference (dr0-riseonly). Conclusion: training simply never VISITS the paid states -- the gradient cannot cross from lying-at-ref-start to the full rise because nothing between is ever experienced. RSI (DeepMimic; stage-2 load-bearing in HumanUP/HoST) spawns rise episodes ON the reference at a random phase (goal.rise_rsi_frac=0.5), so rollouts start in paid states and learning proceeds backward along the path. Validated pre-launch: forced-RSI spawns continue to valid plant 7/8, returns +400..+860 with late spawns paying best; ref-clock re-aligns to the settled pose (sag 3-4deg rms) so the limp settle does not break alignment.

**gate**: W&B env/rise_score must lift off the 0.01-0.03 floor (every prior arm flatlined there) and env/reward_rise_ref hold >=0.3/tick with RSI episodes present (env/rise_rsi confirms the mix). Harness at 2M: rise valid_plant from FLAT starts (RSI off at eval -- gate tests the unassisted skill); full success may need a continuation, so the discovery verdict is MECHANISM HEALTH: rise_score climbing + RSI episodes holding the path. If rise_score is still <0.05 at 2M, RSI-with-this-stack is refuted and the structural height<->contact coupling (RISE.md lever b) is next.

**verdict**: CORRECTION (supersedes my earlier FAILED verdict on this run): this run's erosion is CONFOUNDED, not a genuine RSI refutation. Root cause found+fixed by a concurrent cycle (commit 65edba7, rl_docs/RISE.md 08-11~03:30): the warp/MJX pool-restore path was missing the score-stack + RSI per-episode attrs from SNAP_ATTRS, so pool-recycled episodes silently inherited another episode's _score_best ratchet and ramp/RSI clocks -- score/ref income stopped paying as pooled generations took over, which is exactly rsi1's observed env/rise_rsi decay 0.58->0.15 with zero terminations (impossible for a constant 0.5 spawn frac -- a code tell, not behavior). This also reopens score1/scoreref1/scoreref1-dr0(-lowlr/-riseonly): none of those stacks were paid as designed on the GPU path either. rsi1 itself is not informative either way on RSI; the clean test is cw-stand-rsi2 (same args + the fix), already running. Retracting the 'RSI refuted' / 'state-distribution-gap hypothesis closed' claims from my prior verdict.

