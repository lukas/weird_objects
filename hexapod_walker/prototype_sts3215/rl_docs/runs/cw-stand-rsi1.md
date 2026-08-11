# cw-stand-rsi1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: EVALUATED

**created**: 2026-08-11T02:25:40+00:00

**pod**: hexapod-mjx-train-6

**steps**: 2000000

**parent**: cw-stand-scoreref1-dr0

**wandb_id**: zcmvyr1g

**hardware_ready**: False

**hypothesis**: RSI closes the rise state-distribution gap. The 08-10 operator forensic ladder exonerated every other suspect with one controlled run each: pricing (score bank + noisy-replay probe: honest path under FULL 0.198 action noise earns +357 and stands 2/3, ordering noisy-honest >> every cheat), DR (dr0), LR (dr0-lowlr), mode interference (dr0-riseonly). Conclusion: training simply never VISITS the paid states -- the gradient cannot cross from lying-at-ref-start to the full rise because nothing between is ever experienced. RSI (DeepMimic; stage-2 load-bearing in HumanUP/HoST) spawns rise episodes ON the reference at a random phase (goal.rise_rsi_frac=0.5), so rollouts start in paid states and learning proceeds backward along the path. Validated pre-launch: forced-RSI spawns continue to valid plant 7/8, returns +400..+860 with late spawns paying best; ref-clock re-aligns to the settled pose (sag 3-4deg rms) so the limp settle does not break alignment.

**gate**: W&B env/rise_score must lift off the 0.01-0.03 floor (every prior arm flatlined there) and env/reward_rise_ref hold >=0.3/tick with RSI episodes present (env/rise_rsi confirms the mix). Harness at 2M: rise valid_plant from FLAT starts (RSI off at eval -- gate tests the unassisted skill); full success may need a continuation, so the discovery verdict is MECHANISM HEALTH: rise_score climbing + RSI episodes holding the path. If rise_score is still <0.05 at 2M, RSI-with-this-stack is refuted and the structural height<->contact coupling (RISE.md lever b) is next.

**verdict**: FAILED: RSI (spawn-on-reference) is REFUTED as the rise fix. rise 0/6 det+sto at DR0, worst-foot clearance 176-183mm — same flag-leg cheat as score1/scoreref1/scoreref1-dr0 family, now confirmed even with RSI engaged. env/rise_score never left the 0.01-0.03 floor (median 0.015, max 0.089) over the full 2M steps — the pre-registered numeric refutation trigger (gate: <0.05). RSI fixed the state-distribution-visitation theory's premise (rollouts do land on-reference) but the cheat still dominates once trained; this closes the state-distribution-gap hypothesis too. Fifth distinct mechanism beaten by the identical exploit. Next lever per RISE.md is (b): structural height<->contact coupling (CODE+bank, SPECIFICATION phase, not a training run).

