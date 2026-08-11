# cw-stand-holdstill1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: DONE

**created**: 2026-08-11T06:09:10+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-stand-bc1-hard1

**wandb_id**: 71g1npfo

**hardware_ready**: False

**hypothesis**: Make the standing robot hold STILL: until now, parking one leg in the air or endlessly shuffling the legs during a 'hold still' command earned the same reward as standing quietly (the new HOLD bank measured a literal tie, 368.0 vs 367.9), so that is what training converged to. This arm warm-starts the stand-up specialist (ppo_goal_cw_stand_bc1_hard1) and retrains briefly with the ONE change reward.hold_still_gate=1.0 (kernel income on hold/track ticks now requires feet down, no flagged leg, and stillness while the reference is stationary) to test whether the policy learns a quiet six-foot hold without losing its honest stand-up.

**gate**: Harness at 2M: hold-mode episodes end in a quiet valid plant — worst-foot end_clear_mm < 20, swing_count <= 2 per 15 s episode (parent: 6-19 swings, 100-161 mm), det >= 4/6 — AND rise retention: gate det rise valid_plant >= 3/6 with zero flag-leg on video (parent band at 2M was 3/6). KILL early if env/rise_feet_factor < 0.4 for 2 consecutive periodic-eval windows (the pre-anchor collapse signature) or env/hold_feet_factor stays < 0.2 past 1M with hold income flat.

**verdict**: FAIL on the discovery question; rise retention PASS. Hold/track 0/12 det+sto: identical fingerprint to the parent pathology (leg 0 parked 107-116mm, legs 1/3/5 cycling duty ~0.9) despite the gate zeroing its income all run (env/hold_feet_factor crashed 0.99->~0.1 by 260k, stayed <0.2 — the pre-registered kill signature; the sub-4-min run finished before any kill could fire). Rise held: det 4/6 / sto 6/6 valid_plant (parent 2M band 3/6), rise_feet_factor 0.53-0.79 all run. Read: pricing-out the flag is necessary but NOT sufficient from this warm start — the hard no-flag zero is a zero-gradient plateau (all hold behaviors in the sampled neighborhood earn ~0, so PPO gets no slope toward lowering the parked leg). Next: reward.hold_flag_fade (linear fade 60->120mm restores slope; bank update first), holdstill2 = holdstill1 + fade, one variable.

