# cw-stand-riserock2-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: DONE

**created**: 2026-08-12T00:36:56+00:00

**pod**: hexapod-mjx-train-2

**steps**: 10000000

**parent**: cw-stand-holdbc1-hard1

**wandb_id**: ak795dj9

**hardware_ready**: False

**hypothesis**: RELAUNCH of cw-stand-riserock2 (launch 1 died at env construction: pod tree predated the axis commit, the unknown-DR-override guard fail-louded exactly as designed; tree synced to code_sha 7d34fc6, cfg keys verified present on train-0). HARDWARE-DRIVEN one-axis arm, and the second relaunch of the defective cw-stand-riserock1 (drained before its CODE-FIRST axis existed; DEFECTIVE, no science). Bench 08-11: the learned rise deterministic-fails on hardware — 5/5 tilt_roll trips at the same tick (~9s mid-curl), rel roll 10.1-10.6deg, currents <=0.27A, clean zero verified — while the sim curl stays <=1.7deg under BOTH actuator fits (loaded fit probed 08-11: does NOT reproduce the rock, so it is not simple lag). ONE CHANGE (commit 36076a6): dr.rise_rock_* — rise-mode episodes carry a persistent one-side hip/knee fold bias on the PHYSICAL servo command (tipped-start fold->roll mapping; logical loop blind like zero_drift_cmd_frame, encoders read the true drooped angles, tilt ref stays LEVEL so leveling is paid). Mechanism probed before launch: a forced 10deg dose rocks hard1s own curl into the 10-11deg trip band with 5/8 terminations = the bench signature reproduced in sim; a dumb P-feedback leveler clears it on both sides (peaks 4.6-6.7deg, zero falls) so the skill is learnable. Config = hard1 exactly + dr.rise_rock_prob=0.5 + dr.rise_rock_deg=6,12 (covers the measured hardware band with margin; half the rise episodes stay nominal for retention; BC anchor stack untouched). Prediction: the policy learns curl-phase leveling, rocked det rise stops tripping, nominal rise/hold retention unchanged.

**gate**: PASS if injected eval (eval_checkpoint --cfg-set dr.rise_rock_prob=1.0 --cfg-set dr.rise_rock_deg=10,10 --baseline hard1, det) rise valid_plant >= 5/6 with ZERO tilt falls AND hard1 under the IDENTICAL injection fails >= 2/6 (matched-parent mechanism control) AND nominal det rise/hold matches hard1s own probe (rise valid incl flat 4/4, hold valid_plant 6/6, no duty regression) AND frames watched (level curl, no new cheat). PASS -> export + deploy the stand-specialist port and retry /api/rl/stand on the bench WITH the operator present. FAIL-A (rocked rise never learned, still trips) -> dose curriculum: one retry at deg 6,10 or prob 0.3. FAIL-B (nominal rise regressed) -> the bias poisons the curl at this dose: one retry at prob 0.25. Double-FAIL -> command-bias axis closed; next lever is modeling the belly contact geometry (rounded chassis collision), not another dose.

**verdict**: FAIL — matched-parent mechanism test (own gate command: dr.rise_rock_prob=1.0, deg=10,10 fixed, det, --baseline hard1) shows ZERO separation: child rise 0/6 valid_plant (1/6 tilt_roll fall, 5/6 stall well short of plant height/footprint), frozen hard1 under the IDENTICAL injection also 0/6 valid_plant (2/6 tilt_roll falls, 4/6 stall) — training bought no measurable resilience to the exact bench-measured critical rock threshold (10 deg is where the real robot's tilt_roll trip sits). Video confirms identical fingerprint both sides: rises partway then tips/settles splayed to one side. Retention on the run's OWN trained-mix distribution (prob 0.5, deg 6-12) is clean and matches hard1's own probe: det rise/hold/lower 6/6, end_posture 6/6 — so the arm did not regress anything, it just failed to learn the specific harder acceptance test. Pre-registered FAIL-A branch.

