# cw-stand-riserock2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-12T00:19:29+00:00

**pod**: hexapod-mjx-train-0

**steps**: 10000000

**parent**: cw-stand-holdbc1-hard1

**hypothesis**: HARDWARE-DRIVEN relaunch of the defective cw-stand-riserock1 (drained onto a pod before its CODE-FIRST axis existed — no dr.rise_rock cfg, none of hard1s recipe; marked DEFECTIVE, no science). Bench 08-11: the learned rise deterministic-fails on hardware — 5/5 tilt_roll trips at the same tick (~9s mid-curl), rel roll 10.1-10.6deg, currents <=0.27A, clean zero verified — while the sim curl stays <=1.7deg under BOTH actuator fits (loaded fit probed 08-11: does NOT reproduce the rock, so it is not simple lag). ONE CHANGE (commit c794de0): dr.rise_rock_* — rise-mode episodes carry a persistent one-side hip/knee fold bias on the PHYSICAL servo command (tipped-start fold->roll mapping; logical loop blind like zero_drift_cmd_frame, encoders read the true drooped angles, tilt ref stays LEVEL so leveling is paid). Mechanism probed before launch: a forced 10deg dose rocks hard1s own curl into the 10-11deg trip band with 5/8 terminations = the bench signature reproduced in sim; a dumb P-feedback leveler clears it on both sides (peaks 4.6-6.7deg, zero falls) so the skill is learnable. Config = hard1 exactly + dr.rise_rock_prob=0.5 + dr.rise_rock_deg=6,12 (covers the measured hardware band with margin; half the rise episodes stay nominal for retention; BC anchor stack untouched). Prediction: the policy learns curl-phase leveling, rocked det rise stops tripping, nominal rise/hold retention unchanged.

**gate**: PASS if injected eval (eval_checkpoint --cfg-set dr.rise_rock_prob=1.0 --cfg-set dr.rise_rock_deg=10,10 --baseline hard1, det) rise valid_plant >= 5/6 with ZERO tilt falls AND hard1 under the IDENTICAL injection fails >= 2/6 (matched-parent mechanism control) AND nominal det rise/hold matches hard1s own probe (rise valid incl flat 4/4, hold valid_plant 6/6, no duty regression) AND frames watched (level curl, no new cheat). PASS -> export + deploy the stand-specialist port and retry /api/rl/stand on the bench WITH the operator present. FAIL-A (rocked rise never learned, still trips) -> dose curriculum: one retry at deg 6,10 or prob 0.3. FAIL-B (nominal rise regressed) -> the bias poisons the curl at this dose: one retry at prob 0.25. Double-FAIL -> command-bias axis closed; next lever is modeling the belly contact geometry (rounded chassis collision), not another dose.

**verdict**: INFRA FAILURE, no science: the launch synced pod code to snapshot 7d649b5, which predates the dr.rise_rock_* commit (36076a6) — trainer crashed at startup on 'unknown DR override dr.rise_rock_prob' and never reached W&B. Spec is sound (hard1 recipe + rise_rock overrides are absolute post-scaling, randomizer present). Relaunched unchanged as cw-stand-riserock2-r1 on current code.

**failed_reason**: run never appeared as 'running' in W&B within 240s

