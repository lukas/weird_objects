# cw-quad-turn1-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T07:02:57+00:00

**pod**: hexapod-mjx-train-3

**steps**: 10000000

**parent**: cw-quad-hold2

**wandb_id**: choi0t15

**hardware_ready**: no

**hypothesis**: Retry of cw-quad-turn1 (died at init: obs 72 vs 73 mismatch, I omitted --obs-pad-transplant 1 -- fixed here, 0 science lost). Quad-hold (30% mix) x yaw-rate turning command -- composes two independently-landed mechanisms (quad goal-mix c086a22, yaw-rate channel c086a22, obs 72->73 via pad transplant same as yawcmd1) that have never been trained together; tests WISHLIST 15's 'quad turn' rung, which was marked [CODE] on the assumption of needing new machinery -- both flags already exist so this may be launchable without new code. If-true: quad hold survives while walk segments turn (heading-hold+commanded-turn |wz_err| within the yawcmd gate) AND slip/m<=1.25. If-false: the combined obs/reward pressure breaks one skill or the other -- names the specific interaction for a future [CODE] fix rather than guessing.

**gate**: own-cfg (quad=0.3/walk=0.6/hold=0.1 + yaw_cmd) det+sto gv 6/6, 0 term; quad survived_frac>=0.9; walk-mode commanded-turn |wz_err| med<=0.10 rad/s, zero-seg |wz| med<=0.05 rad/s, slip/m<=1.25; frames watched det on walk and quad segments

**verdict**: FAIL (completes the DIG-IN dropped since 08-10). OBSERVATIONS — own-cfg harness (quad/walk/hold, DR0, det+sto) + eval_yaw panel + matched trainer eval: (1) quad clause PASSES: survived_frac 1.0 (trainer eval), harness quad 12/12 success after the ALL_MODES fix, level fronts-lifted stance (roll tail 0.6-0.9deg settled 12/12, h_err ~3mm) — but it CREEPS 0.33m/15s with ~1.1m foot drag, same creep as its hold mode (0.335m): posture holds, stillness never trained. (2) walk clauses FAIL: slip/m det med 1.45-1.58 vs <=1.25 cap; gv not 6/6 — leg-3 flag-leg churn episodes at ~1/6 rate (slip 20-24, prog ~0) in both det+sto. (3) yaw clauses FAIL decisively: commanded-turn |wz_err| med 0.227 vs <=0.10, heading-hold |wz| 0.102 vs <=0.05, right turns untracked (0.33-0.36 = |cmd|+drift) — byte-similar to sibling yawcmd1-s1 (0.242/0.099) on the same pre-fix yaw stack. INTERPRETATION: both failures are class-known — no policy trained on the pre-fix turn reward ever tracked yaw (turn track since closed reward-shape tuning), and quad-mix walk-slip erosion gets its 4th independent confirmation (after joyquad30/15, hold2-lowgait). The compose adds no new pathology; the quad-hold mechanism is intact under it. VERDICT: FAIL, quad-turn rung CLOSED behind the turn track's own 'commanded yaw needs a new idea' wall — no retry from quad. SIDE RESULT (infra): found+fixed eval_checkpoint's silent quad->walk mode fallback (ALL_MODES lacked quad; every prior harness/periodic 'quad' eval row was actually walk episodes) + loud per-episode mode assert.

