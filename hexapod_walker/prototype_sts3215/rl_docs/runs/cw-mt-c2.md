# cw-mt-c2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-12T20:31:42+00:00

**pod**: hexapod-mjx-train-2

**steps**: 20000000

**parent**: cw-mt-c1

**wandb_id**: kvbcuqox

**hardware_ready**: False

**hypothesis**: The broadest command diet: walk, stop, turn AND sidestep commands for one fresh policy — re-run of the broader generalist (narrow-generalist commands plus heading up to 0.34 rad, i.e. lateral velocity up to ~0.02 m/s) at the 20M budget the recipe needs, after the 2M wave proved under-budget for every arm. Prediction-if-true: cw-mt-b2's full gate passes AND lateral probes respond sign-correctly AND the three zero-shot interpolation probes complete without falls. Prediction-if-false(acquisition): forward/yaw arrive but lateral never does — command-space breadth has a hard edge at this capacity. Strongest alternative: diversity slows gait emergence enough that 20M is under-budget only for THIS arm — visible as b2 walking while c2 creeps.

**gate**: cw-mt-b2's full gate PLUS: sign-correct lateral response on +-0.02 m/s vy probes, and the three MULTITASK.md zero-shot interpolation probes (vx=0.037 wz=0.07; vx=0.025 vy=0.012; vx=0.05 wz=-0.11) complete without falls. FAIL(budget)/FAIL(acquisition) labels binding per MULTITASK.md.

**verdict**: FAIL(acquisition-failure) per MULTITASK.md labels, worse than budget or shortfall — with the broadest command distribution (C), the policy never reaches a valid six-leg gait at the SAME 20M budget where the specialist (a2) passed cleanly. Leg index 2 is a true flag leg in every episode (duty 0.01-0.03) and the robot falls in 10/12 gate / 9/12 own-cfg episodes; prog med numbers (1.01) are drag-then-fall artifacts. || CORRECTION 08-15 (operator directive fb_20260815T114414_3c40d6 + audit fb_20260815T113718_baf9d6): the 'acquisition failure / converged' half of this verdict was premature. (1) Training telemetry (W&B kvbcuqox) shows ACTIVE learning at 20M: return rising to ~166-172, env/reward_task 0.07->0.34, walk_prog_factor 0.34->0.80 — the checkpoint failed the gate, but the mechanism had not converged; ep_len falling to ~148 shows the return was optimized via drag-then-fall, i.e. an UNSAFE-REWARD/GAIT MISMATCH (flat -10 fall penalty + ungated aligned-progress income), not a demonstrated inability to acquire broad commands. (2) The yaw clause is INVALID: cfg set goal.walk_yaw_cmd=1 (wz sampled, in obs) but reward.k_walk_yaw was never set (default 0) — commanded to turn, never paid for turning; the same invalidates cw-mt-b2's yaw clause. (3) Heading was only +/-0.34 rad, so no arbitrary-direction claim was ever testable. Corrected label: PROMISING ACTIVE LEARNING + UNSAFE REWARD/GAIT MISMATCH AT 20M. Operator-authorized follow-up: cw-mt-c2-fullcircle1 (full-circle translation-only continuation, gait gate + horizon fall cost + raw v_along telemetry).

