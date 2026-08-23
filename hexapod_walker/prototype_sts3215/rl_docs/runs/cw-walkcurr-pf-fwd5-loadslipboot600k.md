# cw-walkcurr-pf-fwd5-loadslipboot600k

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-23T19:50:50+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-walkcurr-pf-fwd3-chargeramp

**wandb_id**: qg072atd

**hypothesis**: Plain English: same question as the sibling fwd5-loadslipboot300k arm (does briefly softening ONLY the anti-skate/slip charge let a from-scratch policy discover forward walking, after 5 straight rung-1 arms froze), but with the bootstrap window 2x longer (600k = 30% of the 2M budget vs 15%) in case 300k anneals back to full pricing before the policy has actually found a walking gait to lock onto. Same recipe otherwise (fwd3's charge ramp kept, fresh init, same bank-proven 0.65 floor).

**gate**: Rung-1 gate (same as fwd1/fwd3/fwd4/fwd5-300k): C-env det fixed-forward panel -- zero tilt terms, cmd_prog_frac >= 0.35, direction_err <= 30 deg, slip/m <= 3.0, six legs cycling on >=4/6 episodes, video shows real stepping. Discovery-health: env/walk_freeprog_score > -0.02 and rising at 2M = continue/harden; still flat in [-0.10,-0.05] at 2M = loadslip-bootstrap hypothesis refuted at both window lengths -> escalate to dig-in (rung-0/BC-kickstart/architecture), no further same-class reward-magnitude arm.

**verdict**: Same lever, 2x longer bootstrap window (600k/2M = 30% of budget): identical outcome to the 300k sibling. env/walk_freeprog_score flat in [-0.10,-0.05] the whole run, gate eval 0/6 det + 0/6 sto, prog_ratio 0.00, det speed 0.003 m/s vs 0.05-0.06 cmd, slip/m 10-41, contact sheet shows the same static splayed crouch. Window length was not the missing variable either -- closes the loadslip-bootstrap fork at both tested doses. Per the track's own escalation rule (STATUS.md 'Next'): init/noise (fwd4, 2/2 FAIL) AND loadslip-bootstrap (fwd5, 2/2 FAIL) are now BOTH refuted with aligned reward+eval (reward falls monotonically as charges ramp against a frozen policy, eval flat 0/6 the whole way) at adequate budget (2M, 6 total rung-1 arms) -- the prior-free MLP recipe from random init is refuted at this budget. Escalate to a dig-in (rung-0 curriculum / BC-kickstart / architecture) before any further reward-magnitude variant, per the pre-registered rule.

