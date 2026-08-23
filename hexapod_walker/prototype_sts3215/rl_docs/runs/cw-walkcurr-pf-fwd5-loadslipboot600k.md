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

**verdict**: Result: identical frozen-crouch discovery failure to its 300k sibling and to fwd1-fwd4, with the loadslip-bootstrap window doubled to 600k/30% of budget. Evidence: rung-1 gate 0/6 det + 0/6 sto walk success, prog_ratio ~0.00 both modes, dir_err 28.6deg(det, noise around a stationary robot)/90.0deg(sto), slip/m 10.08(det)/40.71(sto) vs cap 3.0; env/walk_freeprog_score flat in [-0.10,-0.05] the entire 2M steps (last=-0.068), never trending toward 0; contact sheet static splayed stance across all 10 frames, same pose as every prior fwd1-5 arm and its 300k sibling. Why: doubling the bootstrap window did not change the outcome at all -- rules out 'the window anneals back to full pricing before the policy locks on' as the reason 300k failed. The blocker is not a loadslip-charge-schedule-length problem; it sits upstream of any reward-magnitude/schedule lever. What's next: this is the SECOND (and window-length-robust) confirmation that loadslip-bootstrap does not unfreeze discovery, jointly with fwd4's init-noise/entropy refutation -- per the track's own pre-registered rule (STATUS.md 'Next'), the prior-free MLP recipe is now refuted at this 2M budget across every tried reward-magnitude/init/entropy/bootstrap lever. No further same-class reward-magnitude arm should launch; escalate to a dig-in (rung-0 easier-start curriculum / BC-kickstart / architecture change) before any further training. Flagging DIG-IN alongside the -300k sibling for that escalation-path decision.

