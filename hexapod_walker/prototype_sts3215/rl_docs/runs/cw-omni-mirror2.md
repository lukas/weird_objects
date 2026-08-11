# cw-omni-mirror2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-11T01:17:21+00:00

**pod**: hexapod-mjx-train-0

**steps**: 40000000

**parent**: cw-omni-mirror1

**wandb_id**: ryodp3zc

**hypothesis**: Teach one policy to walk in any commanded direction and turn on command; this arm re-runs the 40M mirror-symmetry hardening after fixing the reward hole that paid the robot MORE for standing still during commanded turns than for walking. One variable vs the failed cw-omni-mirror1-r1: reward.walk_kernel_yaw_gate=1.0 gates the linear velocity kernel by achieved yaw on turn-in-place ticks (probe 08-11: a freeze banked ~1122/ep of blind kernel+task income while mid-training walking earned 500-860, so PPO parked; the freeze-floor bank now pins the exploit). If-true: eval_yaw tracks both signs + zero-command drift <0.05 + joystick panel incl backward/lateral = the deliverable omni policy, and mirror-symmetry finally gets its behavioral test. If-false with a healthy gait (drift persists under an enforced-symmetric policy): drift is not policy chirality, mirror line closes, look at sim contact asymmetry. If it collapses to freezing AGAIN despite the gate: the bank's model of the exploit is wrong - stop and re-probe income term-by-term.

**gate**: At 40M vs no-mirror baseline cw-arch-hist16-dep1 under the IDENTICAL panel: (1) eval_yaw both signs turn |wz_err| med toward 0.10 AND hold/zero-cmd |wz| med < 0.05; (2) joystick gate 0 falls incl backward+lateral at own DR and DR0.2; (3) gait_valid 6/6 det; (4) video: real swing/stance forward/backward/lateral/turn-in-place, no flag leg or belly shuffle; (5) mirror_sym_loss stays low. KILL EARLY on the r1 exploit signature: det forward travel collapsing toward 0 with frozen episodes out-earning walking, or train/std climbing monotonically past ~2x start (r1 went 0.39->1.69).

