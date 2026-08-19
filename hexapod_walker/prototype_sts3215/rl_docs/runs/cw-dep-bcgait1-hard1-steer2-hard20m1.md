# cw-dep-bcgait1-hard1-steer2-hard20m1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: LAUNCH_CRASH

**created**: 2026-08-18T19:42:58+00:00

**pod**: hexapod-mjx-train-4

**steps**: 20000000

**parent**: cw-dep-bcgait1-hard1-steer2-stagecurric1

**hypothesis**: Teach the tall walker to survive abrupt joystick direction changes for real, not just a 2M mechanism check, using the STAGED curriculum lever instead of the full-mix-from-tick-0 recipe that FAILED at 20M (-steer1-hard20m1: 3/24 over_current + 1/24 severe 3-leg tangle). The 2M canary (-steer2-stagecurric1) already ramped goal.walk_cmd_stage 0->2 and showed a favorable informational read on the identical 24-episode panel (0/24 over_current vs 3/24, 1/24 mild single-leg drag vs 1/24 severe tangle). This run continues from that ALREADY-RAMPED checkpoint (stage pinned at 2, full stress mix, no more scheduling needed) for ~20M more steps with the same --best-ckpt retention guard as steer1-hard20m1, to see whether the staged-curriculum-trained checkpoint actually hardens into a clean pass instead of just showing a favorable small-budget signal. Prediction-if-true: at 20M the checkpoint retains fixed-command hard1 quality (height >= -20mm, six-leg gait-valid, zero falls, slip <= 1.8/m) AND the same 24-episode direction-switch panel (DR-0 + own-DR-0.35, det+sto) shows ZERO over_current terminations and zero severe (>=2-leg) tangles. Prediction-if-false: jamming/tangle events persist at a similar rate to the 2M canary or worse despite 10x more steps at full stage-2 mix, meaning staged EXPOSURE alone doesn't scale to full hardening and the next lever is structural (blend-in time on switches, or yaw-margin reward pricing), not more steps on this recipe.

**gate**: HARDENING: --best-ckpt retention guard active throughout. PASS requires ALL: (1) fixed-command panel (dr-scale 0, no switches) matches or beats hard1: height >= -20mm, six-leg gait-valid every episode, zero falls, slip <= 1.8/m; (2) the identical 24-episode direction-switch panel (stress_mix cfg, DR-0 gate + own-DR-0.35, det+sto, 120s episodes) shows ZERO over_current safety terminations; (3) zero episodes with >=2 sacrificed legs (severe tangle) -- isolated single-leg drags are a partial-credit near-miss, not an automatic fail, but must be named; (4) no accumulating yaw-limit saturation across repeated switches; (5) command tracking recovers promptly post-switch (vel_err back in-band within the same bound as hard1 steady-state). FAIL on any bar; name it. Matched-parent (hard1) control required for the fixed-command panel.

**verdict**: Launch-tool crash, zero GPU-seconds, no training happened. My respec passed --arg='--best-ckpt=' (with a trailing '='), which routes through set_flag() and appends an empty-string token after --best-ckpt -- reintroducing the exact argparse trailing-empty-arg crash the tool's own set_bare_flag() fix (08-18) targets, because set_bare_flag only triggers on a truly bare flag (no '=' at all). train_ppo_mjx.py died at argparse before any step ('unrecognized arguments'). Fixed by relaunching with the correct bare-flag form (--arg='--best-ckpt', no '='): cw-dep-bcgait1-hard1-steer2-hard20m1-r1 is VERIFIED RUNNING on train-4. No behavioral content here -- pure launch-tooling mistake, corrected same cycle.

**failed_reason**: run never appeared as 'running' in W&B within 240s

