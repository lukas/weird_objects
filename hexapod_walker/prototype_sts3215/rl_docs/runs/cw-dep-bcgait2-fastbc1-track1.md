# cw-dep-bcgait2-fastbc1-track1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: done

**created**: 2026-08-20T23:33:55+00:00

**pod**: hexapod-mjx-train-7

**steps**: 5000000

**parent**: cw-dep-bcgait2-fastbc1

**wandb_id**: nkf9ywol

**hardware_ready**: False

**hypothesis**: Teach the new fast walker to obey the joystick speed: the fastbc1 canary walks tall and stable at ~0.12 m/s under the full servo profile but OVERSPEEDS 2x the command (prog ratio det 1.95 vs the 0.75-1.25 band) because the vel:=ref obs contract gives it no speed feedback and nothing in the income prices the excess. This hardening rung warm-starts from fastbc1 and adds the existing command-tracking price (reward.k_walk_cmd_track=1.0, the same tested key the fastthru stack used) over 5M steps, same profile and 0.05-0.08 band. Single pre-authorized near-miss continuation from the fastbc1 gate.

**gate**: At 5M, DR-0 harness walk: prog_ratio med in 0.75-1.25 on BOTH det and sto (command obedience restored); det slip/m <= 1.8, sto <= 2.5; gait_valid 6/6 det+sto, zero falls; det realized speed >= 0.07 m/s at top-band commands (keeps the speed unlock - no regression to the 0.05-0.06 ceiling); roll_tail no worse than parent (1.1-2.3 deg). FAIL modes: overspeed persists = wrong lever, STOP and hand the fork to the operator; speed collapses to deployed levels = tracking bought by slowing down, STOP. PASS -> retention panel (friction 0.4-1.6x, 5deg tilt) + bulk session gate vs bcgait1_hard1; DOWNLOAD_ANSWER changes ONLY if it beats hard1 (operator order fb 20260820T224241Z).

**verdict**: FAIL vs pre-registered gate: k_walk_cmd_track=1.0 did not fix the fastbc1 overspeed, it worsened it. DR-0 prog_ratio det 1.88->2.10, sto 1.20->1.76 (band 0.75-1.25); own-DR sto prog_ratio 1.12->1.92 (was in-band, now overspeeds). Zero falls, gait_valid 6/6 det+sto, video still tall/clean six-leg (no exploit) - slip and dir_err improved slightly but that's not the gate. Matches the gate's own pre-registered FAIL mode: 'overspeed persists = wrong lever, STOP and hand the fork to the operator.'

