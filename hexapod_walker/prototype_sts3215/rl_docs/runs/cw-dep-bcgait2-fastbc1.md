# cw-dep-bcgait2-fastbc1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-20T23:01:57+00:00

**pod**: hexapod-mjx-train-7

**steps**: 2000000

**parent**: cw-dep-bcgait1

**hypothesis**: Make the sim robot walk noticeably faster without losing direction or footing: re-run the proven clone-the-scripted-tripod-gait-then-RL-fine-tune recipe, but with the servos on their FAST profile (1500/80/5deg) from step 0 and the gait clock UNCHANGED. Operator order 08-20 (fb 20260820T224241Z) asked for a faster-cadence BC teacher; the ordered shorter-period knob FAILED its own teacher preflight at every rung (period_scale 0.9/0.75/0.6 strictly worse at native/mid/full profile: progress collapses 0.76->0.65->0.57->0.10-0.30, slip explodes - probe_fastcad_* on train-0..6), while the SAME grid proved the full profile safe for the native-cadence teacher: progress 0.73-0.76 (~2x native realized speed), slip/m 1.6-3.0, 147mm tall, clean 6-leg tripod, zero falls. The 4 failed fastthru/fastramp canaries transplanted a slow-profile-trained policy under this profile; this arm instead BC-inits from a fresh clone of the teacher that provably walks under it (clone preflight prog 0.63-0.89 all seeds/dirs, slip ~2.7, 147mm, zero terminations, holdout act err 0.013). Speed band 0.05-0.08, the ladder rung below 0.10.

**gate**: At 2M, DR-0 harness det walk: gait_valid 6/6, zero falls; det realized speed >= 0.06 m/s mean (parent cw-dep-bcgait1 hit 0.067 at band max 0.06; this arm must use the 0.08 band without spinning); direction clean (no steer6-style spin: wz ~0, heading tracks command); det slip/m <= 2.2 (parent's own 2M level); walking height in-band (no crouch reversion). FAIL modes: profile-collapse falls/walk_low_height like fastthru = park fast-gait line for operator; stall-tall-no-travel = income too weak (bcgait1 fail-mode b, one pre-authorized retry lever). PASS -> hard1-style hardening rung; NO DOWNLOAD_ANSWER change unless a hardened child beats bcgait1_hard1 on the session gate (operator order).

**refused_reason**: hexapod-mjx-train-7 code marker 6b00684cccc98297075bc6296d230f4fd087031a != local HEAD 1870435fa81118b217180279649e40c31faf5bd1. Sync first: snapshot.sh --sync hexapod-mjx-train-7 (and snapshot/commit before that if the tree is dirty).

