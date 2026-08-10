# cw-dep-vref1-r1-velscale

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-10T16:28:11+00:00

**pod**: hexapod-mjx-train-1

**steps**: 20000000

**parent**: cw-dep-vref1-r1-latency

**wandb_id**: 5h3klp2o

**hardware_ready**: False

**hypothesis**: PROTECT THE NAMED HARDWARE CANDIDATE, tied to today's loaded-actuator finding (RL_LOG 08-10): measured loaded peak velocity 48-67 deg/s exceeds the AIR-fit sim's 30.8 deg/s ceiling by ~1.6-2.2x -- the loaded servo fit already raised the nominal ceiling to 48.5 deg/s, but vref1-r1 itself trained on the pre-fit AIR ceiling and has never seen its velocity-limit assumption randomized. dr.vel_scale (0.85-1.10 default range) widened to 0.6-2.2x directly probes whether the policy's gait timing depends on a precise (and now known-wrong) velocity ceiling. Per P0 rule 3, k_current=0. If-true: own-cfg det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band -- the policy doesn't rely on hitting a specific max joint speed, so the air/loaded ceiling mismatch is not itself a hardware risk. If-false: gait quality degrades outside the default 0.85-1.10 band -- the policy is timing-sensitive to actuator speed and the sim/hardware velocity-ceiling gap (P0 item 6) is a real pre-attempt-#2 risk, not just a modeling nicety.

**gate**: Own-cfg det+sto 6/6 @15s gait_valid, 0 term, slip/m within vref1-r1's own band (0.89-1.36); frames watched det for slowed/rushed stepping

**verdict**: Own-cfg (DR0+latency0.5-2.5x+vel_scale0.6-2.2x) gv 6/6 both passes, 0 term both, slip/m det med 1.30 / sto med 1.04 -- both inside vref1-r1's own combined band (0.89-1.36); gait stays valid and clean (video-checked det+sto, no flag-leg/skate) across the whole widened velocity-ceiling range. Note for the record: det progress_ratio spreads wide (0.58-1.20, only 2/6 clear the tight 0.85-1.15 success band) because covering a fixed 15s distance mechanically depends on the drawn joint-speed ceiling -- an expected physical consequence of THIS axis, not a gait defect (slip/gv/term all clean on the same episodes). Confirms gait quality is robust to the air/loaded velocity-ceiling mismatch (P0 item 6); walking speed itself is not, which is expected.

