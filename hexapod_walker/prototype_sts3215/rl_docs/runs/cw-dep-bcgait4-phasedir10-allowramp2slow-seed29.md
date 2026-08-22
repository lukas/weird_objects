# cw-dep-bcgait4-phasedir10-allowramp2slow-seed29

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-22T16:31:48+00:00

**pod**: hexapod-mjx-train-2

**steps**: 4000000

**parent**: cw-dep-bcgait4-phasedir10-allowramp-seed29

**wandb_id**: 8ummjo1h

**hardware_ready**: no

**hypothesis**: Plain English: loosening the foot-drag price early helped one training seed but hurt the other two; this arm tests whether the WORST seed just needs the loose allowance held LONGER before the strict price kicks in. Single change vs cw-dep-bcgait4-phasedir10-allowramp-seed29 (FAIL, 0.725x/1.466x — worse than its own no-ramp longrun29 baseline 0.740x/1.296x): reward.drag_stance_allow_ramp_steps 1.2M->2.4M (post-anneal grace window; ramp_mm stays 48). 'Slower ramp' half of the untried branch in joystick STATUS Next item 3, on the harder seed; read together with the seed23 sibling, never alone. Grid launched as one batch per operator 08-22.

**gate**: Watcher DR-0 gate report, det mode, clone-relative rung-A: PASS = progress >=0.9x clone AND slip <=1.2x clone, zero falls, gait_valid 6/6. Compare against the no-ramp baseline (longrun29: 0.740x/1.296x) and the 1.2M/48mm ramp reading (0.725x/1.466x). Prediction-if-true: both axes improve vs both baselines. Prediction-if-false: flat/worse vs no-ramp — slower schedule refuted on this seed; combined with the seed23 sibling and the wide arms, a 0-for-4 grid CLOSES the ramp lever class for failing seeds and redirects to the matched-timing stance-slip mechanism (STATUS item b).

**verdict**: FAIL of the rung-A gate (det prog 0.78x clone vs 0.9x cap, slip 1.25x vs 1.2x cap), but the ONLY grid arm whose prediction-if-true nominally fired: better than both baselines on both axes (no-ramp 0.740x/1.296x, 48mm 0.725x/1.466x). Caveat: prog delta +0.044x is at the edge of 6-ep noise and slip delta -0.047x inside noise — weak evidence, mirrors seed13's one-off 48mm improvement (each ramp dose helps exactly one seed). Zero falls, gait 6/6, clean video.

