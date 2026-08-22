# cw-amp-m3-pushsmoke1-noamp-r4

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-22T23:06:19+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m2-bcinit-sec5-noamp-headingsfull

**hypothesis**: Plain English: this is the FIRST-EVER training use of the just-built dr.ext_push_* mechanism (a mid-episode horizontal shove, distinct from the existing dr.walk_push_* takeoff-only roll torque) -- can the already-walking full-circle BC-init checkpoint tolerate being randomly shoved mid-stride (10-25N, 0.15-0.4s pulse, random horizontal direction, once per episode at a random time 1.5-9s in) without the training signal collapsing, and does video show real recovery attempts rather than instant catastrophic falls every time? This is a MECHANISM-SAFETY smoke (per the fault-injection precedent, cw-amp-m4-faultsmoke1), not a graded M3 push-recovery gate -- M3 was NOT STARTED before this. Continues from cw-amp-m2-bcinit-sec5-noamp-headingsfull (the just-verdicted full-circle-heading walking checkpoint, --init-from-source), single lever: dr.ext_push_prob 0.0 -> 1.0 (dose menu at its just-built defaults, force 10-25N / duration 0.15-0.4s / start delay 1.5-9s / world-frame random direction -- chosen so the resulting impulse, ~3-12 N*s at the low-mass 2.1kg chassis, is a meaningful shove without being an instant unrecoverable launch, per the on-pod Warp smoke that showed a 30N/0.2s pulse producing a large but not exploding displacement). Prediction-if-true: reward finite and rising through 2M (same shape as faultsmoke1-noamp's 9->209/quarter), DR-0 gate gait_valid stays >=4/6 det+sto despite the pushes, video shows stumble-then-recover behavior on at least some episodes (not every push ends in an immediate topple). Prediction-if-false-A: reward flat/declining and video shows catastrophic falls on every pushed episode -- the dose is too aggressive for a first exposure, next arm should curriculum the force in from a smaller starting dose. Prediction-if-false-B: gait_valid collapses to 0/6 even off-push (mechanism itself is buggy/leaking into non-pushed ticks) -- code bug, not a training result; the xfrc_applied ownership-gated write (test_ext_push_injection.py::test_off_episode_never_touches_xfrc_row_0_3) should already rule this out but the CPU test suite cannot see the batched Warp path. Strongest alternative: partial -- the policy tolerates the push near the low end of the dose range but topples reliably near the high end, motivating a push-magnitude curriculum (brief 7.4's own stated design) rather than a flat dose.

**gate**: Mechanism-safety smoke (2M, DR-0, own-cfg with dr.ext_push_prob=1.0), same bar as cw-amp-m4-faultsmoke1-noamp: PASS = finite losses/reward the whole 2M (no NaN/crash), reward trend not catastrophically worse than the faultsmoke1-noamp shape, DR-0 gate at own cfg shows gait_valid >=3/6 det (visible six-leg cycling and net travel in the video, not a frozen statue) with pushed episodes showing SOME recovery signature (a stumble/regain, not instant collapse every single time). FAIL = NaN/crash, or gait_valid 0/6 with every episode toppling immediately on the push (dose miscalibrated -- needs a curriculum-in follow-up before any further push-recovery work).

