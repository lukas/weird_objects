# cw-amp-m3-pushhard1-style05-repeat3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T00:06:56+00:00

**pod**: hexapod-mjx-train-2

**steps**: 6000000

**parent**: cw-amp-m3-pushacq1-style05

**wandb_id**: ywnpdlmn

**hypothesis**: Plain English: the push-hardened walker has stopped improving under ONE 10-25N shove per episode (training tilt-terminations flat over pushacq1-style05's final 3M at constant dose), and milestone M3 explicitly demands surviving REPEATED pushes — so raise exposure from one to up to three shoves per episode (dr.ext_push_repeat_max=3, gaps 1-3s, per-shove dose unchanged, horizon 13s of the 15s episode) and see whether learning resumes and multi-push survival emerges. Warm-start from the pushacq1-style05 checkpoint, style kept (it passed acquisition with style_reward 0.116>0.1), 6M, DR-0. Companion arm -n2040 escalates FORCE instead of COUNT — together they decompose which axis is the M3 frontier. Prediction-if-true: tilt terminations first rise (3x exposure) then fall over the run, and the DR-0 own-cfg gate holds gait_valid >=5/6 with topples <=2/6 det, <=3/6 sto under multi-push episodes. Prediction-if-false: terminations stay high/flat all run — recovery capacity, not exposure count, is the binding constraint. Strongest alternative: first-push survival transfers so completely that 3-push episodes are barely harder (terms hardly rise at start) — that would say push COUNT is already solved and force is the real frontier.

**gate**: Hardening (6M, DR-0, dr.ext_push_repeat_max=3, per-shove dose unchanged 10-25N). PASS = DR-0 own-cfg gate gait_valid >=5/6 det+sto, zero sacrificed legs, det prog med >=0.9, topples <=2/6 det AND <=3/6 sto, style_reward_mean >0.1 at end, video shows repeated stumble-and-keep-walking within one episode. INFORMATIVE-ceiling = topples above bar but training tilt-terms still falling at cutoff => continue per 08-21 ruling. INFORMATIVE-transferred = tilt-terms barely rise at start vs pushacq1-style05's floor and gate matches PASS bar trivially => push count already solved, force is the frontier. FAIL-statue = det prog med <0.6 or crouch fingerprint (height band exit). FAIL = collapse/NaN.

