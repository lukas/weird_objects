# cw-amp-m3-pushhard1-style05-repeat3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-23T00:06:56+00:00

**pod**: hexapod-mjx-train-2

**steps**: 6000000

**parent**: cw-amp-m3-pushacq1-style05

**wandb_id**: ywnpdlmn

**hypothesis**: Plain English: the push-hardened walker has stopped improving under ONE 10-25N shove per episode (training tilt-terminations flat over pushacq1-style05's final 3M at constant dose), and milestone M3 explicitly demands surviving REPEATED pushes — so raise exposure from one to up to three shoves per episode (dr.ext_push_repeat_max=3, gaps 1-3s, per-shove dose unchanged, horizon 13s of the 15s episode) and see whether learning resumes and multi-push survival emerges. Warm-start from the pushacq1-style05 checkpoint, style kept (it passed acquisition with style_reward 0.116>0.1), 6M, DR-0. Companion arm -n2040 escalates FORCE instead of COUNT — together they decompose which axis is the M3 frontier. Prediction-if-true: tilt terminations first rise (3x exposure) then fall over the run, and the DR-0 own-cfg gate holds gait_valid >=5/6 with topples <=2/6 det, <=3/6 sto under multi-push episodes. Prediction-if-false: terminations stay high/flat all run — recovery capacity, not exposure count, is the binding constraint. Strongest alternative: first-push survival transfers so completely that 3-push episodes are barely harder (terms hardly rise at start) — that would say push COUNT is already solved and force is the real frontier.

**gate**: Hardening (6M, DR-0, dr.ext_push_repeat_max=3, per-shove dose unchanged 10-25N). PASS = DR-0 own-cfg gate gait_valid >=5/6 det+sto, zero sacrificed legs, det prog med >=0.9, topples <=2/6 det AND <=3/6 sto, style_reward_mean >0.1 at end, video shows repeated stumble-and-keep-walking within one episode. INFORMATIVE-ceiling = topples above bar but training tilt-terms still falling at cutoff => continue per 08-21 ruling. INFORMATIVE-transferred = tilt-terms barely rise at start vs pushacq1-style05's floor and gate matches PASS bar trivially => push count already solved, force is the frontier. FAIL-statue = det prog med <0.6 or crouch fingerprint (height band exit). FAIL = collapse/NaN.

**verdict**: PASS on every pre-registered bar: the style-kept push walker now survives up to THREE 10-25N shoves per episode. DR-0 own-cfg gate: gait_valid 6/6 det + 6/6 sto, zero sacrificed legs, det prog med 1.18 / slip med 2.78, topples 1/6 det (det/1 tilt_roll) + 0/6 sto, style_reward_mean 0.109 (>0.1 bar, still thinning 0.85->0.11). Recovery-without-reset is on mechanical record: 10/12 episodes roll_class=recovered (det/0 rode a 16.9deg roll spike back to 1.0deg tail while covering 1.21m; sto/2 19.7deg->1.3deg), strips show upright six-leg walking throughout with no crouch (height_err_end 1.8mm), no flag leg, no paddle-creep. Blunt residuals: pushes cost HEADING - det dir_err mean 38.7deg and per-episode fwd travel scatters 0.09-1.21m, so command tracking is useful but not tight under fire; and training tilt-terms went UP ~3x at the 3-shove dose start (pitch ~20, roll ~18/window vs parent floor 6-7/5-6.5) then stayed roughly flat across 6M (pitch dipped 21->15->8 only in the last windows) with reward flat ~200-225 in the second half - eval-time multi-push survival was mostly already present in the parent, and same-dose training budget is again near-exhausted. Joint grid read: push COUNT is effectively solved at 10-25N; FORCE (n2040 sibling, pending) is the frontier. Next: grid-completing combo arm (repeat3 ckpt + 20-40N, repeat_max kept) launched this cycle.

