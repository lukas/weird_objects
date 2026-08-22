# cw-amp-m2-sec5-noamp

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-22T19:15:31+00:00

**pod**: hexapod-mjx-train-9

**steps**: 2000000

**parent**: cw-amp-m2-styleonly-v2-c1b

**hypothesis**: Plain English: does the AMP brief's own literal section-5 minimal task reward (plain Gaussian velocity-tracking kernel + linear progress + the shared upright/height/action/current regularizers, zero of the SLIPWALK anti-slip/anchor/idle-charge/gait-gate/drag-stance apparatus every one of the 9 prior from-scratch M2 arms reused, plus the already-validated reward.term_penalty=400 anti-suicide fix and goal.walk_pure=1.0 to close the M2-c1 mixed-goal-mode statue exploit) let a from-scratch policy actually walk on its own, before any AMP style is added? Task-only control for the sec5-{taskA,taskB,taskC} sweep (launched this same cycle) -- isolates what the minimal task reward alone buys, matching the AMP_MINIMAL_OVERRIDES bank's own scripted-twin reward (rl_move/tests/test_task_semantics.py) exactly in a real trained (not scripted) setting. Same held envelope as the sweep (speed 0-0.25 m/s, yaw +/-0.5 rad/s, matching the retired freeprog family for comparability).

**gate**: Discovery (2M steps, judged on det video (3+ episodes) + gait_valid/fwd-travel harness numbers, NOT the joystick DONE gate). INFORMATIVE-PASS = det video shows real net forward travel (>=0.10 m/15s) with visibly cyclic multi-leg contact/swing. FAIL-same-statue = frozen/half-tripod/march-in-place basin persists despite dropping the whole SLIPWALK stack -- if this control ALSO fails while a style-weighted sibling passes, that isolates AMP style (not the reward rewrite) as the actual fix; if this control also fails and every sibling fails identically, closes the reward-architecture-alone hypothesis per q_20260822T1815Z. Read jointly with sec5-taskA/B/C.


## Triage notes (08-22 ~19:4x cycle, evals read, verdict deferred to dig-in)

- W&B finished at 2,031,616 steps (checkup "stall at 2007040" = normal
  completion of the 2M arm, not a hang; no rescue needed).
- DR-0 gate (cw_amp_m2_sec5_noamp_gate): det prog med 0.02 / slip med
  11.34 / fwd med 0.03m / gait_valid 1/6, leg [3] sacrificed in 5/6
  det episodes; sto prog 0.05 / slip 11.46 / fwd 0.04m / gv 6/6 (but
  no travel). Bar was fwd >= 0.10m/15s. Zero terminations.
- Contact sheet + det videos: crouched splayed statue, no net
  translation — the same basin as taskA/B/C and every prior M2 arm.
- Training reward DECLINED every quarter (-60.0/-72.2/-122.0/-132.5).
  env/height_err_mm 5.8 -> 83.8 (Q1) -> 70.2 (end): immediate crouch
  away from the upright target, never recovered. Reward-falling +
  eval-flat = genuine stuck mechanism, NOT the 08-21 rising-reward
  continue case.
- Pre-registered FAIL-same-statue branch fires; as the task-only
  CONTROL failing alongside all style-weighted siblings, this closes
  the reward-architecture-alone hypothesis per the run's own gate
  (q_20260822T1815Z) and points at task restructuring or a
  BC-pretrain phase.
- Cross-arm mechanism note for the dig-in: end-of-run height_err falls
  monotonically with style weight (noamp 70.2 / taskA@0.3 58.5 /
  taskC@0.7 53.8mm) — style pushes against the crouch but is far too
  weak; and ALL arms have negative per-tick income, so
  crouch-and-survive (term_penalty=400 avoidance) may be the priced
  optimum — audit pricing before choosing between the two levers.
