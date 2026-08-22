# cw-amp-m2-sec5-noamp

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-22T19:15:31+00:00

**pod**: hexapod-mjx-train-9

**steps**: 2000000

**parent**: cw-amp-m2-styleonly-v2-c1b

**hypothesis**: Plain English: does the AMP brief's own literal section-5 minimal task reward (plain Gaussian velocity-tracking kernel + linear progress + the shared upright/height/action/current regularizers, zero of the SLIPWALK anti-slip/anchor/idle-charge/gait-gate/drag-stance apparatus every one of the 9 prior from-scratch M2 arms reused, plus the already-validated reward.term_penalty=400 anti-suicide fix and goal.walk_pure=1.0 to close the M2-c1 mixed-goal-mode statue exploit) let a from-scratch policy actually walk on its own, before any AMP style is added? Task-only control for the sec5-{taskA,taskB,taskC} sweep (launched this same cycle) -- isolates what the minimal task reward alone buys, matching the AMP_MINIMAL_OVERRIDES bank's own scripted-twin reward (rl_move/tests/test_task_semantics.py) exactly in a real trained (not scripted) setting. Same held envelope as the sweep (speed 0-0.25 m/s, yaw +/-0.5 rad/s, matching the retired freeprog family for comparability).

**gate**: Discovery (2M steps, judged on det video (3+ episodes) + gait_valid/fwd-travel harness numbers, NOT the joystick DONE gate). INFORMATIVE-PASS = det video shows real net forward travel (>=0.10 m/15s) with visibly cyclic multi-leg contact/swing. FAIL-same-statue = frozen/half-tripod/march-in-place basin persists despite dropping the whole SLIPWALK stack -- if this control ALSO fails while a style-weighted sibling passes, that isolates AMP style (not the reward rewrite) as the actual fix; if this control also fails and every sibling fails identically, closes the reward-architecture-alone hypothesis per q_20260822T1815Z. Read jointly with sec5-taskA/B/C.

**verdict**: FAIL-same-statue, the task-only control arm (identical section-5 minimal reward, zero AMP flags). Evidence: DR-0 gate det gait_valid 1/6, prog med 0.02, fwd med 0.03m/15s (bar 0.10), slip 11.34; sto gait_valid 6/6 but fwd 0.04m, slip 11.46. Contact sheet watched: same crouched splayed statue, zero translation. Reward declines all run (-60.0/-72.2/-122.0/-132.5) with the same Q1 crouch (height_err 59->86mm). Grid-level read now FINAL 0/4 (taskA/B/C + noamp all statue): the crouch/statue basin exists WITHOUT AMP and is not rescued at any style dose — the section-5 minimal reward is NOT the M2 fix; the from-scratch exploration problem is deeper than reward shape. Per the grid's own pre-registered prediction-if-false and brief sec.4.3/sec.14, the next real lever is a gait-initialized actor (BC-pretrain from the verified teacher/clone as initialization only) or task restructuring — no further reward-ratio arms.

