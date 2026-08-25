# cw-standwalk-stance-mesh2-loweronly-bcchain3-stdanneal

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-25T13:57:43+00:00

**pod**: hexapod-mjx-train-0

**steps**: 8000000

**parent**: cw-standwalk-stance-mesh2-loweronly-bcchain3

**hypothesis**: The robot already sits down cleanly when executed without action noise; this arm asks whether annealing away the exploration noise makes that sit-down robust, exactly as it did for standing. Lower acquisition off the rung-8 CANARY PASS (loweronly-bcchain3: DR-0 det 6/6 honest descents, height_err_end 0.1-3.7mm, zero over_current, zero terms; sto 0/6 all fell under un-annealed policy_std~1.0 — the same signature the hold rung had before bcanchor3-stdanneal took sto 0/6->6/6 by log-std anneal alone). Warm-start from the canary checkpoint, 8M, log-std 0->-4.0 over first 50%, identical to the hold stdanneal recipe. Secondary read: does anneal also cool the hot crouch (canary det cur_max 2.17-2.26A, cur_s_above_soft up to 10.2s) the way it cooled hold (0.53->0.44A)? If sto passes but current stays pinned hot, the fallback is goal.lower_height_mm belly-rest recalibration per the 08-25 dig-in, not more pricing.

**gate**: 8M acquisition. Lower DR-0 det+sto n=6+6 AND own-DR(0.2) det+sto n=6+6. PASS: sto >=4/6 honest descents at DR-0 (>=60% commanded drop, feet grounded, no fall) with det >=5/6 preserved and zero over_current terms in det; report cur_p95/cur_max vs the canary's 2.17-2.26A. PARTIAL: sto improves materially (1-3/6 or falls shifting late / cur_max trending down) but short of bar — budget or dose follow-up. FAIL: sto stays 0/6 with det intact — noise-floor is not the lower sto blocker; fork to goal.lower_height_mm belly-rest recalibration (crouch intrinsically too hot on mesh) before any DR-exposure fix.

**refused_reason**: acquisition runs require --evidence: name the healthy canary and a comparable full-budget learning precedent.

