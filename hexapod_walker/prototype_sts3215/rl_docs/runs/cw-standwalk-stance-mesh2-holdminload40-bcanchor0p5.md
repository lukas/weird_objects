# cw-standwalk-stance-mesh2-holdminload40-bcanchor0p5

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-25T12:19:04+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-holdminload40

**wandb_id**: vggrigsq

**hypothesis**: Rung-7 dose floor sibling of bcanchor1/bcanchor3: completes a 3-point dose curve (0.5/1.0/3.0) for the joint-space imitation anchor in one wave rather than serializing across cycles (operator 08-22 batching guidance). Same single-lever-off-holdminload40 base.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. Same as bcanchor1/3: 2M canary, hold DR-0 det+sto n=6+6, >=4/6 det valid_plant + cur_p95<=1.5A + rising hold_feet_factor/hold_load_factor = PASS; 1-3/6 = PARTIAL/continue; 0/6 pinned-signature = FAIL. Joint 3-arm dose read decides which dose (if any) funds an 8M acquisition arm.

**verdict**: CANARY FAIL - MECHANISM (mechanism-health scope) -- dose 0.5 is too low to sustain the hold. FINAL synthesis after two independent reads (this cycle initially mis-called it PASS, then self-corrected; a concurrent cycle independently reached FAIL on the same evidence -- both converge here): DR-0 det shows valid_plant=True and cur_p95<=1.5A on all 6/6 episodes, which would clear the gate letter, BUT every one of those 6 episodes TERMINATES EARLY via hold_min_load at t~3.5s (return ~57.8, not the full 15s) -- i.e. even under pure deterministic policy execution with zero action noise, one foot's load fraction drops below the safety floor within seconds. Contrast with bcanchor1 (dose 1.0) and bcanchor3 (dose 3.0): both hold the FULL 15s with ZERO terminations in DR-0 det. valid_plant reads true only because it grades geometry/height/current at the termination instant, not sustained load balance -- a near-honest pose that still cannot hold itself up. Own-DR(0.2) det: 5/6 valid_plant but only 1 of 6 runs to truncation without any termination. Sto 0/6 (same un-annealed-policy_std signature as siblings). Read: the BC-anchor mechanism itself is confirmed (a real six-foot plant is reachable at every dose tried, unlike any of the six prior pure-pricing rungs) but 0.5x sits below the dose floor needed to sustain it even without noise -- 1.0x and 3.0x are the genuine passers. Does not change the acquisition pick: the concurrent cycle already funded the 8M std-anneal arm off dose 3.0 (the cleanest) before either read landed.

