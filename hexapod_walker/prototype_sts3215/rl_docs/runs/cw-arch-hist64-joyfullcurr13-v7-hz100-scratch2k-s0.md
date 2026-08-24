# cw-arch-hist64-joyfullcurr13-v7-hz100-scratch2k-s0

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-24T18:48:48+00:00

**pod**: hexapod-mjx-train-2

**steps**: 40000000

**parent**: cw-arch-hist16-dep1-c1-joyfullcurr13-v7-hz100

**hypothesis**: Plain English: retry-once of the SIGBUS'd from-scratch 100 Hz arm (cw-arch-hist64-joyfullcurr13-v7-hz100-scratch-s0 died to /dev/shm exhaustion, zero steps — n_envs cut 3072->2048 so the hist64 obs shm buffer fits the 64MB tmpfs; science unchanged). Two transplant attempts proved a 25 Hz brain cannot be weight-converted to 100 Hz (naive: precert prog 0.203; verified rate-matched inputs: prog 0.052 — worse, leading mechanism: per-decision motion authority quartered under the preserved 37.5 deg/s slew), so per fb_20260824T180427_4c2e26's fallback this is the LABELED from-scratch arm: V7 certfreeze joystick ladder, control.hz=100, max_delta_q_deg=0.375, obs.history_frames=64 (rate-converted 640 ms context), fresh init BY DESIGN — the fresh init IS the hypothesis (rate-native action rhythm must be learned, not transplanted). Complementary to the running -r2 adaptation arm. RATE CAVEAT: 40M ticks at 100 Hz = 1/4 the simulated seconds of a 25 Hz 40M run. Prediction-if-true: the ladder climbs from b0 with reward and frontier promotions rising together. Prediction-if-false: b0 never clears with flat reward at adequate budget — next is a rate-normalized action/obs contract audit, not seeds. Strongest alternative: learns but slower purely from the 4x sim-seconds deficit — continuation candidate per 08-21.

**gate**: PASS: frontier promotions past b0 with reward/eval AGREEMENT at 100 Hz AND 60s randomized joygate at 100 Hz: falls<=2/48, no over_current cluster, directions followed on side/rear/turn/reversal stress, slip <=~2.9/m, video shows all six feet cycling. PARTIAL: genuinely learning (b0+ promotions, improving rung evals) but short of the 25 Hz lineage at budget end — continuation candidate (expected: 1/4 sim-seconds). FAIL: b0 never promotes with reward AND rung metrics flat at adequate budget, or reward rises while rungs stay flat (100 Hz reward/eval mismatch — audit before any seed sweep).

