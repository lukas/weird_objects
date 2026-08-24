# cw-walkcurr-pf-fwd6-rscale50-sde-actbias1-idleterm1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-24T03:10:24+00:00

**pod**: hexapod-mjx-train-5

**steps**: 2000000

**parent**: cw-walkcurr-pf-fwd6-rscale50-sde

**hypothesis**: Plain English: covers the named failure branch of the sde-actbias1 sibling -- if the recentered stable stance ABSORBS the gSDE excursions back into a clean park-stand (exactly what actbias1 alone converged to), the qvel idle-terminate mechanism evicts that absorbing state so the only long-episode income left is actual walking, while the sde noise supplies the directional escapes idleterm's raw-a=0 lineage never had. Full three-ingredient stack (stable zero-point + park eviction + correlated exploration), each ingredient individually built, bank-proven and behavior-verified; fresh 2M discovery. Prediction-if-true: episodes stop running full length from a park (idle-terminate fires), freeprog leaves the dead band and det gate shows stepping where sde-actbias1 alone parks. Prediction-if-false (evicted park just cycles into re-park or falls): parking is priced AND evicted AND escapable yet still wins -- strong evidence the discovery gap is value-initialization, feeding the BC-kickstart operator question (q_20260824T0233Z) with the completed lever inventory.

**gate**: Same rung-1 gate as every fwd6 arm: C-env det fixed-forward panel -- zero tilt terms, cmd_prog_frac >= 0.35, direction_err <= 30 deg, slip/m <= 3.0, six legs cycling on >=4/6 episodes. Mechanism-health: idle-terminate fire rate > 0 if parking recurs; tilt-term rate vs sde parent; freeprog vs dead band.

