# cw-amp-m3-pushhard1-style05-n2040

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-23T00:10:47+00:00

**pod**: hexapod-mjx-train-1

**steps**: 6000000

**parent**: cw-amp-m3-pushacq1-style05

**wandb_id**: dztnp1p9

**hypothesis**: Plain English: the push-hardened walker has stopped improving under one 10-25N shove per episode (training tilt-terminations flat over pushacq1-style05's final 3M at constant dose) — this arm escalates the OTHER axis vs its -repeat3 sibling: keep ONE shove per episode but make it substantially harder (dr.ext_push_n=20-40N, duration/timing unchanged). Does the walker learn to survive ~1.6x stronger pushes, or is ~25N near the recoverable ceiling for this morphology at this stance? Warm-start from the pushacq1-style05 checkpoint, style kept, 6M, DR-0. Prediction-if-true: tilt terminations spike at the start (harder dose) then fall over the run; DR-0 own-cfg gate holds gait_valid >=5/6 with topples <=2/6 det, <=3/6 sto at 20-40N. Prediction-if-false: terminations stay high AND flat with reward flat — 40N-class shoves are beyond recoverable for this build; that names a physical dose ceiling for the M3 spec (informative, not a lineage kill). Strongest alternative: survival improves via crouch-statue (drop stance, stop walking) — pricing problem to fix before any M3 claim, watch height band + prog.

**gate**: Hardening (6M, DR-0, dr.ext_push_n=20-40N single shove, timing unchanged). PASS = DR-0 own-cfg gate gait_valid >=5/6 det+sto, zero sacrificed legs, det prog med >=0.9, topples <=2/6 det AND <=3/6 sto at the 20-40N dose, style_reward_mean >0.1 at end, video shows stumble-and-keep-walking. INFORMATIVE-ceiling = topples above bar with training tilt-terms still falling at cutoff => continue per 08-21 ruling; topples above bar AND terms flat => 20-40N dose ceiling named for the M3 spec. FAIL-statue = det prog med <0.6 or crouch fingerprint (height band exit). FAIL = collapse/NaN.

**verdict**: The style-kept walker clears its pre-registered 20-40N single-shove hardening bar: topples 2/6 det + 2/6 sto (bar <=2 det / <=3 sto), gait_valid 12/12, zero sacrificed legs, det prog med 1.23 / slip 2.98, style_reward_mean ends 0.13 (>0.1 bar, though still thinning 0.29->0.13 — the discriminator signal keeps eroding as recovery transients dominate), and video shows real stumble-and-keep-walking on survivors (det_0: upright six-leg cycling through the shove) with the 2 det topples being genuine full flips onto the back, not statues or crouches. HONEST JOINT READ vs the noamp twin: both arms land 4/12 total topples at this dose with overlapping quality stats (style05 det slip 2.98 vs noamp 4.05, sto slip 4.37 vs 3.83 — mixed, within noise) — the differing verdicts reflect differing pre-registered bars (noamp's was tighter, earned by its 0/12 parent floor), so style is STILL not load-bearing on the push axis, consistent with every M2/M3 style-vs-nostyle read so far. 20-40N robustness itself remains unfinished; the force-curriculum follow-ups run on the noamp line this cycle (pushcur1-noamp-b1530 + n2040-c1), with style05 escalation cheap to respec if the curriculum works. Hardware-ready: no.

