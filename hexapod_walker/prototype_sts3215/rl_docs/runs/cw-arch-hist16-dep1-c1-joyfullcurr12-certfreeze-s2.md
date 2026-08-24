# cw-arch-hist16-dep1-c1-joyfullcurr12-certfreeze-s2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-24T13:43:44+00:00

**pod**: hexapod-mjx-train-2

**steps**: 40000000

**parent**: cw-arch-hist16-dep1-c1-joyfullcurr12-certfreeze

**wandb_id**: k7jfofa9

**hypothesis**: Plain English: seed replicate 2 of the cert-only-freeze repair (see certfreeze-s1) -- third arm of the n=3 seed-pass-rate batch on whether cert-only freeze keeps the curriculum-promotion win without the training-time-freeze fall regression.

**gate**: Same per-arm gate as certfreeze: frontier promotes past b1 (>=b2) with real b2+ practice; held-out joygate falls <=2/48, dr0 dir_err ~<=36deg, slip <=2.9, no new leg-sacrifice signature. Grid read as registered on certfreeze-s1: repair VALIDATED if >=2/3 seeds pass both halves.

**verdict**: Result: FAIL, replicates the certfreeze base's exact failure shape -- seed-grid arm 2/3, closing the n=3 batch (all 3 seeds now read). Evidence: frontier promoted b0->b5 cleanly (walkcurr/frontier=5, precert b0/b1 both pass), matching base and s1 -- the cert-only-freeze frontier-unlock transfers to all 3 seeds. Held-out 60s joygate: falls 8/48 (dr0 5/24, dr0p5 3/24, the worst of the 3-seed grid) vs the <=2/48 gate; dir_err 42.72deg fails the 40deg allow; slip/m 2.18 ok. DR-0 gate det gait_valid 2/6 (sac legs [3,4]) and own-DR(0.5) det gait_valid 2/6 (sac [3,4]) -- the leg-3(+4) lock again, worse than base's 3/6. Why: third independent seed, identical root cause -- the b2+ heading-widening V6 diet damages stop-adjacent leg coordination regardless of seed. GRID CLOSED: 3/3 seeds promote frontier cleanly but 3/3 fail the joygate (6/48, 7/48, 8/48 falls, all over the 2/48 cap) and 3/3 show the leg lock at DR-0 -- uniform, not split, so per the grid's own read this is decisively NOT seed-sensitive; the cert-only-freeze repair does not transfer safety even though it transfers curriculum promotion. What's next: no further certfreeze arms of any seed. V7 (wz turning + reversal diet diversification on top of the same cert-only-freeze assist, already built+launched as cw-arch-hist16-dep1-c1-joyfullcurr13-certfreeze-v7) is the standing next lever; if it also fails, escalate to the heading-band-width axis per its own pre-registered FAIL branch.

