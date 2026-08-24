# cw-arch-hist16-dep1-c1-joyfullcurr12-certfreeze-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-24T13:41:53+00:00

**pod**: hexapod-mjx-train-0

**steps**: 40000000

**parent**: cw-arch-hist16-dep1-c1-joyfullcurr12-certfreeze

**wandb_id**: 5nsinbnj

**hypothesis**: Plain English: seed replicate 1 of the cert-only-freeze repair, launched as a batch (operator 08-22 batching order) so the seed-pass-rate question is answered in one wall-clock window instead of serialized cycles. certfreeze's PASS bar (joygate falls <=2/48) sits inside the parent lineage's seed-noise band (parent 1/48, parent+eval-freeze 2/48), so a single-seed read risks seed luck in either direction; n=3 seeds (s0 base + s1 + s2) gives an honest pass-rate on the decisive fork: does removing the freeze from TRAINING (keeping it cert-only) preserve the b1->b2+ promotion win without the 6-7/48 fall regression both training-freeze twins showed?

**gate**: Same per-arm gate as certfreeze: frontier promotes past b1 (>=b2) with real b2+ practice; held-out joygate falls <=2/48, dr0 dir_err ~<=36deg, slip <=2.9, no new leg-sacrifice signature. Grid read: repair VALIDATED if >=2/3 seeds pass both halves; if seeds split on the joygate half, the regression is seed-sensitive and the next lever is stress_mix in bucket training; if 0/3 promote, the cert-only freeze does not transfer promotion and the supervisor-assisted-cert ruling needs revisiting.

**verdict**: Result: FAIL, replicates the certfreeze base's exact failure shape -- seed-grid arm 1/3 (n=3 seed-pass-rate batch on the cert-only-freeze repair). Evidence: frontier promoted b0->b5 cleanly (walkcurr/frontier=5, precert b0/b1 both pass), matching base -- the cert-only-freeze frontier-unlock DOES transfer to this seed. But the held-out 60s joygate is worse, not better: falls 7/48 (dr0 5/24, dr0p5 2/24) vs the <=2/48 gate and vs base's own 6/48; dir_err combined 43.2deg fails the 40deg allow (base's combined read was inside band); slip/m 2.19 ok. DR-0 gate det gait_valid crashed to 4/6 (sacrificed leg index 2 = 'leg 3', same signature as base) with prog_ratio 0.85/slip 1.50 on the clean episodes; own-DR(0.5) det gait_valid 2/6 (sac leg 3 again). Contact sheet (walk_det_0) shows the same rigid held-aloft rear leg across most of the clip as base's leg-3 lock video. Why: identical root cause to base -- 3/3 independent seeds of this exact recipe now show the b2+ heading-widening V6 diet itself damaging stop-adjacent leg coordination, not a seed-luck artifact. What's next: no further action on this arm -- V7 (wz turning + reversal diet diversification, already built+launched as cw-arch-hist16-dep1-c1-joyfullcurr13-certfreeze-v7) is the correct next lever and needs no seed batch of its own until its single read comes back.

