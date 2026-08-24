# cw-arch-hist16-dep1-c1-joyfullcurr12-certfreeze-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-24T13:41:53+00:00

**pod**: hexapod-mjx-train-0

**steps**: 40000000

**parent**: cw-arch-hist16-dep1-c1-joyfullcurr12-certfreeze

**wandb_id**: 5nsinbnj

**hypothesis**: Plain English: seed replicate 1 of the cert-only-freeze repair, launched as a batch (operator 08-22 batching order) so the seed-pass-rate question is answered in one wall-clock window instead of serialized cycles. certfreeze's PASS bar (joygate falls <=2/48) sits inside the parent lineage's seed-noise band (parent 1/48, parent+eval-freeze 2/48), so a single-seed read risks seed luck in either direction; n=3 seeds (s0 base + s1 + s2) gives an honest pass-rate on the decisive fork: does removing the freeze from TRAINING (keeping it cert-only) preserve the b1->b2+ promotion win without the 6-7/48 fall regression both training-freeze twins showed?

**gate**: Same per-arm gate as certfreeze: frontier promotes past b1 (>=b2) with real b2+ practice; held-out joygate falls <=2/48, dr0 dir_err ~<=36deg, slip <=2.9, no new leg-sacrifice signature. Grid read: repair VALIDATED if >=2/3 seeds pass both halves; if seeds split on the joygate half, the regression is seed-sensitive and the next lever is stress_mix in bucket training; if 0/3 promote, the cert-only freeze does not transfer promotion and the supervisor-assisted-cert ruling needs revisiting.

