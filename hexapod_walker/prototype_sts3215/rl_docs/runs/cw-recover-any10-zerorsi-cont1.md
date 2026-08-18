# cw-recover-any10-zerorsi-cont1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-16T10:35:57+00:00

**pod**: hexapod-mjx-train-1

**steps**: 20000000

**parent**: cw-recover-any9-lessfocus-cont1

**wandb_id**: kjl7unpn

**hardware_ready**: False

**hypothesis**: Teach the fallen robot to get up from lying completely flat on its belly by letting it PRACTICE from waypoints partway along the known-good stand-up demonstration, instead of only ever starting from flat. Dig-in on any8/any9 (both stuck at bucket 11 'zero' for their whole budgets) found the mechanism gap: the ladder's 'partial curl' rungs are linear joint blends, NOT states on the executable belly->plant rise path, so the stuck policy (splays to a low crouch, stalls over-current) never visits the mid-rise states it must learn to push through — while its imitation-anchor loss is already near-minimized (no headroom there). This arm is a matched A/B against any9: same stuck any8 checkpoint, same diffuse replay masses, same seed, ONE delta — the new RECOVER RSI mechanism (goal.recover_rsi_frac=0.5, snapshot a1994dee) spawns half of naturally-drawn zero-family episodes on a random row of the rise reference (belly curl through ~90% of the ramp, the exact lever/fraction that cracked stand-up for the footlow2 stance champion via goal.rise_rsi_frac). Certification exams are untouched by construction (forced kinds never RSI), so a PASS is a legitimately promoted frontier. Prediction-if-true: bucket 11 CERT climbs off 0 and reaches >=0.5, frontier resumes toward B12+ (tangle), and the late-run crouch/partial cert oscillation (churn from grinding a 0%-success frontier) calms. Prediction-if-false: zero stays at 0 even WITH on-path practice — the stuck-lineage rescue is then closed and the recovery line's live frontier returns to any7's tangle wall on the healthy lineage.

**gate**: Read at 20M (or earlier plateau): bucket 11 (zero) CERT success_fraction (pure forced-kind certs, RSI cannot touch them) must show a clear rising trend and reach >=0.5 in at least one of the last 5 certs for a PASS, AND buckets 0-10 retain >=0.8 gate_fraction at the final cert. FAIL (zero stays <0.2 throughout, or retention breaks) = on-path exposure does not rescue a stuck policy: close the any8/any9 stuck-lineage rescue entirely (no third warm-start), keep the RECOVER RSI mechanism for future from-scratch/any7-lineage arms, and return the recovery line's frontier to the any7 tangle wall.

**verdict**: RECOVER RSI (recover_rsi_frac=0.5, on-path belly->plant practice) did NOT rescue the stuck any8/9 lineage: bucket-11 (zero) CERT success stayed <=0.125 across every one of the last 5 certs (16,17,18,19,20M: 0, 0, 0, 0, 0.0625; never a rising trend), and the single-sample harness gate eval agrees exactly (zero 0/1 det, over_current termination, same flat-splay-then-stall video as any8/any9). Buckets 0-10 retention also broke at the final cert (bucket 9 gate_fraction 0.5625 < 0.8 bar). Per the pre-registered gate this CLOSES the any8/any9 stuck-lineage rescue entirely (no third warm-start); RSI is retained as a mechanism for future from-scratch/any7-lineage arms. Recovery line's live frontier reverts to any7 (B15, tangle+bank, bank solved / tangle contested).

