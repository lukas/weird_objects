# cw-standwalk-stance-mesh2-stage2-dualbc1-anchor5-stdmild1-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: CANARY FAIL - MECHANISM

**created**: 2026-08-27T00:51:53+00:00

**pod**: hexapod-mjx-train-3

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor4-stdanneal-s1

**wandb_id**: duwxunzn

**hypothesis**: Seed-1 twin of anchor5-stdmild1 (see that entry for the full dose-bracket hypothesis): --log-std-final -1.0 on top of anchor4-stdanneal-s1's own leak-fixed coef=3.0 dual-core checkpoint, testing whether this soft a dose protects walk while still helping hold, on the second seed.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY. Same panel as anchor5-stdmild1. WALK-SURVIVES if det gait_valid >=5/6 (no anchor1/anchor4-class freeze, prog_ratio >=~0.2). HOLD-HELPS if hold/sto DR-0 term improves meaningfully from anchor2-s1/anchor3-s1's 6/6 baseline. JOINT call with anchor5-stdmild1 (seed0) and anchor5-stdmild2/-s1 (the -2.0 dose pair) decides the dose-response: both doses protect walk on both seeds -> pick the dose with more hold improvement and promote; either dose wrecks walk on either seed -> magnitude is closed as a lever, per-core log_std split (DIG-IN) is next.

**verdict**: CANARY FAIL - MECHANISM (hold-helps branch, own-scope, seed1). Same -1.0 dose as its seed0 twin but this seed does NOT show the walk catastrophe -- gait_valid 6/6 DR-0 det, 6/6 own-DR det, 5/6 own-DR sto, prog_ratio med 0.29-0.30 (healthy anchor2/3 band), video-consistent with real cycling. But hold/sto stays 6/6 hold_min_load term at BOTH DR-0 and own-DR -- bit-for-bit the unchanged anchor2/3 baseline, zero improvement -- and hold/det picked up new terms (2/6 DR-0, 3/6 own-DR) vs anchor2/3's clean 0/6. So on this seed, walk survives but hold gets no help at all, matching stdmild2's own signature at a different dose. Combined with its seed0 twin (walk WRECKED, same zero hold-help) and stdmild2/-s1 (-2.0 dose: walk survives both seeds, hold still unhelped), the joint dose-bracket call across all four is now readable: no tested dose on the SHARED log_std (-1.0, -2.0, or the already-closed -4.0) both reliably protects walk across seeds AND meaningfully helps hold. -1.0 already fails cross-seed reliability (this pair) even before considering hold; -2.0 is seed-safe but hold-inert; -4.0 is hold-effective but walk-fatal both seeds. Per the track's own pre-registered rule this closes the anneal-MAGNITUDE lever on a single shared log_std; the per-core log_std split (DualGruActorCriticPolicy currently shares one log_std across both gated dual-cores) is now the pre-registered next lever -- see STATUS.md Next -1.8 for the design.

