# cw-standwalk-stance-mesh2-stage2-dualbc1-modeseq1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-26T11:57:03+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**wandb_id**: zygtbdyy

**hypothesis**: Plain English: we now have one candidate policy that already imitates BOTH skills at once (a dual-brain network BC-cloned this cycle from the mesh stance teacher acq8m for rise/hold/lower and the primitive walk teacher stotight45-seed13 for walking, verified this cycle to transfer to mesh dynamics/the real 100Hz motor contract almost for free and to compose via mode-switching at only a 10% fall rate concentrated in the ALREADY-TRACKED rise/hold residual, zero walk-composition pathology). This arm asks whether a modest RL fine-tune from that BC init (mode-gated dual-core GRU architecture, so walk-tick gradients cannot corrupt the stance cores by construction -- the exact lesson the cw-arch-gru-dual1 lineage already proved) holds or improves all three skills together, closing Stage-2's first walking-source x mechanism matrix cell (source=primitive stotight45-seed13 via direct-inference BC transfer; mechanism=dual-teacher BC + RL fine-tune). No train.bc_anchor_* this canary (one lever at a time: prove the bare fine-tune first; the arch-gru-dual1 precedent's own anchor-interference lesson is the pre-registered fallback if walk regresses).

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. 2M canary, own-cfg gate/owncfg/seqprobe (pod_eval auto) plus a DEDICATED flat-pinned composed probe (goal.mode_seq_stance=1 equivalent via goal.mode_seq single-mode fallback + rise_flat_frac=1.0) matched against the dualbc1 BC-only checkpoint's own probes (walk ep returns ~2600-2700, hold ~490-630, one seq PASS/one seq FALL out of 2). PASS if: hold/lower do not collapse below BC-parent (no majority min-load/over_current term) AND walk shows real per-leg swing/translation (not park/freeze) on video AND the composed seq fall rate is <= the BC-parent's own 10% (60-seq) read within noise. FAIL if hold/lower regress to majority-term OR walk freezes/paddles (the shared-trunk failure mode this dual-core architecture is specifically built to prevent) -- routes to the bc_anchor(stance-only, walk-off) fallback per the arch-gru-dual1 precedent. Read reward trend alongside eval per the 08-21 ruling.

