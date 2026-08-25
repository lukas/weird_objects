# cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref-flatmix70-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: CANARY FAIL - MECHANISM

**created**: 2026-08-25T17:47:12+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref-flatmix70

**wandb_id**: xz8urts2

**hypothesis**: Seed-1 hedge for flatmix70 (double flat-start exposure 0.35->0.70 within the non-rsi half, single lever, from-scratch): is the flat-exposure lever's effect (if any) recipe-level or seed-luck? Exact flatmix70 recipe, only seed changed, mirroring every other mechanism canary this campaign hedged (bcanchor3-s1, loweronly-bcchain3-s1, meshref-s1). Judged jointly with flatmix70 as a 2-seed pair before any 8M acquisition commitment.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. Same flat-pinned probe as flatmix70 (--cfg-set goal.rise_flat_frac=1.0 --cfg-set goal.rise_partial_frac=0.0 --cfg-set goal.rise_rsi_frac=0.0, det+sto n=6+6, DR-0) plus the standard DR-0 gate for non-flat kinds. Joint pair read: PASS if BOTH seeds clear flat det>=4/6 AND sto>=4/6 valid_plant with zero over_current and non-flat kinds not regressed -> promote 8M + port into stancemix. PARTIAL if seeds disagree or both land in the PARTIAL band (>=2/6 flat valid or over_current halved) -> dose flat higher or promote with caution. FAIL if both plateau at 0-1/12 flat valid with the same never-tucks press-up signature -> exposure refuted as the lever regardless of seed, next is ref-content/phase treatment (tuck-phase anchor dose or tuck-segment curriculum).

**verdict**: CANARY FAIL - MECHANISM: doubling flat-start practice did NOT teach the robot to tuck its splayed front legs — and this cycle found out why no amount of practice could have: the training signal itself skips the tuck. Seed-1 flat-pinned probe 0/12 valid_plant, all 12 episodes over_current-pinned at exactly 2.64A with the same never-tucks radial press-up (h_err_end 13-62mm); standard DR-0 gate det 0/6 (the 0.70-flat mix sampled 4 flat + 2 rsi det starts, and BOTH rsi det starts are now dragged into the same 2.64A press-up basin — a non-flat regression vs the meshref canaries' det 5/6), sto 4/6 (rsi+bridge clean at 0.56-2.52A p95, flat 0/2). Seed-0's standard gate replicates episode-for-episode (det 0/6 all oc incl. both rsi, sto 4/6; its verdict belongs to its own cycle) — this seed lands squarely on the pre-registered FAIL branch: exposure is refuted as the lever. ROOT CAUSE (measured, not conjecture): rise_ref_mesh_scripted.npz spends ticks 0-245 (~4.9s) tucking at exactly h=0.0mm, then presses 0->83mm over ticks 245-400; the state-aligned BC anchor's height-floor pursuit (train.bc_anchor_min_h_ahead_mm=8, sim_env.py ~L4097) requires the target tick to command >=8mm above current height, so from ANY flat/tuck-segment state (h~0) the first qualifying tick is ~258 — the anchor jumps the ENTIRE tuck and supervises flat starts directly toward press-phase legs-under-body poses while the real legs are still splayed radially: the servos press the extended lever arms into the ground = the exact 2.64A pin on video. Non-flat starts match at/past the tuck, which is why only flat fails. The floor is nearly a no-op in the press segment (climbs ~0.5mm/tick, so floor 8mm ~ 0.32s ~ the 0.25s lookahead) — it was built as anti-freeze for the LEGACY ref's 5s 0->25mm crawl, and is segment-blind to a zero-height tuck. NEXT (launching this cycle): tuckfloor0 canary pair — meshref recipe, single lever min_h_ahead_mm 8->0 (the key's own bit-exact default), restoring tuck supervision from flat while leaving press supervision effectively unchanged. Reward quarters 11/3/-49/-307 falling is recipe-shape (start-mix curriculum), not the story; the eval + mechanism are.

