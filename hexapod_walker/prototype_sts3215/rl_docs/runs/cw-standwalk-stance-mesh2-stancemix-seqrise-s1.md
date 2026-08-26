# cw-standwalk-stance-mesh2-stancemix-seqrise-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: CANARY FAIL - MECHANISM

**created**: 2026-08-26T02:17:11+00:00

**pod**: hexapod-mjx-train-5

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stancemix-tuckclock-stdreopen-s1

**wandb_id**: 6x4ivpd1

**hypothesis**: Seed-1 twin of cw-standwalk-stance-mesh2-stancemix-seqrise: does the flat-rise/mix interference close if the 3-way hold+rise+lower mix warm-starts from the ALREADY-SOLVED flat-rise checkpoint (riseonly-bcchain3-meshref-tuckclock-acq8m-s1, 12/12 flat valid_plant on seed 1) instead of stancemix_bcchain3_stdanneal? Same recipe/hypothesis as the seed-0 arm, seed 1 lineage throughout (source config + warm-start ckpt) to answer the joint 2-seed pass-rate question in one batch.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. CANARY (2M, 2-seed pair with seqrise seed0): PASS if flat-pinned probe shows genuine non-freeze tuck (majority NOT pinned at 2.64A, valid_plant clearly > the stdreopen-acq8m-s1 floor) AND hold/lower det+sto >=5/6+5/6 zero-term in both seeds -> fund 8M acquisition pair. FAIL if flat probe pin unchanged/still-majority-2.64A in both seeds -> interference is reward-mix pricing, not initialization; next lever is per-mode reward re-pricing or a staged/frozen-rise curriculum.

**verdict**: CANARY FAIL - MECHANISM (own scope): the SEQUENCING lever (warm-start the 3-way hold+rise+lower mix from the already-solved riseonly-tuckclock-acq8m-s1 flat-rise checkpoint instead of stancemix_bcchain3_stdanneal) does NOT deliver working flat-start rise at 2M on this seed, but the failure mode is a NEW, qualitatively different pathology, not the previously-refuted over-current pin. Evidence (own scope, DR-0 gate + own-DR(0.2) + dedicated flat-pinned probe det+sto n=6+6, all read from synced report.json + contact sheets on train-5): flat probe 0/12 valid_plant (0/6 det, 0/6 sto), but NOT current-pinned (cur_max 2.04-2.62A, well under the 2.64A trip, zero terminations) and NOT frozen (uniform per-leg duty 0.67-0.78, real swing counts on every leg, esp. leg idx1 9-26 swings) -- video/contact-sheets (det+sto) show a genuine splay->partial-tuck that SETTLES INTO A STABLE HALF-RISE PLATEAU (height_err_end pinned at 30-34mm, ~triple the mesh rise-solved lineage's <5mm) and never completes the final push to the 79-87mm target for the full 15s episode. This is neither the stdreopen family's over_current/press-up pin nor tuckexempt's snap-fold freeze -- a third residual pathology ('stalls at half-mast'). Non-flat/mixed-kind DR-0 gate stays close to the meshref parent's own band (hold 6/6+6/6 zero-term, rise det 5/6 + sto 4/6 valid_plant zero-term, lower det+sto 0 terms); own-DR(0.2) hold 6/6+6/6 zero-term, lower 0 terms both, rise 3/6 det (1 OC term) + 2/6 sto (1 OC term) -- the compound gate's hold/lower clause (>=5/6+5/6 zero-term) is CLEARLY MET on this seed alone. Per the canary's own registered text, PASS needs valid_plant 'clearly above' a comparator floor (either reading, 2/12 or 11/12) -- 0/12 clears neither, so this does not qualify as PASS despite genuinely refuting the literal FAIL trigger too ('still-majority-pinned', which is false here). WHY: the checkpoint we warm-started from evidently gets partially unlearned/diluted once retrained jointly with hold+lower at only 2M steps -- it retains enough to avoid the OC-pin basin entirely (unlike stdreopen) but not enough to finish the rise. NEXT: this run's own read is FAIL-scope; the joint SEQUENCING call needs the sibling seqrise (seed 0, owned by a concurrent cycle) and stdreopen-s2 before deciding fund-8M vs close-the-lever -- not decided here. If the sibling also plateaus rather than pins, the honest joint read is 'sequencing swaps one residual (OC-pin) for another (stalled plateau), net motion in the right direction but not solved' -- likely worth 8M since the plateau is NOT current-limited (unlike the pin), so budget has actual room to work with.

