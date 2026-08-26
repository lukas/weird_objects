# cw-standwalk-stance-mesh2-stancemix-tuckclock-scratch8m-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: ACQUISITION PASS

**created**: 2026-08-26T03:29:14+00:00

**pod**: hexapod-mjx-train-7

**steps**: 8000000

**wandb_id**: nl1im163

**hypothesis**: Seed twin of cw-standwalk-stance-mesh2-stancemix-tuckclock-scratch8m (seed 0, VERIFIED RUNNING train-6): does training the proven flat-rise recipe FROM SCRATCH (real std 1.0->0.018 anneal, not a silently-pinned warm start -- see the log-std-init-under-init-from bug discovered this cycle) inside the full hold=0.1/rise=0.45/lower=0.45 mix solve all three stance skills in ONE policy, and does it replicate cross-seed? Same recipe, seed 1 only.

**gate**: ACQUISITION (8M, judged jointly with the seed-0 twin): PASS if flat-pinned probe (goal.rise_flat_frac=1.0/partial=0/rsi=0, det+sto 6+6, DR-0) >=10/12 valid_plant per seed with no majority 2.64A OC-pin, AND hold det+sto >=5/6+5/6 zero-term, AND lower >=5/6 honest (<=10mm herr) per seed -> from-scratch is THE mesh stancemix recipe; promote the better seed's checkpoint and move to STAND_HEIGHT rungs / walk distill. PARTIAL if one seed passes all clauses and the other shows a trough-but-rising trajectory at 8M -> continue the lagging seed per the 08-21 ruling before any recipe verdict. FAIL if both seeds are budget-invariant on the flat clause vs their own 2M mark or hold/lower never converge -> mix-diet interference is structural even under full exploration; next lever is staged/frozen-rise curriculum or per-mode gradient isolation, NOT more seeds/budget, and the track weighs promoting stdreopen-acq8m-s1's checkpoint as stage-1 output.

**verdict**: Seed-1 of the from-scratch (real std 1.0->0.018 anneal) full hold=.1/rise=.45/lower=.45 mix clears every registered clause at 8M, and the seed-0 twin (concurrent cycle, ACQUISITION PASS) independently cleared all three too -- JOINT CALL: JOINT PASS, both seeds 2/2. Flat-pinned probe (rise_flat_frac=1.0/partial=0/rsi=0, det+sto 6+6, DR-0): 11/12 valid_plant (det 6/6, sto 5/6 w/ 1 over_current fall), herr_end 0.2-8.6mm, real per-leg swing counts on most episodes (e.g. [1,1,0,2,1,0]), video/strip-consistent splay->tuck->level-plant -- not a majority OC-pin (only 1/12 terminated). DR-0 gate: hold 6/6+6/6 zero-term (herr<=5.5mm); lower 6/6 det + 6/6 sto all success, herr 3.0-5.0mm (well under the 10mm bar; 'valid_plant' reads 0/6 here because that flag means rise-height plant, not lower's own success criterion -- success=true every lower episode both DR settings). Own-DR(0.2): hold 6/6+6/6 (zero term); lower 6/6+6/6; rise 5/6 det (1 OC term) + 6/6 sto. Reward rose every quarter matching the isolated tuckclock-acq8m trough-then-breakout arc. Seed-0 (posted, concurrent cycle's own verdict) is marginally cleaner (12/12 flat, zero own-DR rise terms) -- promoting seed-0's checkpoint (ppo_goal_cw_standwalk_stance_mesh2_stancemix_tuckclock_scratch8m.zip) as THE mesh stancemix recipe per the registered PASS branch. This closes the from-scratch-vs-warm-started/std-reopen saga: pinned-std warm-starts (stdreopen family, seqrise) are refuted, the real std anneal from scratch is the reliable (2/2) mesh stancemix recipe. Residual (both seeds, matches every prior 'solved' rise arm in this campaign): cur_max still kisses 2.4-2.64A on most rise episodes without tripping (structural, non-terminal); a thin over_current/fall tail remains on the hardest deep starts (bridge/rsi/flat-sto) -- ~3/84 read episodes per seed across all three reports -- so this is Stage-1's cleanest result yet, not a zero-fall closure. Next: unblocks STAND_HEIGHT rung 5 (rise->hold(height-cmd)->lower composition) and stage-2 walk distillation per the track's registered PASS route.

