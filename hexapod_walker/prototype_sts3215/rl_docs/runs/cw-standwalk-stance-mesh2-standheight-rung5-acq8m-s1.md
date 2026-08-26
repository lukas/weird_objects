# cw-standwalk-stance-mesh2-standheight-rung5-acq8m-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-26T06:06:10+00:00

**pod**: hexapod-mjx-train-0

**steps**: 8000000

**parent**: cw-standwalk-stance-mesh2-standheight-rung5-s1

**wandb_id**: auf0f70c

**hypothesis**: Seed-1 twin of the rung5-acq8m continuation: does more budget close seed1's lower-phase residual (lower/det trips over_current on 6/6 seqprobe episodes despite tight herr tracking) without a new bc_anchor-coef mechanism? Same recipe, seed 1, warm-started from its own 2M canary checkpoint.

**gate**: PASS: lower/det in the mode_seq_stance+hold_height_cmd seqprobe reaches >=5/6 success with no majority over_current, while hold/rise stay at-or-above the 2M canary's levels. PARTIAL/FAIL judged jointly with the seed-0 acq8m twin per the same disagreement rule as the 2M pair (both must clear for a clean PASS; disagreement -> next lever is the height-cmd-segment bc_anchor-coef loosening, not a 3rd seed).

**verdict**: ACQUISITION PASS (own scope) -- 8M continuation fixes seed-1's specific 2M residual (lower/det over_current-pin) with zero new cost elsewhere. Evidence: registered composed seqprobe (--cfg-set goal.mode_seq_stance=1 mode_seq_hold_height_cmd=1 hold_height_cmd_frac=1, det+sto 6+6, launched this cycle since the watcher only pre-stages the isolated gate/owncfg): lower/det now 6/6 success with ZERO over_current terms (herr 9.9-10.2mm) -- was 0/6, ALL 6/6 over_current-pinned at the 2M canary; lower/sto stays clean 6/6 (herr 9.5-10.1mm, matches 2M). hold improves 5/6det+5/6sto(2M, 1 hold_min_load term each) -> 6/6+6/6 zero-term (herr 0.4-2.0mm). rise is a wash in aggregate (9/12 both budgets) but the FAILURE MODE changed: 2M's 3 terms were hold_min_load (leg-unload softening), 8M's 3 terms are hold_low_height on flat starts only (bridge/crouch/rsi all clean 4/4+4/4) -- the campaign's already-catalogued deep-start residual, not a new pathology, and not over_current. Isolated DR-0 gate/own-DR(0.2) probes (pre-staged) stay clean/near-clean (DR-0: 35/36 zero-term, 1 rise/sto OC; own-DR: 33/36, 1 term each on hold/rise/lower). Video (lower_det_0, contact sheet, both watched): genuine progressive crouch, upright, six feet grounded throughout, no fall/tip. Reward rose every quarter (-4.6/171.8/390.8/812.0), matching the campaign's now-familiar trough-then-breakout arc. This clears the registered PASS branch's own text (lower/det >=5/6 with no majority over_current; hold/rise at-or-above the 2M canary) on THIS seed alone. JOINT RUNG-5 CALL NOT DECIDED HERE -- the seed-0 twin (-acq8m, no -s1 suffix) is owned by a concurrent cycle this round per the prompt's explicit scoping; if it also clears (lower fixed, hold/rise no worse), promote this checkpoint's height-cmd+mode_seq composition and treat rung-5 CLOSED, opening stage-2 (rise->walk->lower) composition design. If seed-0 instead still shows its own herr-drift softening unresolved, the next lever is the height-cmd-segment bc_anchor_coef loosening the gate named, not a 3rd seed. Evidence: logs/ckpt_eval/cw_standwalk_stance_mesh2_standheight_rung5_acq8m_s1_{gate,owncfg,seqprobe}/, W&B auf0f70c.

