# cw-standwalk-stance-mesh2-standheight-rung5-acq8m

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: ACQUISITION PASS

**created**: 2026-08-26T06:01:59+00:00

**pod**: hexapod-mjx-train-6

**steps**: 8000000

**parent**: cw-standwalk-stance-mesh2-standheight-rung5

**wandb_id**: i78gd6k3

**hypothesis**: Does more budget (8M vs the 2M canary) close the lower-phase softening (seed0's high-herr det drift, 18-20.5mm on 4/6 episodes) without a new bc_anchor-coef mechanism, given both seeds' reward was still rising at the 2M cutoff (Q4 recovering hard out of the shared mid-training valley)? Same recipe, seed 0, warm-started from its own 2M canary checkpoint.

**gate**: PASS: lower/det in the mode_seq_stance+hold_height_cmd seqprobe reaches >=5/6 success (herr<=15mm) with no majority over_current, matching the isolated lower champion's own established band, while hold/rise stay at-or-above the 2M canary's levels (no regression). FAIL at the same signature (4-5/6 det >15mm herr, no term): budget is refuted for this residual; next lever is a height-cmd-segment-specific bc_anchor_coef loosening (new default-off cfg + bank rows + unit tests).

**verdict**: 8M budget closed the exact residual this run was funded for, own-scope (seed 0; joint call pends the -s1 twin, off-limits this cycle). Composed mode_seq_stance+hold_height_cmd seqprobe (the registered gate instrument): lower det 6/6 + sto 6/6 (herr 14.1-14.9mm, all clear the <=15mm bar, ZERO terminations, cur_max 2.2-2.64A non-terminal) -- up from the 2M canary's mixed 2/6 det pass (18-20.5mm on 4/6). hold det 6/6 (0 term) + sto 5/6 (1 hold_low_height) and rise det 5/6 (1 over_current) + sto 5/6 (1 over_current) are at-or-above the 2M canary's own levels (2M: hold sto 5/6, rise det+sto 4/6 each) -- no regression on the other two clauses. Isolated DR-0 gate (mode_seq stripped, full episode budget per mode) is clean and strong: hold 5/6 det (1 hold_low_height term) + 6/6 sto, rise 6/6+6/6 zero-term, lower 6/6+6/6 zero-term (herr 1.1-4.6mm) -- confirms the isolated skills themselves are solid, and the composed lower's 14-15mm residual is a segment-time-budget effect (the mode_seq sampler can hand 'lower' as little as its ~3s min-tail when it lands as the sequence's 3rd segment), not a skill regression. own-DR(0.2) comparable (hold 5/6+6/6, rise 6/6 det+5/6 sto [1 over_current], lower 6/6+6/6). Video (hold/rise/lower isolated strips, det+sto) confirms upright six-foot-planted stands throughout, no fall/tip/collapse; session harness HARD GATES PASS (no_falls/rise/sit_descends). Reward rose every quarter (-14.1/204.4/455.3/860.9). Residual: cur_max still kisses 2.4-2.65A on most rise/lower episodes without tripping -- the same structural, non-terminal ceiling every solved rise/lower arm in this campaign rides; a thin 1-episode over_current tail persists in own-DR rise (matches the catalogued bridge/rsi deep-start tail). BUG FOUND+FIXED THIS CYCLE (see CURRENT_TRUTHS/pod_eval.py): the automated gate/owncfg/session eval pipeline was silently running every 100Hz run's pass under the LEGACY 25Hz slew contract (max_delta_q_deg=1.5, 4x looser than the actual 0.375 trained contract) whenever the launch command (correctly) relied on config.yaml's own 100Hz default instead of restating the key explicitly -- this run's gate/owncfg/session were killed and RE-RUN under the fix (also relaunched the manual seqprobe under the fix); all numbers in this verdict are POST-FIX and trustworthy. The -s1 twin's own gate/owncfg/seqprobe were started ~10 min BEFORE the fix landed -- whoever closes the joint call should confirm -s1's reads are post-fix (re-run if not) before trusting a disagreement/agreement read.

