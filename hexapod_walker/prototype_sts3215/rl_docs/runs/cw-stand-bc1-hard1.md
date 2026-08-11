# cw-stand-bc1-hard1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: DONE

**created**: 2026-08-11T04:37:30+00:00

**pod**: hexapod-mjx-train-6

**steps**: 10000000

**parent**: cw-stand-bc1

**wandb_id**: 58sekzjq

**hardware_ready**: False

**hypothesis**: The BC-anchor mechanism that fixed the flag-leg cheat in discovery (cw-stand-bc1) is trained for real: does the SAME recipe at a hardening budget (10M vs 2M) consolidate the honest six-foot plant (raise flat-start valid_plant off its current 0/10 footprint-only miss) and recover the hold/track precision and raise/tipped-recovery probes that looked worse in cw-stand-bc1's own training diagnostic (n=2 samples, weak evidence) - or does more training re-drift back toward the cheat now that the anchor's pull is diluted over more updates? Continuation of the SAME arm (init-from-source), one variable (steps), per the phase system's hardening rule (already visibly works, now prove it holds and improves with budget).

**gate**: harness at 10M (all of hold/track/rise/raise/lower/tipped, det+sto, DR0 + own-DR): rise valid_plant >=8/18 det (44%, vs today's 13/30=43% RSI-off / 3/6=50% RSI-on) held or improved on ALL start kinds including >=2/10 flat (today 0/10); hold/track_err_mean_deg back under 2.0 deg (today 3.0); raise and tipped_recovery each >=1/2 in the training periodic-eval at every checkpoint from 4M on (today's parent-vs-child regression, if real, must resolve). Kill early if rise_feet_factor collapses back under 0.4 for 2 consecutive periodic-eval windows (the pre-anchor collapse signature) - would mean the anchor's fix doesn't survive a longer horizon.

**verdict**: PASS on the hardening question; gate-as-written PARTIAL; the escalation's 'hardening regression' is REFUTED by a matched-parent control. OBSERVATIONS: rise consolidated decisively — gate det valid_plant 5/6 (parent 3/6; the one miss parks 22mm short, honest), RSI-off probe 12/12 across bridge/crouch/FLAT with worst foot 7mm (parent 13/30, flat 0/10 — the flat-start footprint miss is RESOLVED); env/rise_feet_factor held 0.69-0.82 for all 10M (the pre-anchor collapse signature never appeared, kill rule never fired); bc_anchor_loss 0.01-0.03. Matched-parent probe run this dig-in (same seed 7/cfg/RSI-off, per-mode 12, parent ckpt logs/ckpt_eval/cw_stand_bc1_parentctl_norsi on pod train-6): parent ALREADY fails hold 0/12 (worst foot 51mm), track 0/12 (65mm), raise 0/12 (40mm), lower 0/12 with a 166mm FLAG-LEG — vs child hold 0/12 (162mm), track 0/12 (184mm), raise 0/12 (81mm), lower 0/12 (189mm). Nothing the parent could do was lost. INTERPRETATION: 10M improves what this stack prices (hold/track err 3.0->1.5deg det, meets the <2.0 gate line; lower reaches belly height; tipped diag 0->0.5-1.0 from 5M) but converges hold to a splayed front-legs-up crouch at 2.6A and keeps lower's pre-existing flag-leg — a hold/track/lower stillness+feet-down pricing gap in this rise-rebuilt stack, pre-existing and mildly amplified, NOT anchor re-drift. The raise gate criterion was ill-posed: p_raise=0 in the goal mix, so the mode is untrainable in this arm (0.0 all 10M). VERDICT: keep checkpoint as the RISE SPECIALIST (12/12 honest cold-start stands incl. the operator-placement flat start); hardware-ready NO as a unified policy (hold splays+over-current, lower flag-legs). STOP hardening this lineage; next is SPECIFICATION (hold-mode stillness bank/pricing) + the composition test rise-specialist -> walk-champion handoff. Also fixed this cycle: eval harness now refuses unknown --modes loudly (the 'tipped' NaN-crash that killed the triage probe's report).

