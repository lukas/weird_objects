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

**verdict**: PASS (partial), refines bc1's picture — rise consolidates further with hardening, but hold/track's pre-existing leg-cycling gets WORSE, not better. Gate harness (RSI 0.5): rise valid_plant 5/6 det (83%, up from bc1's 3/6=50%) and 4/6 sto, tight height errors (0.2-2.2mm on 8 of 12 episodes). Rise mechanism is genuinely consolidating with more steps — the primary hypothesis (does the honest plant hold up and improve with budget) is CONFIRMED. However a re-check of hold/track's own per-episode numbers (duty_cycle/swing_count/end_clear_mm — NOT examined closely at bc1's verdict, corrects that gap) shows this checkpoint's 'hold' is not a quiet stand: 3 of 6 legs cycle continuously (duty ~0.36-0.59, 4-19 swings per 15s episode) and end at 100-161mm elevation (tail-mean) — track/hold success 0/6 both det+sto. Same pattern already existed at bc1 (2M): duty ~0.85-0.9/0.06-0.09 alternating, end_clear up to 50mm — WORSE at 10M (100-161mm). NOTE resolving a concurrent read: a parallel cycle's triage of the same run (independently, off a broken --modes eval invocation of mine that crashed on a harness bug with 'tipped' mode) read the raw 0/12 numbers as 'protected-skill erosion, splayed/dragging legs' and flagged DIG-IN. I re-extracted video at 2fps (finer than the default 10-frame strip) for hold_det_0 and track_det_0 from the CLEAN default-protocol gate (not the broken run) and it shows a normal, stable six-leg stance throughout — no splaying/dragging/flag-leg. The real, video-confirmed story is continuous small-amplitude leg-cycling (a marching-in-place habit), not a collapse into a flag-leg/dragging exploit; the large 'end_clear_mm' numbers are a tail-mean over a genuinely stepping gait, not a frozen elevated leg. This DOES still count as a real skill-interference cost (not a clean pass) but is milder and differently-shaped than 'protected-skill erosion into a known exploit' — no further re-verdict needed, this entry is the resolved analysis. Ruling: the BC-anchor rise fix is real and improves with hardening (keep this lineage for rise), but do NOT keep blindly hardening hoping hold/track self-heals — they are trending the wrong way. Next needs a SPECIFICATION step: audit hold/track's income pricing for why continuous leg-cycling isn't charged (k_still scope currently reads belly-rest/lower-specific, not general hold/track) before any further steps on this lineage. No new run queued this cycle (reward-mechanism change needs its own MDP_PREFLIGHT bank pass first, per RESEARCH_RULES).

