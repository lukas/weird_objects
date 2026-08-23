# cw-amp-m3-pushcur1-noamp-b1530

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-23T00:51:19+00:00

**pod**: hexapod-mjx-train-1

**steps**: 6000000

**parent**: cw-amp-m3-pushacq1-noamp

**wandb_id**: ynpvsudc

**hypothesis**: Plain English: a shove-force CURRICULUM should reach 20-40N robustness where the flat jump plateaued — this is stage 1, an intermediate 15-30N bridge dose trained from the clean 10-25N champion (pushacq1-noamp, 0/12 topples), with a later cycle chaining stage 2 (20-40N) from this run's checkpoint. Same recipe as pushhard1-noamp-n2040 except dr.ext_push_n=15,30 and everything else identical. Prediction-if-true: training tilt-terms fall well below n2040's plateau (pitch settled ~53/window there; expect <25) and the DR-0 own-cfg gate at 15-30N holds topples <=1/6 det and <=2/6 sto with gait_valid >=5/6 and det prog med >=0.9, making the 20-40N stage-2 chain the next rung. Prediction-if-false: even 15-30N plateaus above the bar — the recoverable-force ceiling is closer to 25N than 40N and M3 needs a recovery-specific mechanism (e.g. get-up/recovery reward or longer episodes), not just dose staging. Strongest alternative: the matched continuation arm (n2040-c1, launched same cycle) closes the gap with raw budget alone, meaning curriculum staging is unnecessary.

**gate**: Hardening stage 1 (6M, DR-0, 15-30N single shove, from pushacq1-noamp ckpt). PASS = DR-0 own-cfg gate gait_valid >=5/6 det+sto, zero sacrificed legs, det prog med >=0.9, topples <=1/6 det AND <=2/6 sto at 15-30N, video shows genuine recovery; on PASS the pre-registered follow-up is stage 2: respec at 20-40N from THIS checkpoint, judged against n2040-c1 at matched total budget. INFORMATIVE-ceiling = topples above bar with terms still falling at cutoff => continue per 08-21 ruling. INFORMATIVE-plateau = topples above bar AND terms flat => force ceiling is <30N, names a recovery-specific mechanism (not dose staging) for M3. FAIL = collapse/statue/NaN.

**verdict**: Stage-1 force bridge works: 15-30N single-shove trained from pushacq1-noamp lands 1/6 det + 1/6 sto topples (bar <=1/6 det, <=2/6 sto), gait_valid 12/12, zero sacrificed legs, det prog med 1.20. Training tilt terminations fell 26->14/window — under the pre-registered <25 prediction and far below n2040's ~53 flat-jump plateau. Strips watched: genuine shove absorption (sto_1 rides a 22deg roll peak back to upright six-leg cycling; no crouch/statue). Why: the bridged dose keeps the recovery gradient alive where the flat 20-40N jump starved it. Next: pre-registered stage 2 — 20-40N respec from THIS checkpoint (pushcur2-noamp-n2040), judged against n2040-c1 at matched total budget; NOTE the n2040-c1 budget control never trained (REFUSED x3 on stale pod code marker), relaunched this cycle.

