# cw-amp-m3-pushcur2-noamp-n2040

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INFORMATIVE

**created**: 2026-08-23T01:21:00+00:00

**pod**: hexapod-mjx-train-0

**steps**: 6000000

**parent**: cw-amp-m3-pushcur1-noamp-b1530

**wandb_id**: dihecyxp

**hypothesis**: Plain English: the walker that survived the 15-30N bridge now trains at the full 20-40N target — does staging the force in two rungs reach the robustness the flat jump could not? Stage 2 of the pre-registered force curriculum: identical recipe to pushcur1-noamp-b1530 except dr.ext_push_n=20,40, warm from b1530's own checkpoint (which passed stage 1 at 2/12 topples, terms 26->14). Judged against pushhard1-noamp-n2040-c1 at matched 12M total budget (control relaunched this cycle after 3x REFUSED on stale pod code). Prediction-if-true: tilt terms start elevated then FALL well below n2040's ~53/window plateau (expect <30 at end) and the 20-40N own-cfg gate holds topples <=1/6 det + <=2/6 sto vs n2040's 4/12 — curriculum beats flat jump AND beats matched raw budget. Prediction-if-false: terms flatten and topples sit at the ~4/12 floor — staging cannot cross ~30N either, and joint with style05-r3b1530-r1's count-x-force plateau the M3 lever becomes a recovery-specific mechanism (get-up reward / longer episodes), not dosing. Strongest alternative: n2040-c1 matches this arm — then budget, not curriculum, was the missing ingredient.

**gate**: Hardening stage 2 (6M, DR-0, 20-40N single shove, warm from pushcur1-noamp-b1530). PASS = own-cfg gate gait_valid >=5/6 det+sto, zero sacrificed legs, det prog med >=0.9, topples <=1/6 det AND <=2/6 sto at 20-40N, video shows genuine recovery; on PASS the force axis of M3 is closed at 40N and the curriculum-vs-budget fork is judged vs n2040-c1 at 12M total. INFORMATIVE-ceiling = topples above bar with terms still falling at cutoff => continue per 08-21 ruling. INFORMATIVE-plateau = topples ~4/12 with terms flat => staging cannot cross ~30N; recovery mechanism becomes the named M3 lever. FAIL = collapse/statue/NaN.

**verdict**: Staged force at 20-40N gets the walker CLOSE but not under the bar at 12M total: 4/12 topples (det 1/6 PASSES its half, sto 3/6 misses the <=2/6 bar by ONE episode), gait_valid 12/12, zero sacrificed legs, det prog med 1.13. This is the pre-registered INFORMATIVE-ceiling branch, not the plateau: tilt terms fell MONOTONICALLY all run (pitch+roll 136->96/window by 1M-bucket, still declining at cutoff) and reward rose every quarter (-47/52/72/83) — unlike the flat-jump parent that sat flat. Strips watched: clean six-leg cycling, genuine 20-40N absorption (sto_5 rides shoves back to upright), knockdowns are real full topples (det_2 roll, sto_1 pitch), no crouch/statue. Caveat vs its own prediction-if-true: terms did NOT reach <30 — decline is slow (~4-5/window per 1M), so the curriculum keeps the gradient alive at 20-40N but has not converged. Per the branch + 08-21 ruling: 6M continuation launched from this ckpt (pushcur2-noamp-n2040-c2). The 12M curriculum-vs-budget fork vs pushhard1-noamp-n2040-c1r1 stays judged against THESE frozen 12M numbers (4/12, terms 96 falling).

