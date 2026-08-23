# cw-amp-m3-pushcur2-noamp-n2040-c2r2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INFORMATIVE

**created**: 2026-08-23T01:56:14+00:00

**pod**: hexapod-mjx-train-0

**steps**: 6000000

**parent**: cw-amp-m3-pushcur2-noamp-n2040-c2

**wandb_id**: 8534ow0e

**hypothesis**: Plain English: the staged 20-40N walker missed its bar by ONE stochastic episode while still visibly improving — does 6M more at the same dose finish the job? Continuation of pushcur2-noamp-n2040 (its own pre-registered INFORMATIVE-ceiling branch + 08-21 ruling: terms fell monotonically 136->96/window, reward rose -47/52/72/83, no plateau). Prediction-if-true: terms keep falling and the 20-40N own-cfg gate reaches topples <=1/6 det AND <=2/6 sto (from 1/6+3/6) — force axis closes at 40N via 3-stage curriculum, 18M total. Prediction-if-false: terms flatten in the 90s band and topples stay ~4/12 — the slow decline was tail-chasing, staging saturates ~30N and the recovery-specific mechanism (get-up reward / longer episodes) becomes the named M3 lever. Strongest alternative: pushhard1-noamp-n2040-c1r1 (flat-jump control, 12M) lands equal or better — then budget, not curriculum, drives 20-40N and the staging story dies. (Infra retry: verbatim relaunch of a REFUSED stale-pod-code-marker stub via respec --now sync path; no spec change.)

**gate**: Hardening continuation (6M more at 20-40N, warm from n2040 stage-2 ckpt, 18M total). PASS = own-cfg gate topples <=1/6 det AND <=2/6 sto, gait_valid >=5/6 det+sto, zero sacrificed, det prog med >=0.9, genuine recovery on video => force axis CLOSED at 40N. INFORMATIVE-plateau = topples ~4/12 with terms flat over last 2M => staging saturates below 40N; recovery mechanism is the named lever. FAIL = collapse/statue/NaN.

**verdict**: Six more millions of staged 20-40N training bought nothing: the force axis SATURATES below 40N without a recovery mechanism — this is the run's own pre-registered INFORMATIVE-plateau branch. Evidence: own-cfg gate (pushes active, dr0) 3/12 topples at 18M total vs the 12M parent's 4/12 — inside n=6-per-mode eval noise, no evidence of change; the det/sto split flip (det 3/6 misses its <=1/6 bar, sto 0/6 passes) is the same noise. The deciding scalar: tilt terminations re-climbed to ~145/window on warm-start then declined only to ~102 and sat FLAT over the final 2M+ (107/102/102 per 1M bucket) — never even recovering the parent checkpoint's 96/window exit level — while reward rose every quarter (-67->96, warm-start recovery shape). Reward rising + gate flat at matched dose = mechanism-limited, not undertrained (08-21 ruling + 08-23 CURRENT_TRUTHS reading); joint with pushhard1-noamp-n2040-c1r1's raw-budget plateau (3/12, terms flat) at 12M, BOTH curriculum and budget are now refuted at this dose. Strips watched: clean six-leg gait, genuine full-topple knockdowns (det_3 flips onto its back), sto episodes ride shoves back to upright, zero crouch/statue/sacrificed. Interpretation: 20-40N shoves include physically-unrecoverable-in-stride knockdowns; without a get-up-after-knockdown mechanism (tilt-term relaxation + get-up reward + longer episodes) the policy can only reduce, never eliminate, topples. Named M3 lever stands: recovery-specific mechanism — but it is a reward/env change gated on semantics-bank work, AND the M3 brief bar (repeated pushes, recovery without reset, tracking preserved, no crouch) is already MET at the 10-25N x3 dose (repeat3 PASS + 6x density probe). Judgment: record the 20-40N force axis as ceiling-reached-at-3/12, deprioritize the get-up build vs the M5 composition critical path (question logged). No further same-recipe continuations at this dose. repeat3 remains the M3 champion recipe.

