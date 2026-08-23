# cw-amp-m3-pushcur2-noamp-n2040-c2r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-23T01:54:31+00:00

**pod**: hexapod-mjx-train-1

**steps**: 6000000

**parent**: cw-amp-m3-pushcur2-noamp-n2040-c2

**hypothesis**: Plain English: the staged 20-40N walker missed its bar by ONE stochastic episode while still visibly improving — does 6M more at the same dose finish the job? Continuation of pushcur2-noamp-n2040 (its own pre-registered INFORMATIVE-ceiling branch + 08-21 ruling: terms fell monotonically 136->96/window, reward rose -47/52/72/83, no plateau). Prediction-if-true: terms keep falling and the 20-40N own-cfg gate reaches topples <=1/6 det AND <=2/6 sto (from 1/6+3/6) — force axis closes at 40N via 3-stage curriculum, 18M total. Prediction-if-false: terms flatten in the 90s band and topples stay ~4/12 — the slow decline was tail-chasing, staging saturates ~30N and the recovery-specific mechanism (get-up reward / longer episodes) becomes the named M3 lever. Strongest alternative: pushhard1-noamp-n2040-c1r1 (flat-jump control, 12M) lands equal or better — then budget, not curriculum, drives 20-40N and the staging story dies. (Infra retry: verbatim relaunch of a REFUSED stale-pod-code-marker stub via respec --now sync path; no spec change.)

**gate**: Hardening continuation (6M more at 20-40N, warm from n2040 stage-2 ckpt, 18M total). PASS = own-cfg gate topples <=1/6 det AND <=2/6 sto, gait_valid >=5/6 det+sto, zero sacrificed, det prog med >=0.9, genuine recovery on video => force axis CLOSED at 40N. INFORMATIVE-plateau = topples ~4/12 with terms flat over last 2M => staging saturates below 40N; recovery mechanism is the named lever. FAIL = collapse/statue/NaN.

**refused_reason**: hexapod-mjx-train-1 code marker 331a459014c31c4796e32cb27ff66067468c8749 != local HEAD 8bb6ade3192610acc1e9152bcdd11575f16a75d4. Sync first: snapshot.sh --sync hexapod-mjx-train-1 (and snapshot/commit before that if the tree is dirty).

