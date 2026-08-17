# cw-dynrep-livewalkrise1-canary2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-17T21:48:18+00:00

**pod**: hexapod-mjx-train-5

**steps**: 60000

**git_sha**: 3271920ca1e15b8d2e1f5e07ed057c916b0d2d5e

**wandb_id**: 2auqhfmy

**hardware_ready**: False

**hypothesis**: Verify the exogenous-command mask fixes the live predictor divergence canary1 found: with priv channels 7:14 masked on live windows, the online predictor trained on command-rich live data should keep its corpus heldout near the pretrained 2.286 reference instead of tripling, and live-val losses should sit at physical-head scale. Mechanism re-canary before the 10M continuation (operator order fb_20260817T210422_9df9c7 arm A).

**gate**: MECHANISM-HEALTH CANARY ONLY: canary1 gates still hold (actor KL 0, quotas, boundary-only attempts with logged gate values) AND the fix is effective: pred/gate/corpus_val_candidate at the boundary attempts stays < 2x the 2.286 pretrained ref (vs 7.3-9.5 in canary1) and live_val walk/rise land at physical scale (<50, vs ~1900-2500). PASS -> launch cw-dynrep-livewalkrise1 (10M, boundary 1M) immediately.

**verdict**: CANARY PASS (mechanism): the exogenous-command mask fix works — online predictor corpus-val 2.496/2.529 at the two 30k boundary attempts (inside the 15% band of the 2.286 pretrained ref; was 7.3-9.5 in canary1), live_val at physical scale (walk 3.8-4.5, rise 3.1-3.4; was ~1900-2500), candidate BEAT the snapshot on live walk at both attempts and held rise retention, latent drift 0.02-0.03, value-jump 0.014/0.159 — both attempts ACCEPTED, snapshot_version 0->2 (accept path now exercised; canary1 exercised reject). actor KL exactly 0 on all 30 iterations. Continuation cw-dynrep-livewalkrise1 (10M, boundary 1M) launched per the pre-registered gate.

**note**: Script-owned re-canary after the canary1 exogenous-cmd-priv finding (fix 3271920c: live windows mask priv 7:14). 60k steps, boundary 30k. Same mechanism scope as canary1.

