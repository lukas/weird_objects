# smoke-rnd-wiring-check

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-23T22:18:04+00:00

**pod**: hexapod-mjx-train-0

**steps**: 100000

**hypothesis**: Smoke: does the new RNDVecWrapper wiring run end-to-end on the real warp/mjx GPU stack without crashing (unit tests already cover the wrapper math in isolation on CPU stubs; this is the integration check --use-sde's landing skipped and later needed).

**gate**: No traceback, W&B run reaches step>=90000, rnd/intrinsic_mean and rnd/loss appear in the log.

**refused_reason**: hexapod-mjx-train-0 code marker 75863b8119a452cd35b4fde2447e427e99a3b6b0 != local HEAD e180b1611eb3905b5dc0a20e794eae55ef2f89e1 and the delta is not benign-orchestrator-only. Sync first: snapshot.sh --sync hexapod-mjx-train-0 (and snapshot/commit before that if the tree is dirty).

