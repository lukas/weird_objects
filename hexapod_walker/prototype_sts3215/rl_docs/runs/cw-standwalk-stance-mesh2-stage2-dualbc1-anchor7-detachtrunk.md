# cw-standwalk-stance-mesh2-stage2-dualbc1-anchor7-detachtrunk

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-27T09:54:45+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor6b-logstdsplit-fix

**wandb_id**: hz8mdkn4

**hypothesis**: Plain sentence: stop the anchor (BC) loss's gradient from touching the shared GRU trunk -- only the actor head trains on it -- to test whether trunk-gradient bleed (not action-noise starvation) is what freezes walk in the anchor4/5/6/6b-class collapse. This is the SAME lever (train.bc_anchor_detach_trunk) that cw-arch-gru-anchor2 already validated on the joystick track's dual-core GRU: turning off the walk anchor there did not save walk, proving the interference came from OTHER modes' anchor pairs bleeding into the shared trunk, not the walk term itself. Prediction-if-true: on top of the identical anchor6b-logstdsplit-fix recipe (log_std split, wiring-confirmed working, seed1 catastrophically froze anyway), detach_trunk=1 rescues seed1's walk (gait_valid >=5/6, no 3+-leg sacrifice) while hold still benefits from the log_std anneal. Prediction-if-false: seed1 (or now seed0 too) still collapses even with the trunk fully shielded from the anchor gradient -- that would point at the critic or the PPO policy-gradient/anchor-loss interaction itself, not any gradient-sharing path.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition or require mature gait at this checkpoint. WIRING CHECK FIRST: train log must show detach_trunk active (no_grad extract_features/GRU path) -- diff the anchor loss's grad-norm on trunk params vs the fix/fix-s1 baseline (should be ~0 on trunk, nonzero on action_net only). Then joint 2-seed call vs the anchor6b-logstdsplit-fix{,-s1} baseline: FULL PASS = WALK-SURVIVES on BOTH seeds (det gait_valid >=5/6, no 3+-leg-sacrifice freeze, prog_ratio >=~0.2) where fix-s1 alone failed -- this promotes detach_trunk into the standing dual-anchor recipe and confirms trunk-bleed as root cause. PARTIAL if it rescues one seed but not the other (matches this arm's own -s1 twin) -- treat as still seed-sensitive, dig deeper into critic. FAIL if walk still shows the anchor4-class catastrophe on either seed with wiring confirmed -- refutes trunk-bleed too, escalate to critic-level dig-in.

**verdict**: CANARY FAIL - MECHANISM: detaching the stance-anchor loss's gradient from the shared GRU trunk does NOT reliably fix the anchor4-class walk collapse -- it made THIS seed's walk worse. Evidence (report.json, DR-0 gate + own-DR owncfg, vs the exact anchor6b-logstdsplit-fix parent this respec's from): fix (seed0, no detach) walked CLEAN at DR-0 (gait_valid 6/6, sacrificed_legs=[] every episode, prog_ratio 0.22-0.24, fwd 0.52-0.55m). detach_trunk (this run) REGRESSES that same seed to gait_valid 0/6 with a consistent 2-leg persistent drag (sacrificed_legs [3,4] every det episode, one episode [2,3,4]), prog_ratio down to 0.13-0.14, fwd 0.30-0.33m; own-DR(0.5) stays similarly degraded (sac [2,3](,4), prog 0.10-0.17). This is milder than the anchor4/5/6-class TOTAL freeze (no 3-5-leg static catastrophe, real continuous forward translation on the contact sheet) but is still a clear regression from a previously-clean seed, not a neutral or improving change. Hold/rise/lower are roughly unchanged vs fix (hold/det terms 2/6 both runs). Why: if trunk-gradient bleed from the stance BC-anchor loss were the sole cause of the anchor4-class freeze, isolating the trunk from that gradient should never hurt an already-clean seed's walk -- it did, so trunk-bleed is not a sufficient explanation on its own. Joint call with -s1 below decides the arm; per this arm's own pre-registered gate text (neither seed reaching det gait_valid>=5/6 nets FAIL), escalate to critic-level dig-in, do not fund a further trunk-detach dose/variant.

