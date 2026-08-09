# cw-walk-step0-anchor-c1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: KILLED_BY_VERDICT

**created**: 2026-08-09T09:55:17+00:00

**pod**: hexapod-mjx-train-0

**steps**: 40000000

**parent**: cw-walk-step0-anchor

**hypothesis**: AUTO-CONTINUE of cw-walk-step0-anchor (watcher, directive 0-a): reward still climbing at segment end; identical config, init-from ppo_goal_cw_walk_step0_anchor.zip. Trailing cycle owns the verdict.

**gate**: DR0 harness 15s 6+6: gv 12/12, >=2 swings/leg, 0 term, det fwd mean >=0.55, agg slip/m det <=1.0 AND sto <=1.2, W&B walk_anchor_frac >0.85 earned (income held, not forfeited); frames: anchored stance vs paddle; verdict reads if-false shape (a) paddle-again vs (b) no-gait

**verdict**: Never started (launcher REFUSED on code-SHA mismatch: pod at 1002f7a, HEAD 038235f - cycle 32 snapshot was never synced to mjx-train-0). Cycle 33 verdict KILLS the lineage continuation regardless: parent fired if-false shape (a) at its full 40M budget (same paddle, slip WORSE than warm lineage, cadence-inflation from scratch, drummer-leg pathology in frames) - the basin question is ANSWERED and further identical-config steps optimize income inside a refuted basin. Do NOT relaunch.

**refused_reason**: hexapod-mjx-train-0 code marker 1002f7af954d1b169efbbdae05d76f5905e7ff6a != local HEAD 038235f023a8d07ab2cea9883c56df356043dc56. Sync first: snapshot.sh --sync hexapod-mjx-train-0 (and snapshot/commit before that if the tree is dirty).

