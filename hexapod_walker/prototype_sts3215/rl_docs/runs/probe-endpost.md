# probe-endpost

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-09T00:18:26+00:00

**pod**: hexapod-sweep-friction

**steps**: 150000

**parent**: ppo_goal_cw_stance_dr10.zip (md5 da1d912a)

**hypothesis**: Mechanical probe of reward.k_end_posture (new mechanism, audit §6): no tracebacks to 150k, reward_end_posture part present only in terminal windows of rise/lower/raise episodes, magnitude within audited band (flag leg -1.2/tick, grounded ~0). Snapshot 5589bc4.

**gate**: mechanical only: healthy to 150k, reward_end_posture in audited bands, no traceback

**verdict**: LAUNCHER VERIFICATION FAILED but probe evidence COMPLETE. The 150k smoke at ~2430 fps FINISHED (log ends exactly at budget 19,344,960 cum, 0 tracebacks, std 0.198 sane) before the launcher growth check, which read the stopped log as a dead run and its cleanup pkill killed the trainer during final save (no ckpt). NEW LAUNCHER BLIND SPOT: finish-before-verify race on fast smokes. Part bands verified locally instead (cycle-13 pattern): champion ckpt rolled in exact endpost cfg -> lower flag ending -0.82..-1.20/tick over 30-39 terminal ticks (audit predicted -0.95), raise -0.15..-0.47 (predicted -0.50 for 120mm), planted rise endings ~0, no charge outside terminal windows. Mechanical gate MET on combined evidence.

