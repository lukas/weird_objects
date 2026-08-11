# EVALS — every evaluation metric, what it means, where it lives

Status date: 2026-08-10 (operator ruling: the per-eval total scores
are how we know a model is improving — they get clear names, the top
of the W&B page, and this doc). If a metric is not documented here,
document it before using it in a verdict.

## 1. Periodic training evals (the improvement curves)

Every `--eval-every` steps the trainer freezes the policy
(deterministic), isolates ONE goal mode at a time (all other `p_*`
zeroed), runs 2 episodes per mode, and logs to W&B. Shared by both
trainers (`train_ppo_sim._run_periodic_eval`; the MJX trainer imports
it), so metric names are identical everywhere.

### SCORE/ — the headline section (top of the W&B page)

Pinned via `wandb.define_metric("SCORE/*", summary="last")`: the
latest value of each appears in the run Overview summary, and the
SCORE section sorts above canary/env/eval in the workspace.

| metric | meaning | direction |
|---|---|---|
| `SCORE/<mode>_total_reward` | ALL reward terms summed over one eval episode of that isolated mode, mean of 2 episodes. Was called `eval/<mode>/return` before 08-10 (an awful name: it is not a discounted return, it is the episode's total earned reward). | up, **but see the caveat** |
| `SCORE/rise_flat_success` `SCORE/rise_bridge_success` `SCORE/rise_crouch_success` | rise completion split by start kind (2 eps each): survived AND final height err ≤ 15 mm. flat/bridge are THE rise metrics; crouch is solved and must stay 1.0. Was `eval/rise_<kind>_frac`. | up |
| `SCORE/raise_success` | survived AND final height err ≤ 5 mm (deliberately tight — canary: if not ~100% the height pathway is broken, not under-trained). Was `eval/raise_success_frac`. | 1.0 |
| `SCORE/lower_success` | survived AND final height err ≤ 15 mm. Was `eval/lower_success_frac`. | up |
| `SCORE/tipped_recovery_success` | (added 08-10, after the hardware runaway roll) forced 12° roll-tipped start with a LEVEL tilt reference, in the run's primary mode (walk for walk runs, else hold); success = survived AND mean \|roll − ref\| over the last quarter ≤ 3° AND body within 90 mm (walk; the gait normally rides 54–70 mm below the spawn settle) / 30 mm (hold) of the settled start height — belly-flat reads level and must not count. The 12° dose is capped by the run's own tilt envelope (stance 10° → 7° tips), so compare within one envelope. Baselines at 12° (fixed gate, 8 eps): dep-vref1-r1 7/8, dep-tip1 6/8, null policies 0. **Caveat:** curves logged 08-10 (incl. cw-dep-tip1's 0/2) used a flat 30 mm gate that failed every healthy walk recovery — ignore them. Static-lean recovery was already present in the champion; this eval did NOT reproduce the hardware runaway (progressive lean with a pinned loaded leg mid-gait — a sim-to-real grip gap). It remains as a regression floor. | up |

**The caveat on `_total_reward`:** it is measured under the RUN'S OWN
reward config. It is the right curve for "is this run still
improving," and NEVER comparable across runs with different reward
cfgs (a run with an extra income term scores higher forever). A run's
resolved reward config is in its W&B notes (`=== REWARD FUNCTION ===`
block) and `config.reward_cfg`; term meanings in rl_docs/REWARD.md.
For cross-run comparisons use the success/error metrics or the offline
harness (§2). This is also why the auto-continue logic reads
`rollout/ep_rew_mean` quarters — same caveat applies there.

### eval/ — per-mode detail

| metric | meaning |
|---|---|
| `eval/<mode>/survived_frac` | fraction of eval episodes not safety-terminated |
| `eval/<mode>/track_err_deg` | mean \|tilt − reference\| over the episode |
| `eval/<mode>/height_err_end_mm` | \|height − ref\| at episode end |
| `eval/walk/vel_err_m_s`, `eval/walk/speed_m_s` | mean commanded-velocity error / achieved speed |
| `eval/tipped/roll_end_deg`, `eval/tipped/z_drop_mm` | tipped-start eval detail: mean \|roll − ref\| over the last quarter / body-height drop vs the settled start (>30 mm = collapsed, not recovered) |
| `canary/<case>`, `canary/auto_stop` | protected-skill regression flags (0/1) |

### Naming history (for reading old runs)

Runs before 2026-08-10 logged `eval/<mode>/return`,
`eval/rise_<kind>_frac`, `eval/{raise,lower}_success_frac`. The W&B
UI will not overlay old and new names on one chart; when comparing
across the rename, pull both keys (`ops.sh wandbdump`).

### Other W&B curves (not evals)

`rollout/ep_rew_mean` (SB3, training-noise rollouts — what the
watcher's "reward quarters still climbing" auto-continue reads),
`env/<part>` per-term reward means (see rl_docs/REWARD.md),
`train/*`, `time/*`, `lp/*` (walk-speed curriculum),
`terminations/<reason>` (MJX trainer).

## 2. Offline checkpoint harness (`eval_checkpoint.py`) — the gate

6+ episodes per mode, det (+ sto with `--stochastic`), report.json +
videos + contact sheet; used for every gate verdict. Headline fields
per episode: `success` (posture-strict since 08-08; `--valid-plant-gate`
adds the geometric PLANT_SPEC for rise/raise once champions are
baselined), `return` (same total-reward caveat), `progress_ratio`
(walk: along-command distance / commanded; promotion band 0.75–1.25),
`slip_per_m` (loaded foot-XY travel per meter of progress — the
skating metric; the REAL robot walks at ~1.0), `gait_valid`,
`end_posture_ok` / `end_clear_mm`, `valid_plant` / `plant_fail` /
`plant_margin_mm`, currents. With `--baseline <parent>` it evaluates
the frozen parent under identical config/seed (matched-parent
control, binding for injected-axis verdicts).

The harness mirrors its summary into the training run's W&B page
under `eval/<dr-tag>/<mode>_<det|sto>/...` (summary fields, not
charts) and uploads report.json.

## 3. Specialist harnesses

- `eval_drive.py` — the joystick gate: scripted command schedule;
  falls, tracking error, distance. Gate wording lives in the run's
  ledger entry.
- `eval_yaw.py` — turn-segment |wz_err| median (pass ≤ 0.10 rad/s)
  and heading-hold |wz| median (pass ≤ 0.05). Judge turn arms with a
  matched-parent control (rl_docs/TURN.md).
- `test_task_semantics.py` — MDP_PREFLIGHT banks (RISE/WALK/TURN
  passing, LOWER owed): required orderings of scripted policies under
  the full reward stack BEFORE any arm of that mode launches.

## 4. Rules of use

1. A gate verdict quotes harness numbers (§2/§3), never SCORE curves.
2. SCORE curves answer "still improving?" and "which skill broke?"
   at a glance; regressions there mean watch the video before any
   other move.
3. Any new metric: add the row here + the definition in the emitting
   code in the same commit.
