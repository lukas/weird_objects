# walkcurr — prior-free walking curriculum (Kawawa-2022 lineage)

Registered 2026-08-23 by operator order (MCP focus note
20260823T154657Z) after the `cw-kawawa2022-pf-flat1` FAIL. Plain
English: teach a from-scratch PPO policy (no gait clock, no BC
teacher, no motion prior) to walk by climbing a curriculum that starts
with ONE fixed forward command and only widens after certified passes.

## Goal (DONE gate)

A prior-free policy passes a held-out C-env contextual walking panel
(fixed forward + heading set + irregular direction changes) with zero
falls, directions actually followed, low slip/m, all-six-leg gait
validity, on video. Speed obedience is secondary throughout.

## Binding track rules (operator, 08-23)

- **Walk-only diet**: every rung trains with `goal.walk_pure=1`. The
  flat1 failure mode (hold/raise/track/unload carrying aggregate
  reward while walk dies) must be impossible by construction.
- **Bank before launch**: any reward-mechanism change re-proves the
  WALKCURR_PF ranking bank (test_task_semantics.py): clean commanded
  walking > park/stall > sideways/reverse/wrong-way >
  high-slip/skate/fall, under the run's exact cfg.
- **Triage rule**: every triage logs reward trend AND walk-eval trend.
  Reward rising while walk eval is flat/down or walk terminates =
  MISALIGNED -> stop same-recipe seeds/continuations, audit
  reward/eval/simulator. No same-recipe seed sweeps past a misaligned
  read.
- Slip is priced by charge (loadslip excess), never by a hard early
  gate that teaches parking, unless bank and eval agree.

## Rung ladder

1. **fwd1 (NOW)**: fixed forward 0.05-0.06 m/s, heading 0, DR0,
   discovery 2M. Recipe: `rl_move/sim/kawawa2022_recipe.py`.
2. Small heading set (± up to ~15 deg), one command/episode.
3. Full fixed headings.
4. Irregular direction changes (mid-episode resampling).
5. DR/push hardening (paper's friction 0.5-1.25 + periodic pushes).

## Now

- **`cw-walkcurr-pf-fwd3-chargeramp` FAIL (verdicted 08-23 ~18:3x):**
  the dense-charge ramp fix did NOT unfreeze discovery either — gate
  eval 0/6 walk success (det+sto), prog_ratio ~0.00, speed
  0.003-0.034 m/s vs 0.05-0.06 cmd, slip/m 10.4-39.2 (gate <=3.0),
  contact sheet static splayed stance across all 10 frames — same
  pathology as fwd1/fwd2. `env/walk_freeprog_score` (see metric note
  below) sat flat NEGATIVE (~-0.06..-0.10) the entire 2M steps, never
  crossing zero. This is the THIRD independent reward-magnitude/
  schedule fix (fwd2 swing income, fwd2 term-penalty cut, fwd3 charge
  softening) refuted with the same frozen-video signature — the
  blocker looks upstream of reward shape: random-init PPO rollouts
  never visit a state with positive forward progress to reinforce, so
  no reshaping of charges the policy never triggers can matter.
  **METRIC DEFECT found (fwd3 triage):** the discovery-health litmus
  this section's gate text has been citing, `env/reward_walk_prog`,
  is a DEAD METRIC under this recipe — `walk_task.py` sets
  `r_prog = 0.0` unconditionally whenever `k_walk_freeprog > 0`
  (freeprog REPLACES it), so it reads exactly 0.0 regardless of
  actual behavior on every fwd1/fwd2/fwd3 run. It happened to agree
  with the true (video-confirmed) verdict every time so far, but is
  not a valid go/no-go signal — **use `env/walk_freeprog_score`
  (or `walk_freeprog_score` in eval reports) instead in all future
  rung-1+ gate text.**
  **UNTRIED lever surfaced:** `k_loadslip_excess` is EXCLUDED from
  `walk_charge_ramp` entirely (always full dose — a pre-launch safety
  fix after floor 0.15 inverted the skate/shuffle ranking at
  convergence-level pricing). It is the exact charge fwd1's dig-in
  named (the loadslip "travel-floor" that prices a tiny exploratory
  step's slip/progress ratio harshly since progress is floored at
  `loadslip_floor_m`=0.03 m) and no arm has yet tried softening it,
  even as a short bootstrap window (distinct from a permanent low
  floor, which the bank already rejected at convergence).
  Snapshot `exp/walkcurr-fwd3-chargeramp` (unchanged, FAIL result
  only).
  **`cw-walkcurr-pf-fwd4-logstd0` / `-fwd4-entboost` LAUNCHING
  (08-23 ~18:3x):** the pre-registered init/exploration fork, 2-arm
  dose pair off the exact fwd3 recipe (charge ramp kept — it's
  harmless and bank-proven safe, just not sufficient alone):
  (a) `--log-std-init 0.0` (std 0.37->1.0, ~2.7x wider initial action
  noise) to test whether the random-init rollout distribution simply
  never samples a forward-progress trajectory at the current noise
  scale; (b) `--ent-coef 0.01` (10x default 1e-3) to test whether
  entropy collapses to the frozen/park optimum before progress-
  bearing states are ever visited. Single-lever each, no reward-
  mechanism change (no bank re-proof required). Prediction-if-true:
  `walk_freeprog_score` leaves its flat ~-0.07 band and trends toward
  0 within 2M. Prediction-if-false (both freeze identically): the
  loadslip-bootstrap lever above becomes the next fork to build, or
  the prior-free MLP recipe is refuted at this budget per the rule
  below.

## Now (updated 08-23 ~20:0x)

- **`cw-walkcurr-pf-fwd4-logstd0` / `-fwd4-entboost` FAIL (both
  verdicted):** wider initial action noise (std 0.37->1.0) and 10x
  entropy coefficient (0.001->0.01) both leave `walk_freeprog_score`
  flat in [-0.10,-0.055] the whole 2M run — never trending toward 0.
  Gate eval 0/6 det on both, identical static splayed-crouch video to
  fwd1-fwd3. `fwd4-logstd0` additionally destabilized sto-mode into
  real falls (5/6 term, tilt_roll/tilt_pitch, one sacrificed leg) — a
  new, worse failure mode layered on the same freeze, not progress.
  Exploration-noise-scale hypothesis CLOSED.
- **`cw-walkcurr-pf-fwd5-loadslipboot300k` / `-600k` FAIL (both
  verdicted):** built + bank-proved + smoke-tested the pre-registered
  loadslip-bootstrap mechanism (`reward.walk_loadslip_bootstrap_steps`
  / `_min_frac`, walk_task.py + train_ppo_mjx.py, 6 unit tests +
  3 semantics-bank tests, smoke-verified end-to-end on-pod) — softens
  ONLY `k_loadslip_excess` for an early window, annealing back to full
  dose. **Bank floor-search finding**: 0.40 (the walk-charge-ramp's
  own validated floor) is UNSAFE for this charge alone — held statically
  at 0.40 with the other three discovery-friction charges at full dose,
  'sideways' (-231) out-earns 'park' (-352), a genuine ranking reversal
  (unlike the ramp's own 0.15/0.40 story, because nothing else is
  loosened alongside it here). A 0.5-0.8 sweep found **0.65** the
  lowest safe floor (park/stall stay strictly above every wrong-way
  gait, margin ~37; skate charge still cut ~30%, -1339->-940 at full
  dose vs floor). Trained at 0.65 for both 300k (15% of budget) and
  600k (30%) windows: **both freeze identically** —
  `walk_freeprog_score` flat in [-0.10,-0.05] the entire run, gate 0/6
  det+sto, identical static-crouch video. Window length was not the
  missing variable. **This closes the loadslip-bootstrap fork at both
  tested doses.**
- **Escalation triggered**: per the pre-registered rule below, BOTH
  init/noise (fwd4, 2/2 FAIL) AND loadslip-bootstrap (fwd5, 2/2 FAIL)
  are now refuted with aligned reward+eval (reward falls monotonically
  as the charges ramp against a frozen policy; eval flat/0 the whole
  way; adequate budget — 2M steps, 6 total rung-1 arms: fwd1, fwd2x2,
  fwd3, fwd4x2, fwd5x2 is 8 arms total, all FAIL). **The prior-free
  MLP-from-scratch recipe is refuted at the 2M discovery budget.**
  Escalate to a dig-in before any further reward-magnitude variant —
  see Next.

## Next

- **NEW DIAGNOSTIC (08-23 ~20:1x, cheap W&B-history read on
  `fwd5-loadslipboot600k`, no extra training): the freeze is a
  vanishing-GRADIENT local optimum, not (only) an under-exploration
  one.** `train/clip_fraction` goes to EXACTLY 0 by ~15% of the run
  and stays there (policy_gradient_loss magnitude shrinks toward
  ~1e-4, approx_kl toward ~4e-5) — PPO's updates become negligible
  very early and never recover, while `rollout/ep_len_mean` climbs
  steadily to the full episode length (freezing avoids every
  tilt-term, so episodes stop truncating early) and `ep_rew_mean`
  falls only because the ramped/annealed charges accrue over a longer
  and longer episode against an UNCHANGING policy — this is consistent
  across the reward-quarter pattern seen in every one of the 8 FAILed
  arms. Working theory: once the policy locks onto "stand still safely"
  in the first few rollouts, nearly all of the per-step reward
  (idle/park/height/heading charges, term risk) is a function of STATE
  and TIME since reset, not of the action taken from that state — so
  the value function learns to predict it well (explained_variance
  trends up to ~0.15-0.17) and the resulting ADVANTAGE for any
  alternative action is close to zero, starving the policy gradient
  regardless of how the reward is priced or how much noise/entropy is
  added on top of it. This would explain why every dose/pricing/
  exploration-noise lever failed IDENTICALLY: none of them change how
  much the per-step reward actually depends on the action once frozen.
  **CONFIRMED GENERAL (08-23 ~20:2x cycle, cached-history read on 3
  more FAILed arms, no training):** `fwd1` clip_fraction quarters
  [0.0054, 0, 0, 0] (zero from ~30% of run, approx_kl -> 4.6e-5),
  `fwd4-entboost` [0.0069, 0, 0, 0] (zero from ~35%, kl -> 4.0e-5),
  `fwd4-logstd0` [0, 0, 0, 0] (zero from the FIRST logged decile,
  kl -> 3e-7) — identical collapse across baseline, 10x-entropy, and
  wide-noise arms, with ep_len climbing in all three. The logstd0
  detail is the sharpest: WIDER initial noise made the gradient die
  FASTEST, directly supporting the theory that per-step reward is
  action-insensitive once frozen (noise samples all score alike, so
  advantages stay ~0 regardless of exploration scale). Noise/entropy
  levers are dead for this freeze; only mechanisms that make reward/
  novelty depend on the ACTION-reachable state (RND/curiosity, rung-0
  sub-goal, or state-dependent exploration) remain live.
  Original note (superseded where it says unconfirmed):
  **Not yet confirmed** (only one run's history was read) — the next
  cycle should (a) check 1-2 more of the 8 FAILed runs for the same
  clip_fraction/approx_kl collapse-to-~0 signature to confirm it's
  general, not one seed's artifact, and (b) if confirmed, prioritize
  fixes that restore action-sensitive gradient EARLY (before the
  freeze locks in) over fixes that only change the reward's resting
  values: a short forced-exploration/action-noise-floor window, a
  curiosity/RND-style bonus keyed to STATE novelty (still no motion
  prior), or rung-0 (a) below, rather than more entropy-coefficient or
  charge-dose variants (7 of those already refuted).
- **DIG-IN (rung-0 / architecture), not another reward-magnitude
  arm.** Candidates, roughly in order of how little they compromise
  "prior-free": (a) a genuinely prior-free EXPLORATION fix the reward-
  shape/noise-scale arms haven't tried yet — e.g. a curiosity/RND
  bonus, or a much bigger discovery budget (5-10M) before declaring
  the recipe dead at 2M, since every arm so far used the same 2M cap;
  (b) rung-0: an even simpler sub-goal than "walk 0.05-0.06 m/s" (e.g.
  "lift any foot" or "shift weight" as a separate certified step before
  full walking) — closer to a real curriculum than the current single
  rung; (c) `--gru --gru-hidden-size 64` (in-repo recurrent path) in
  case memorylessness itself is the blocker, though this hasn't been
  implicated by any finding so far; (d) last resort, most prior-
  breaking: a brief BC kickstart from a few seconds of scripted motion
  to escape the freeze basin, then continue prior-free — flag for an
  operator read given it brushes against the track's own "no BC
  teacher" rule.
- Once ANY rung-1 mechanism actually gates: rung 2 (small heading set)
  respec; consider the recurrent path + paper-pure proprioception-only
  obs A/B (`goal.walk_obs_body_vel=0.0`) once commands start changing
  mid-episode (rung 4).

## Key facts

- The RAW kawawa2022 reward stack was bank-REFUTED on 08-23: park
  (+387) out-earned clean walking (+325) under the walk goal alone —
  flat1's walk was misaligned even before the multi-goal diet starved
  it. v2e re-pricing measured: gait +346 > stall -31 > park -352 >
  sideways -609 > reverse -741 > skate -1058 > topple -1164.
- The harsh SLIPWALK doses (idle 20 / loadslip 6 / gait_gate) are
  refuted for from-scratch discovery (8 statue arms, amp track).
- Lost code: desktop temp commit b126ceb3 (RecurrentPPO/LSTM trainer
  support) was never pushed and the pod deploy copy was overwritten;
  recipe/tests/docs were recovered from the pod and re-landed
  canonically with `--activation-fn` trainer support (08-23).

## WAITING-ON

(none)
