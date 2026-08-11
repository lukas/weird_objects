# arch — Advanced architectures

W&B: tag `track:arch`. Excess-capacity research.

**Goal:** get a more advanced model (GRU/recurrent/temporal) to walk,
stand up, and sit down. What architecture learns the full skill set,
at what budget, with which failure modes.

## Now

- **From-scratch GRU walking CLOSED (08-11, gru-r4c):** both
  pre-registered levers (BPTT window 64→256 steps, hidden 128→256)
  tried and the identical leg-sacrifice/paddle fingerprint survived
  (legs [0,2,3] parked, det gait_valid 0/6) — worse, rise also
  regressed to 0/6 (r3 had it at champion grade). Reward already
  works on the MLP lineage, so this is a capacity/architecture limit,
  not a pricing gap. No further from-scratch GRU variant.
- Frame-stack line passed: hist16 → hist16-dep1 (deploy contract).
- **BC-distill-then-RL-finetune tried (08-11, `cw-arch-gru-bc-ft1`):
  walk survives, stance is MIXED, and a dig-in corrected the root
  cause.** The GRU was BC-distilled from the walk+hold specialists
  (rise was NEVER in the distillation data — the BC parent itself is
  a hold+walk artifact only) then PPO-finetuned 10M steps on a
  walk-heavy mix (60/15/15/10). Result across all four gate+own-DR0.5
  passes: walk gait_valid 6/6 throughout, no paddle at any point in
  10M steps (economy softened, slip/m 2.1-2.6 vs BC parent 1.5);
  lower IMPROVED (4-6/6 vs parent 2/6 det); rise stayed 0/6 (unchanged
  from the BC parent — RL never invented a skill it had zero demos
  for, not forgetting); hold REGRESSED 0/6 det (parent was 6/6 — RL
  genuinely traded hold precision for locomotion polish, real
  forgetting this time). So two different mechanisms in one run:
  data-poverty (rise, never learnable this way) vs erosion (hold,
  actually lost). Keeping `ppo_goal_cw_gru_bc.zip` (md5 864c02fb) as
  the hold+walk reference artifact.
- **lr/KL lever CLOSED (08-11, `cw-arch-gru-bc-ft2`, 2M discovery)
  — FAILS exactly as pre-registered.** 3x lower LR (3e-4→1e-4) +
  tighter target-KL (0.02→0.01) on the identical BC-parent finetune:
  gate (DR0) det rise 0/6 (bridge/crouch/flat all 0, honest stalled
  climb, plant_margin 122mm short, balanced duty — not a cheat), det
  hold 0/6 (all 6 eps fail valid_plant on height/feet_down/footprint,
  worst foot 19-33mm proud, balanced duty — honest precision loss,
  not a flag-leg). Walk stayed clean (gait_valid 6/6 det+sto, slip/m
  1.3-1.7 in-band). Two attempts now (ft1 default hp, ft2 tight
  hp) at protecting stance via PPO hyperparameters alone — both lose
  it identically. **Per RESEARCH_RULES "two misses in the same
  behavioral class": stop tuning lr/KL, change the mechanism.**
  **Root blocker found, CODE not config:** the gate's own
  pre-registered fix ("an auxiliary BC-anchor loss during the
  finetune, or freezing early layers") is not available for this
  network — `train_ppo_mjx.py` explicitly raises `SystemExit` for
  `--gru` + `train.bc_anchor_coef>0` ("not implemented, wraps stock
  PPO"), and there is no layer-freeze flag either. The MLP-lineage
  BC-anchor (`cw-stand-bc1`/`holdbc1`, twice-proven on hw track) does
  not reach RecurrentPPO as written.

## Next

- **The real next lever needs CODE, not another training run:**
  either (a) extend `bc_anchor.py`'s auxiliary-loss wrapper to
  `RecurrentPPO`/`GruActorCriticPolicy` (needs hidden-state-aligned
  minibatches, nontrivial), or (b) add a simple param-freeze option
  (freeze the GRU cell, finetune only the output head) — neither
  exists today. Until one lands, no arch-track spec is launchable
  for this lineage; do not queue a 3rd lr/KL/coefficient variant.
- Operator-directed next lever for RISE: re-distill stance-heavy +
  DAgger rounds on `distill_gru.py` (give the BC step actual rise
  demos before any further RL) — in progress outside this loop;
  unaffected by the above (it fixes the DATA, not the finetune loss).
- Later: contact-from-proprioception aux head; distill specialists
  into one recurrent net.

Detail: RL_PLAN.md "Architecture" · ledger cw-arch-* lineage.
