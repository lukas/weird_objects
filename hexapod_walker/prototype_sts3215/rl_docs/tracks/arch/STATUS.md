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

- **CODE landed (08-11 night): `bc_anchor.py` now supports
  `RecurrentPPO`/GRU** (hidden-state-correct aux step — anchors at
  the rollout's own hidden state, not h=0). Unblocks the CODE item
  below. Two arms ran on it 08-12:
- **`cw-arch-gru-anchor1` (10M hardening, warm from the ft1
  BC-distilled GRU, rise+hold+lower+walk all anchored) FAILS —
  but informatively.** Hold det 6/6 and lower det 6/6 (up from ft1's
  0/6 and 4/6) and rise det 2/6 with a non-flat start all clear their
  bars: **the anchor DOES protect/recover stance skills on a
  recurrent net**, exactly as it does on the MLP lineage. But walk
  freezes solid (gait_valid reads 6/6 with zero legs literally
  parked, yet prog_ratio 0.01, speed 0.001 m/s, video pixel-static
  for the full 15s clip) — the SAME twice-closed BC-anchor-on-walk-
  ticks failure (`cw-walk-gaitbc1`, `cw-omni-transbc1`), now
  reproduced on a GRU. Gate required both; overall FAIL.
- **`cw-arch-gru-scratch-anchor1` (2M discovery, from-scratch GRU +
  the same anchor stack) FAILS per its own pre-registration:** the
  anchor loss converges cleanly (0.010, under the 0.02 bar — a live,
  working teaching signal) but the walk mode still finds the
  parked-leg paddle (det gait_valid 0/6, leg idx1 flagged sacrificed)
  — "paddle beats a live anchor" was the pre-registered CLOSE
  condition. **From-scratch-GRU-with-anchor is CLOSED**; distill-
  then-finetune (ft1, warm from a BC-distilled net) remains the only
  path that has kept a GRU walking cheat-free.

## Next

- **The next arm is one variable off `cw-arch-gru-anchor1`: drop
  the walk-tick anchor, keep rise+hold+lower anchored.** anchor1
  proved the mechanism works for stance on this net; the walk
  freeze is caused by anchoring walk specifically (already twice
  closed on the MLP lineage) — never re-propose a walk-tick BC
  anchor. Untried combination, not a coefficient variant.
- Operator-directed next lever for RISE: re-distill stance-heavy +
  DAgger rounds on `distill_gru.py` (give the BC step actual rise
  demos before any further RL) — in progress outside this loop;
  unaffected by the above (it fixes the DATA, not the finetune loss).
- Later: contact-from-proprioception aux head; distill specialists
  into one recurrent net.

Detail: RL_PLAN.md "Architecture" · ledger cw-arch-* lineage.
