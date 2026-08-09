# Architecture deep dive — how "advanced" should the walk policy's model be?

Date: 2026-08-09. Operator-requested review. Complements
HEXAPOD_RL_LITERATURE_REVIEW_2026-08-08.md §1/§2/§8 (asymmetric
actor-critic, temporal actor, model-size skepticism) with a wider,
current (2025–2026) survey of policy architectures for legged
locomotion, and a concrete recommendation ladder for THIS robot.

## Bottom line (operator-approved 08-09)

Transformer policies are NOT the move for a single blind hexapod at
25 Hz. The field's successful sim-to-real recipes for exactly our
setting (proprioception-only, one embodiment, embedded deployment)
converge on: **small MLP policy + short observation/action history +
a small learned estimator that compresses that history into "body
state + environment context."** History is the advanced model; the
container stays boring.

## What the literature actually says (2021–2026)

- **RMA (Kumar et al., RSS 2021; A1 quadruped).** Base MLP policy
  conditioned on an 8-dim "extrinsics" latent z, encoded during
  training from privileged env params (friction, payload, terrain).
  At deployment an *adaptation module* — a small 1D-conv net over
  ~0.5 s (k=50 steps) of state/action history — regresses ẑ online.
  Zero-shot real-world: rocky/slippery/deformable terrain, oil slick,
  foam. No reference trajectories, no foot-trajectory generators.
  Bipedal follow-up (2205.15299) adds: finetune the base policy under
  the *imperfect* estimated ẑ (phase 3), because a policy trained on
  perfect z degrades under estimator noise.
- **DreamWaQ (ICRA 2023; A1).** One-stage alternative to
  teacher-student: a Context-aided Estimator Network (CENet, tiny
  β-VAE encoder over observation history) jointly trained with PPO,
  outputs estimated body velocity + 16-dim context latent for the
  actor; the critic sees privileged state (asymmetric AC). Encoder
  ~128×64 units. Long real-world proprioception-only outdoor runs.
  Direct extension of "concurrent policy + state estimator" (Ji et
  al., RA-L 2022). This avoids the multi-stage distillation error
  accumulation the teacher-student pipeline suffers.
- **Transformers in locomotion (2023–2026)** earn their keep for:
  (a) *cross-embodiment generalists* — LocoFormer (2509.23745):
  Transformer-XL over many-second contexts, trained across bodies,
  in-context adaptation; (b) *multimodal fusion* — LocoTransformer
  etc.: proprioception tokens + depth-image tokens; (c) *unifying
  teacher+student in one net* — ULT (2503.08997): causal transformer
  where proprio tokens can't attend to privilege tokens, RL +
  next-state prediction + imitation in one stage. All three motivations
  are absent here: one body, no camera, sub-second gait period.
  Vanilla-transformer cost scales quadratically with context; at
  25 Hz even 5 s is >100 tokens for no demonstrated blind-locomotion
  gain over history-MLP/GRU at our scale.
- **CPG hybrids (hexapod-specific, 2023–2025).** RL-tuned central
  pattern generators remain common for hexapods (CPG-RL, Bellegarda
  & Ijspeert; terrain-adaptive CPG works; fault-tolerant PMTRL-CPG
  2025 with 95–100% success over 10 cm obstacles and single-leg
  failures). They buy sample efficiency and guaranteed rhythm by
  restricting the action space — at the cost of expressiveness and
  another hand-designed prior. Our step-event reward + history-MLP
  just produced a six-leg gait from scratch WITHOUT a CPG prior
  (step0/hist8 lineage), so the prior is not currently needed; CPG
  remains a fallback if rhythm proves unlearnable at DR 1.0, and a
  natural structure for the fault-tolerance party trick someday.
- **Recurrent (GRU/LSTM) policies** sit between frame-stacks and
  transformers: used in student encoders (e.g. 24-step LSTM students
  in 2025 teacher-student work). Worth an A/B only if frame-stack
  hits a ceiling that looks like "needs longer memory than ~0.5 s".

## Constraints that decide it for us

1. **Deployment: 25 Hz on the robot's small Linux board.** A
   frame-stacked MLP is microseconds and constant-latency; attention
   caches introduce latency jitter exactly where the safety layer
   lives. Policies in the cited sim-to-real work are <1 MB.
2. **Training: CPU pods, SB3 PPO, 48 envs.** Transformer-policy RL
   wants GPU-scale batch; that bet is the MJX port, not this track.
3. **Evidence tonight:** cw-walk-step0-hist8 (8-frame stack, MLP)
   from scratch is ahead of the no-history baseline's pace (572 @
   3.0M vs 586 @ 4M) with healthy std 1.75 — the cheapest temporal
   model is already paying. Campaign review §8 ordering (history >
   recurrence > size) stands.

## Recommendation ladder (in order; each rung is falsifiable)

1. **NOW (running):** 8-frame history MLP (hist8 lineage). Keep.
2. **NEXT (after 0-c stability pricing lands):** asymmetric critic
   (already plan rung) + **DreamWaQ-style concurrent estimator**:
   small encoder over the same history window, trained jointly with
   PPO to (a) estimate body velocity (privileged in sim, absent on
   hardware) and (b) emit a ~16-dim context latent; actor consumes
   estimates, critic keeps privileged state. ~50k params. One stage,
   no teacher-student. Gate: walk metrics unchanged or better with
   the actor consuming ESTIMATED velocity instead of privileged vx/vy
   — that gate is literally the sim-to-real deployability check.
3. **AT HARDWARE TIME (DR 1.0 passed):** RMA-style extrinsics module
   if one policy over the full DR range underperforms per-range
   experts; include the bipedal-RMA phase-3 finetune under imperfect
   estimates. Party tricks (fall recovery, quadruped mode) stay on
   the same MLP+history+estimator stack — published recovery work
   needs no more.
4. **NOT PLANNED:** transformer policies — revisit only if (a) a
   second embodiment appears, (b) a camera is added, or (c) rhythm
   fails at DR 1.0 and LONG memory (>1 s) is the diagnosed miss
   (then A/B a GRU first, it's cheaper). CPG prior: fallback only,
   same trigger (c).

Sources: RMA rss2021/2107.04034 + 2205.15299; DreamWaQ 2301.10602;
Ji et al. RA-L 2022 concurrent estimator; LocoFormer 2509.23745;
ULT 2503.08997; LocoTransformer (ICLR'22); KiVi 2509.23650;
CPG-RL RA-L 2022; PMTRL-CPG ICIA 2025; hierarchical hexapod CPG-DRL
birob 2025.100231.
