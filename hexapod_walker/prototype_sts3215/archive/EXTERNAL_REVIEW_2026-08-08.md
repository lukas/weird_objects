# Hexapod RL — Integrated External Review & Action Plan (2026-08-08)

Sources: GPT external review + Claude second-pass review of the same status
brief (`GPT_STATUS_2026-08-08.md`). Where the two disagreed, the resolution
is stated explicitly and marked **[RESOLVED]**. This document has been
merged into `RL_PLAN.md` (priorities, gates) and
`orchestrator/ORCHESTRATOR_PROMPT.md` / guardrails YAML (autonomy
sections); this archive copy is the full-detail reference. Nothing here
relaxes any standing constraint.

---

## 0. Shared core diagnosis (both reviewers agree)

PPO is not failing to understand walking; it is finding easier optima than
the behavior we mean by "walk." The shuffle/flag-leg gait is a
reward-landscape problem, not evidence PPO cannot learn hexapod locomotion.
Four shaping levers are now refuted (command widening ×2, 3× progress,
all-modes flag penalty). Stop iterating penalty coefficients.

Rise does not require six simultaneous contacts throughout recovery. Target:
sensible six-leg participation and a stable, balanced six-foot **final**
stance. Legs may legitimately unload/reposition mid-recovery (this is why
the all-modes flag penalty collapsed rise: belly-starts need >50 mm
transient swings).

---

## 1. Priority sequence [RESOLVED — supersedes both individual orderings]

Let round 8 finish, then:

1. **Asymmetric actor–critic vs NV controlled comparison.** Already built
   (`asym_policy.py`, tests passing, smoke checkpoint done). Launch.
   Controls held fixed: parent, actor capacity/obs, reward, curriculum,
   steps, seeds, eval. Compare learning curves and seed distributions, not
   single finals. See §6.
2. **Speed-range diagnostic (NEW — Claude, cheap, potentially dissolves the
   flag-leg problem).** One run commanding 0.10–0.15 m/s. Hypothesis: at
   2–6 cm/s a drag-shuffle is genuinely near-optimal; at higher speeds
   shuffling physically cannot track and stepping is forced to emerge.
   See §3.
3. **Learning-progress speed curriculum** (bucketed commands, frontier
   sampling). Bucket seeding depends on the outcome of item 2 — the
   frontier may start fast and refine slow, not the reverse. See §4.
4. **If gait is still pathological: phase-based periodic reward
   (alternating tripod)** — see §2 for why this is ordered *ahead of* the
   dense step-component decomposition.
5. **Temporal deployable actor**: ~300 ms frame stack vs modest GRU
   ablation. Treat history as online system identification (implicitly
   recovers velocity, contact, lag, deadband, friction response).
6. **Dense step-component walk reward** — fallback only, if 1–5 fail. See
   §2 caveats before implementing.
7. Model-size sweep later. Raise-specific work only if the current
   diagnostic exposes a general problem (see §7).

In parallel, regardless of the above: eval canaries (§5), regression
auto-stop (§5), and autonomy hardening (§8) — these are cheap and protect
everything else.

## 2. Walk reward: phase prior before dense decomposition [RESOLVED]

**GPT's position:** implement a nine-component dense walk reward (swing
progress, swing clearance saturating, touchdown, stance no-slip, six-leg
participation, etc.) before any structural prior.

**Claude's counter, adopted:** each dense contact-based term is a new
exploit surface with its own coefficient; contact-dependent shaping in
MuJoCo is notoriously farmable (foot vibration at the contact threshold to
harvest swing/touchdown credit). The nine-term decomposition risks
replacing a 1-D coefficient search with a 9-D one. The sim-to-real legged
locomotion literature converged on simple rewards + curriculum or an
explicit **periodic/phase-based reward structure** (Siekmann et al.,
Cassie, "periodic reward composition" — a reward formulation, not a
trajectory prescription, i.e. exactly the "weak prior" GPT allows in
principle, but proven on hardware).

**Adopted plan if `cw-walk-flagw` fails its gate:**

- Implement a **weak alternating-tripod contact-phase reward**:
  - Phase A stance tendency L1/R2/L3, swing tendency R1/L2/R3; reverse for
    phase B.
  - Actor observes `sin(phase), cos(phase)`.
  - Modest reward for contact-state agreement with the phase. **Do NOT**
    prescribe joint angles, foot trajectories, body trajectory, or rigid
    timing.
  - Routing: WALK MODE ONLY (per §2b).
- Keep the dense decomposition (GPT §1) in the queue as the fallback if the
  phase prior also fails across genuine seeds. If it is ever implemented:
  log every component independently, saturate clearance (20–30 mm
  sufficient; 150 mm must not score higher), and expect an exploit-hunting
  eval pass immediately after.

**Additional cheap lever (Claude, do alongside):** inspect per-servo
current on the shuffle gait — dragging feet against friction should show as
elevated current on the dragging legs. If confirmed, a **walk-routed
effort / cost-of-transport term** may suppress the shuffle nearly for free
using telemetry we already collect. Check existing reels before writing any
new code.

### 2b. Reward routing (GPT §2 — confirms existing constraint, now formal)

- GLOBAL: safety, excessive current, joint limits, dangerous impacts,
  universal smoothness. Nothing else.
- MODE-SPECIFIC: all gait morphology, swing/touchdown/clearance, stance
  contact/load, rise/lower progress, phase-agreement terms.
- Every new term declares its routing at introduction. Interference proven
  three times; this is not negotiable.
- Flag-leg **detection stays as an evaluation gate permanently**, even if
  its reward coefficient is later reduced or removed (GPT §3).

## 3. Speed-range diagnostic (NEW)

- One run (plus seed twin if a slot is free), commands 0.10–0.15 m/s,
  otherwise identical to the current walk config.
- Prediction if hypothesis true: flag-leg/shuffle disappears or sharply
  reduces at high commands; gait metrics (§5b) show real swing cycles.
- Prediction if false: policy finds a high-speed drag variant or falls;
  flag leg persists.
- Either outcome reshapes the curriculum (§4): if true, seed the frontier
  at high speeds and expand downward; if false, proceed with GPT's
  low-to-high buckets.

## 4. Learning-progress speed curriculum (GPT §9, amended by §3 outcome)

- Buckets (initial proposal): 0.02–0.03, 0.03–0.04, 0.04–0.05, 0.05–0.06,
  0.06–0.07, 0.07–0.08, 0.08–0.10, 0.10–0.12 m/s.
- Per bucket track: tracking error, valid-gait rate (§5b), falls, slip,
  current, recent improvement.
- Preferentially sample the moving learning frontier — neither solved nor
  currently impossible. Never return to abrupt global range widening
  (refuted twice).
- Bucket ordering/seeding to be revised after the §3 diagnostic.

## 5. Evaluation & mid-run monitoring

### 5a. Fixed-seed canaries replace 2-ep random mid-run evals (GPT §11)

- Identical cases every probe: flat seeds 1001/1002, bridge 2001/2002,
  crouch 3001/3002 (extend per mode as needed).
- Mid-run question becomes: "did this checkpoint lose a behavior it
  previously demonstrated on identical cases?"
- Randomized ≥20-episode harness evals remain the promotion standard.

### 5b. Automatic gait-validity metrics (GPT §4)

Per leg, per eval: contact duty cycle, swing count, touchdown count, swing
length, max clearance, sustained high-clearance time, stance slip,
loading/current, joint-range occupancy, time near limits. Plus velocity
tracking, distance, falls, stability, cross-leg load imbalance.

**A walking checkpoint is invalid if any leg is persistently sacrificed,
regardless of velocity error.** Eval definitions must be independent of the
reward terms.

### 5c. Automatic regression stopping (GPT §12)

- Warm-start multi-skill runs store parent canary performance at launch.
- Protected skill below conservative threshold for 3 consecutive canary
  probes → terminate the run automatically. (The `cw-walk-flag` rise
  collapse would have been caught millions of steps earlier.)
- Implement before the next warm-start launch.

## 6. Asymmetric actor–critic comparison (GPT §7, confirmed)

- NV PPO: actor = hardware-realistic obs, critic = same.
- ASYM PPO: identical actor obs; critic additionally gets privileged sim
  state (true body velocity, exact contacts, pose).
- Everything else held fixed. Compare curves + seed spread; sample
  efficiency is itself an outcome.
- **8M steps is a fair budget** to call NV weak *at 8M* — this is a
  fixed-budget sample-efficiency comparison, not an asymptote claim. No
  over-crediting risk: the asym actor is deployment-identical to NV's, so
  the deployed artifact is the same class either way. Do not extend NV
  beyond 8M unless asym fails to dominate.

## 7. Raise (GPT §10, confirmed)

- Finish the `cw-stance-raisemix` diagnostic before acting on any
  hypothesis. Do not overweight the velocity-zeroed run's good raise result
  — one run at known eval-noise levels is not evidence.
- Then classify failures from trajectories: lost contact, joint saturation,
  barely-missed threshold, pitch/roll, insufficient extension, current
  constraint, ambiguous goal/end-state.
- If nothing general emerges: demote raise to canary status and stop
  spending meaningful compute on it.

## 8. Autonomy hardening (GPT §13–16 + Claude additions)

Two demonstrated failure classes: semantic (optimistic video verdict) and
operational (phantom launch). Prompting mitigates the first; **only
deterministic software fixes the second. The LLM is never authoritative
about operational state.**

### 8a. Mechanical launch verification (two-phase commit)

Record intent → launch → verify mechanically → only then mark RUNNING:
remote git SHA/tag, job/process existence, W&B run existence, W&B config
SHA, expected seed, advancing heartbeat/step count, active expected
compute, no duplicate run ID.

At completion verify: W&B finished, expected steps reached, checkpoint
exists + checksum recorded, eval artifact exists, video exists and decodes,
RL_LOG verdict exists.

### 8b. Structured experiment ledger (GPT §14)

Software-owned structured facts per experiment: run, hypothesis, parent,
git SHA, seed, status, explicit success criteria, launch verification, W&B
ID, checkpoint path/checksum, verdict. `RL_LOG.md` stays as narrative on
top. The watcher's dedupe fix keys on this ledger, not on RL_LOG parsing.

### 8c. Verdict format (GPT §15)

Every autonomous verdict contains, in order:
- OBSERVATIONS — machine-generated metrics + visible behaviors (mechanical
  where possible)
- INTERPRETATION
- VERDICT — PASS / FAIL / INCONCLUSIVE + hardware-ready YES / NO
- HYPOTHESIS STATUS — SUPPORTED / REFUTED / INCONCLUSIVE

### 8d. Falsifiable hypotheses (GPT §16)

Before launch, record: hypothesis, prediction if true, prediction if false,
strongest alternative explanation, why this experiment distinguishes them.
Reject experiments whose rationale is "we haven't tried this coefficient."

### 8e. Claude additions

- **Blast-radius cap per cycle**: max N new launches and a compute-budget
  ceiling per agent cycle, so one confused cycle cannot burn a weekend of
  pods.
- **Video-review provenance**: verdicts must reference the artifact
  hash/checksum and frame count of the video actually reviewed — closes the
  semantic-failure loop the same way launch verification closes the
  operational one.

## 9. Later / conditional

- **Contact-from-proprioception auxiliary head** (GPT §17): once a temporal
  actor exists, train joint/current/IMU/action history → 6 foot-contact
  predictions. Free latent state if recoverable; may improve the temporal
  representation. Queue after §1 item 5.
- Model-size sweep: only after the above.

## 10. Gates currently in force (unchanged, restated for the agent)

- `cw-walk-flagw` passes only with sto walk ≥4/6 @ vel_err ≤0.030 AND
  video shows a six-foot gait with no flag leg AND sto rise ≥4/6.
- Scalar gates NEVER override a bad video. Pathology-first verdicts.
- ≥20 episodes for gate decisions; rise/lower split by start kind; DR 0 +
  own-DR evals; noise-response curve per champion.
- Keep-best champions, append-only, per skill. Never keep-last.
- Hardware gate unchanged (status brief §7), including torque→current
  recalibration before trusting any current gate (MuJoCo 3.11 shift
  ~+0.14 A at quiet hold).
- Never touch the physical robot.
