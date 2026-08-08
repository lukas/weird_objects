# RL best-practices audit vs field standard (2026-08-08)

Sources: RSL-RL / Isaac Lab locomotion configs (ANYmal, Unitree G1/H1),
"What matters in on-policy RL"-style implementation guides, andyljones
RL-debugging (probe envs, reward scaling), Menlo "Noise is all you
need" (sim-to-real via timing jitter + firmware-in-loop), arXiv
2509.06342 (systematic sim-to-real for legged robots: actuator-first
system ID + physics-grounded energy loss, minimal DR).

Our config (SB3 PPO, 48 envs, 25 Hz, 250-step episodes):
net [128,128] tanh, log_std_init −1.0 (std 0.37), n_steps 256,
batch 2048, lr 3e-4 FIXED, gamma 0.99, gae 0.95, ent_coef 1e-3,
clip 0.2, no VecNormalize, no target_kl, SB3 defaults elsewhere
(n_epochs 10, max_grad_norm 0.5, advantage norm on).

## Gaps that matter (ordered by suspected impact)

### 1. Exploration is 3–10x below locomotion standard  [HIGH]
Field standard for learning gaits from scratch: init action noise
std 1.0 and entropy coef 0.005–0.01 (every RSL-RL/Isaac locomotion
config). Ours: std 0.37 and ent_coef 0.001. Both were deliberately
lowered early in the campaign to stop violent exploration on the
body-IK line — but they were never revisited for the raw-joint line.
Consequences consistent with our whole pathology record: premature
convergence to shuffles, "noise-fragile choreography", warm-started
runs that never escape their basin. DIRECTIVE: the fresh/basin-escape
phase arm (and any future from-scratch run) uses log_std_init 0.0
(std 1.0) and ent_coef 0.005–0.01; log entropy and treat entropy
collapse (<~0.01 nats/dim) as a run-health alarm. Warm-started
refinement runs may keep lower noise deliberately — but say so.

### 2. No KL control on updates  [HIGH]
Every locomotion stack uses adaptive LR targeting KL ~0.01 (or at
least KL early-stop). We run fixed 3e-4 with SB3 n_epochs=10 and no
target_kl — nothing prevents a destructive update from wiping a
learned skill mid-run (a plausible mechanism behind sudden skill
erosion in warm starts). DIRECTIVE: set SB3 `target_kl≈0.02` (epoch
early-stop) on all runs now; consider RSL-style adaptive LR later.
Log approx_kl; a persistent KL >0.05 in early epochs is a data-staleness
bug, not a tuning knob.

### 3. Reward/obs scale audit is folklore, not a check  [MED]
Standard: per-component rewards land roughly in [-10, +10] per step
aggregate, obs components O(1). We log reward parts to W&B but have
never asserted scales; several parts (task kernels, penalties) were
retuned many times. DIRECTIVE: one-off audit script — mean/std of
every reward component and every obs dimension over 10k steps of (a)
frozen champion, (b) random policy; flag any obs dim with |mean|>3 or
std>10 and any reward part dominating >80% of total variance. Attach
to the ledger; rerun after any reward change (root-cause rule already
requires the chain — this provides the numbers).

### 4. Symmetry augmentation — free sample efficiency  [MED]
Isaac ANYmal configs ship left/right symmetry data augmentation. A
hexapod has L/R mirror symmetry (and near 120° rotational symmetry).
Mirroring states/actions doubles effective data and biases toward
symmetric gaits — directly attacks our asymmetric one-leg-sacrifice
exploits. QUEUE (after phase-arm verdicts): implement mirror
augmentation for the raw-joint obs/action layout; verify with a unit
test that mirrored rollouts are physically consistent.

### 5. Sim-to-real: system-ID over broad DR; timing jitter  [ALIGNED, 2 gaps]
The 2025-26 consensus matches what we already do (motor-dynamics
probe, latency/deadband modeling, privileged-critic) and what we just
directed (physics-grounded energy pricing — arXiv 2509.06342 uses a
first-principles energetic loss exactly like our current-pricing
directive). Two gaps: (a) CONTROL-TICK JITTER — we model constant
latency; real loops jitter. Add randomized per-tick delay (±1 tick)
to the ServoProfile DR. (b) OBSERVATION DELAY — verify IMU/encoder
reads are staggered/delayed like the hardware path, not same-tick
truth. Cheap to check, known transfer-killer.

### 6. Probe-environment discipline (andyljones)  [LOW, adopt the idea]
We already use canaries and a raise "canary task". Missing rung: when
a NEW mechanism ships (phase reward, asym critic), first prove it on
a trivial probe — e.g. phase reward: a single leg in the air must
learn to touch/lift on the phase clock in <100k steps at DR 0. If the
probe fails, the mechanism is bugged; don't burn a 4M-step run to
find out. DIRECTIVE: each new mechanism gets a probe defined at
implementation time, run as a smoke.

### 7. Deliberately NOT adopting
- 4096-env GPU-style batches, [512,256,128] nets, ELU: our scale is
  CPU MuJoCo; net-width/activation is a queued ablation, rank last
  (plan already says so). Note RSL-RL runs actor obs normalization
  OFF in the ANYmal config — our no-VecNormalize choice is defensible;
  the audit in (3) decides if any dim actually needs scaling.
- Off-policy/SAC switch: no evidence PPO is the bottleneck; every
  failure so far traced to reward/pricing/exploration, not algorithm.
