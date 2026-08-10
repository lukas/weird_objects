Hexapod First Hardware Walk: Deep Sim-to-Real Review
LLM handoff memo | 2026-08-09 | first 25 Hz hardware trace
Executive conclusion
This failure is unusually informative. It does not look like an inscrutable neural-policy transfer failure. Before the learned gait could really be tested, the deployed system violated major training assumptions: 97% of joint-ticks hit a hardware-only 1.5 deg/tick slew limiter, leaving a mean 48 deg gap between policy target and applied target; and large leg motions plus roll began while requested velocity was still zero. The known high-grip contact mismatch then converts the simulator's preferred loaded-foot slide into chassis torque. Missing body-velocity feedback and the tether plausibly amplify the divergence, but are not needed to explain its onset.
Recommended pivot: Freeze broad six-leg RL search; audit deployment equivalence; identify the real action/actuator pipeline; calibrate foot-ground contact and current/torque economics with scripted hardware motions; retrain with the exact deployable observation/action contract; then resume a physically supported hardware ladder. Keep anti-slip reward shaping closed until new physical evidence changes the diagnosis.
1. Revised causal ranking
1 — ACTION PIPELINE / ACTUATOR DYNAMICS (very high confidence). 97% slew saturation means policy timing is almost entirely replaced by an unseen nonlinear trajectory generator. This can destroy contact phase before friction or state estimation matter.
2 — ZERO-COMMAND / DEPLOYMENT CONTRACT (high; audit immediately). Yaw sweeps, a large hip move, and roll start before forward command. Either the checkpoint was not trained to stand quietly at zero or exported observations/actions differ from simulation.
3 — FOOT-GROUND CONTACT (very high). The known simulated transport mechanism is loaded sliding. High-grip rubber on rough concrete anchors instead, turning slide strokes into body torque.
4 — MISSING BODY-VELOCITY FEEDBACK. Important for sustained control/recovery, but it does not by itself explain aggressive zero-command motion.
5 — TETHER + TIGHTER 10 DEG TRIP. Aggravators that can bias roll direction and shorten time-to-trip, but do not explain the first three discrepancies.
2. The first zero-command second is a separate experiment
A narrow-band forward locomotion checkpoint is not automatically a standstill controller. The campaign itself found that stop/go exposure produced quiet parks and restarts. Therefore immediate marching may be genuine out-of-distribution policy behavior rather than contact failure.
Capture the complete 72-D hardware observation at tick 0, previous-action initialization, normalization constants, and raw 18 policy outputs.
Feed that exact vector through both the exported NumPy MLP and the training implementation. Require numerical agreement.
Initialize MuJoCo from the captured plant pose and run 1–2 s at vx_ref=vy_ref=0 with exact deployment observation substitutions.
If simulation reproduces the sweeps, explicitly train/deploy a zero-command-capable joystick/stop-go lineage.
If simulation does not reproduce them, stop RL and find the implementation mismatch: joint order/sign, affine map, offsets, qdot units, normalization, IMU convention, goal block, previous-action semantics, or export error.
3. Why 97% slew saturation is the dominant immediate problem
The limiter changes the control system, not merely its speed. PPO emits an absolute joint target, while hardware advances the applied target only 1.5 degrees toward it each 40 ms. When almost every joint is saturated, applied action is dominated by the limiter's internal state rather than the policy's current output. A leg expected to have swung may remain loaded; the next action is then conditioned on a phase/state that never occurred.
This gives a simple explanation for monotonic roll: requested stroke → slew-limited incomplete swing → high-grip foot stays anchored → leg motion creates chassis moment → policy advances as if expected motion occurred → saturation persists → roll accumulates. Lack of visible correction is not yet proof the network ignores roll; corrective requests may simply remain hidden behind the limiter.
4. Rate clamp recommendation
Do not choose 37.5 deg/s or the servo's ~300 deg/s nameplate speed from principle. Measure safe loaded closed-loop response first; free-running servo speed is not usable loaded joint speed.
Run supported single-joint and representative multi-joint step/ramp tests over several slew rates, loads, and postures.
Log policy proposed target, post-safety applied target, measured q/qdot, current, supply voltage, temperature, and timestamps.
Validate servo current telemetry against an external supply/shunt at least once; 25 Hz readings can miss transients or be internally filtered.
Choose the deployed slew envelope from tracking error, current/thermal margin, mechanical behavior, and repeatability.
Put that exact stateful slew/saturation transformation into training from the beginning and modestly randomize around the measured envelope.
Do not make loose→tight curriculum the main strategy; it can first teach a gait whose phase timing depends on dynamics later removed.
Keep a modest action-delta term, but also track the saturation residual ||target_proposed - target_applied||. The desired policy should stop using the safety limiter as its trajectory generator.
5. Velocity observation
v_meas := v_ref is appropriate for a deployment-equivalence test of today's exact stack. It is not the preferred permanent actor interface. For the next hardware-target policy, remove or mask unavailable measured velocity from the actor; let the asymmetric critic retain true simulator velocity.
Later add a real proprioceptive velocity/contact estimator from IMU, joints, and applied-action history. RMA, DreamWaQ, and concurrent state-estimator work support temporal proprioception for hidden dynamics and velocity/contact inference. History expansion is sensible only after realistic actuator dynamics are present; history cannot compensate for an action transformation it never saw.
6. Anti-slip: physics, not another reward sweep
Hardware strengthens the existing diagnosis. A trajectory that generates translation by loaded foot sliding in simulation generates body roll on the real surface. The policy repeatedly paid substantial reward costs rather than abandon sliding, and fresh seeds rediscovered it. This is characteristic of a systematic model exploit.
Do not reduce contact calibration to one friction coefficient. Rubber suction-cup tips on broom-finished concrete may differ in static/dynamic friction ratio, compliance, contact patch, torsional resistance, stick-slip behavior, and normal-force dependence.
Highest-value contact experiment: Run the existing scripted gait that already walks successfully on hardware, record commands/joints/IMU/current/progress, and replay the identical commands in simulation. Tune the plant until the scripted gait has comparable kinematics and energetics. This isolates physics from RL policy quality.
7. Actuator model is now a high-priority build
The project now has the telemetry needed to build an actuator model. Start simple and earn complexity. A low-order identified model may be sufficient for an integrated position servo; if not, fit a learned residual or actuator network.
Baseline: delay + stateful slew + deadband + first/second-order response + load/voltage-dependent speed/torque limits.
Candidate learned residual inputs: recent q/qdot, applied targets, target error, current, voltage, joint identity/posture; outputs: next q/qdot and optionally current/residual torque.
Train on controlled supported trajectories plus the successful scripted gait; hold out entire trajectories/postures.
Judge by multi-step rollout prediction and hardware-vs-sim trajectory/distribution matching, not only one-step MSE.
Prefer the simplest model that closes the mismatch. ANYmal's actuator net is evidence for the approach, not a requirement to copy its exact architecture.
8. Permanent gate: Deployment Equivalence Gate 0
No learned policy should reach hardware unless it passes simulation with the exact deployed control contract.
same controller frequency/timing;
same action map, joint order/signs, offsets, and limits;
same stateful slew/rate/saturation pipeline;
measured actuator delay/dynamics plus plausible variation;
same actor-visible observations — no ideal simulator-only signals;
explicit previous-action semantics: raw proposal versus post-safety applied action;
same 10 degree relative-tilt threshold and relevant safety behavior;
same initial pose/reference convention;
zero-command settle, command ramp, stop/restart, abrupt-command, and supported-load panels;
per-tick proposed target, applied target, and measured joint-state logs.
The old champion would have failed this gate immediately. That makes Gate 0 one of the highest-value outcomes of the first hardware attempt.
9. Additional checks before retraining
Verify whether previous-action observation on hardware is the raw policy proposal or the post-safety applied target. With a stateful limiter this distinction is crucial.
Audit qdot units/filtering. Noisy 25 Hz finite differences can easily leave the training distribution.
Audit episode-start tilt reference and sign conventions exactly.
Run inference at zero command while physically supported; abort on large proposed target changes.
Run inference while applied targets are temporarily frozen at plant stance to see whether observation drift alone causes increasingly extreme proposals.
Remove/slacken the tether when safe, or estimate its force and include a comparable random disturbance in simulation.
Separate three errors in every plot: proposed target → SafetyLayer applied target → measured servo position.
10. Architecture priority after tonight's data
P0 — exact deployment pipeline and deployment-equivalence evaluation.
P0 — identified actuator dynamics; actuator net only if a simpler model is insufficient.
P0 — contact/current calibration using scripted gait, drag tests, planted-load tests, and validated current measurement.
P0 — actor observation cleanup: remove fake v_meas and make applied-action history explicit.
P1 — 8→16→24 frame history after P0 dynamics exist; at 25 Hz these span ~0.32/0.64/0.96 s.
P1 — proprioceptive velocity/contact estimator or RMA/DreamWaQ-style adaptation for residual hidden dynamics.
P1 — teacher/student privileged observations if useful; asymmetric critic already captures part of the benefit.
LATER — on-robot policy-gradient fine-tuning. Avoid while a failed rollout can collapse the robot.
11. Recommended experiment order
1. Exact export/observation audit.
2. Zero-command replay from captured hardware plant pose.
3. Deployment-pipeline eval of existing longdist, stop-go, and best joystick checkpoints. Do not assume historical champion is best hardware candidate.
4. Supported actuator characterization: loaded slew/step ladder, latency, deadband, voltage/load dependence, current validation.
5. Scripted-gait hardware-vs-sim replay.
6. Foot/contact tests: breakaway/static friction, sliding friction, loaded planted-foot micro-motion.
7. Update action/actuator/contact/current models and validate on held-out scripted trajectories before PPO.
8. Retrain a forward/joystick hardware lineage with exact deployment dynamics, actor-visible observations only, zero-command exposure, 10 degree termination, and realistic disturbances.
9. Pass Deployment Equivalence Gate 0 plus frozen multi-seed physical-metric panels.
10. Return to supported hardware: substantial body offload → small steps → progressively unload support → short forward segment → forward joystick.
12. What not to do next
Do not launch another broad anti-slip reward coefficient sweep in the old simulator.
Do not simply raise the hardware clamp to the servo's nominal speed and retry the champion.
Do not train with ideal simulator velocity and substitute command velocity only at deployment.
Do not add longer history before putting actual action/actuator dynamics into training and expect history alone to solve the mismatch.
Do not interpret a 0.3 A peak at 25 Hz as proof there were no important current transients until telemetry is validated.
Do not let quadruped/mirror/robustness side projects delay the P0 sim-to-real repair, though they can continue in spare compute/agent time.
13. Suggested binding RL_PLAN text
HARDWARE TRANSFER RULING (08-09): First walk attempt failed before meaningful gait transfer because the deployed action path was not represented in training (97% joint-ticks slew-saturated; mean proposed-to-applied target gap 48 deg), with additional zero-command and high-grip contact mismatches. Broad six-leg reward/DR search is paused as P0. No policy reaches hardware without Deployment Equivalence Gate 0: exact controller rate, action map, stateful slew/saturation, measured actuator dynamics/latency, deployable actor observations, safety thresholds, and zero/stop/ramp panels. P0 sequence = export/obs audit → actuator/contact/current identification using supported tests + scripted gait → corrected-physics retrain → supported hardware ladder. Anti-slip reward shaping remains CLOSED absent new physical evidence.
14. Directive to the next LLM/agent
Treat the first hardware trace as a deployment-contract and system-identification result, not evidence that PPO locomotion is fundamentally broken.
Preserve the 97% saturation and mean 48 deg proposal/applied gap as the strongest evidence in every summary.
Investigate zero-command behavior before attributing the whole failure to friction.
Make Deployment Equivalence Gate 0 mandatory and test existing stop-go/joystick checkpoints through it before deciding what to retrain.
Collect the smallest safe hardware dataset that identifies actuator and contact quantities that change the simulator's qualitative behavior.
Fit simple physics first; add an actuator net or residual only if it improves held-out multi-step hardware prediction.
Keep anti-slip reward shaping and generic full-DR search closed.
Do not return to an unsupported learned-policy floor test until the corrected policy passes Gate 0 and supported low-energy tests.
15. Research basis
Tan et al., Sim-to-Real: Learning Agile Locomotion for Quadruped Robots, RSS 2018 — system identification, actuator modeling, latency, randomization.
Hwangbo et al., Learning Agile and Dynamic Motor Skills for Legged Robots, Science Robotics 2019 — learned actuator network for ANYmal sim-to-real.
Kumar et al., RMA: Rapid Motor Adaptation for Legged Robots, 2021 — adaptation from proprioceptive/action history to hidden environment/dynamics variables.
Ji et al., Concurrent Training of a Control Policy and a State Estimator for Dynamic and Robust Legged Locomotion, 2022 — concurrent estimates including base velocity/contact-related quantities.
Nahrendra et al., DreamWaQ, ICRA 2023 — implicit terrain/dynamics information from proprioception for robust locomotion.
Hu et al., Impact of Static Friction on Sim2Real in Robotic Reinforcement Learning, 2025 — static joint friction can remain a critical transfer gap even with actuator-net methods.
Dao & Fern, Simulator Adaptation for Sim-to-Real Learning of Legged Locomotion via Proprioceptive Distribution Matching, 2026 — parameter identification/action-delta/residual actuator adaptation from short hardware proprioceptive rollouts.
Assessment: the first failed walk materially de-risks the intellectual problem because it exposed large, measurable, testable mismatches. The fastest path to joystick walking is now better system identification and deployment equivalence, not more policy-search creativity.
