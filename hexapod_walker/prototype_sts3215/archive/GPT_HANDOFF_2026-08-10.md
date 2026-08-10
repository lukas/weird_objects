Hexapod RL — Cursor Handoff After First Hardware Sessions
External review / operating recommendations | 2026-08-10
Executive judgment
The project is closer to a useful second hardware attempt than it looked after the first failed walk. Two apparently fundamental problems have been reclassified as train/deploy contract and calibration issues: the 1.5 deg/tick slew limiter was already present in training, and the real scripted tripod gait proves the robot can walk on the actual floor. The strongest new finding is that a working real gait naturally rocks roughly +/-10–20 deg, while the RL line trained and deployed with a 10 deg relative-tilt kill. That termination rule may have been suppressing the weight transfer needed for real locomotion and could help explain why the learned policies converged toward low-amplitude creep.
Recommended pivot: The center of gravity should now be: deployment-contract validation -&gt; deterministic liftoff reproduction -&gt; loaded actuator identification -&gt; scripted-gait real-to-sim calibration -&gt; corrected-policy supported hardware attempt. Broad anti-slip reward search and generic full-DR search remain low-value.
1. Revised P0 ordering
P0-A — Deployment-contract validation: exact meas:=ref velocity contract, 25 deg walk tilt envelope, previous-action semantics audit, varied-start/logical-zero panel, and exact initialization/reference handling.
P0-B — Deterministic stance liftoff probe: the stance champion collapses in +roll at the same load-transfer moment on two real runs. This is currently the cleanest reproducible sim-to-real failure and should be treated as a system-identification fixture.
P0-C — Loaded actuator dynamics: characterize the observed 110–210 ms command-to-first-motion delay and 260–330 ms settling under realistic load. Compare directly against the air-fitted motor model.
P0-D — Scripted-gait real-to-sim calibration: start now with q/qdot, IMU and per-servo current; add measured distance/slip immediately when available.
P0-E — Corrected-policy supported hardware attempt after Gate 0 passes.
2. Tilt envelope
Keep the wider 25 deg angle envelope for walking. The scripted gait is direct hardware evidence that +/-10–20 deg body rocking can be normal rather than pathological. Do not replace the angle trip with a pure gyro trip. Use both: a wider angle boundary for legitimate weight transfer, plus an angular-rate condition for rapidly diverging motion.
Recommended safety logic: retain a hard angle ceiling near the validated walking envelope; add an earlier intervention when |roll_rate| or |pitch_rate| is large AND the sign is carrying the body farther away from level.
Avoid a rule where every high gyro value trips, because a legitimate gait may contain fast weight transfer.
Longer term, recovery/intervention is preferable to simply cutting torque, because going limp can worsen a fall. For the immediate supervised ladder, angle + rate is enough.
3. Velocity observation and temporal estimator
If cw-dep-vref1-r1 shows no meaningful erosion under meas:=ref, do NOT block hardware attempt #2 on a proprioceptive velocity estimator or temporal actor. Contract-exact is sufficient for the next supported experiment. Keep estimator/history work high priority, but make it solve demonstrated residual hidden-state problems rather than becoming a default prerequisite.
For attempt #2, actor observations should exactly match the deployed runner.
If the contract-exact warm-start retains behavior, that is evidence the champion did not materially depend on privileged vx/vy feedback.
If it erodes, immediately promote estimator/history work to P0.
History 16/24 remains attractive for latency, contact and hidden-dynamics inference, especially now that the infrastructure blocker is fixed.
4. Contact calibration
Start scripted-gait replay calibration now; do not wait for the tape-measure distance result. Matching joint motion, current and tilt already constrains actuator response, support timing and body dynamics. However, do NOT aggressively fit friction from current+tilt alone because a simulator can match rocking while getting translation/slip badly wrong.
Stage 1: replay the exact known-working scripted commands in sim and match joint trajectories, timing, tilt/gyro patterns and current/load distribution.
Stage 2: measure real distance/speed and, if practical, mark feet or use video to estimate loaded slip. Use this to identify contact/friction behavior.
The real gait visibly contains micro-slip, so the corrected contact model must allow loaded sliding; 'rubber on concrete = no slip' is now known to be too simple.
Calibrate static vs dynamic friction/contact behavior, not just a single nominal mu, if the scripted gait cannot be matched with a simple Coulomb model.
5. Current pricing
Temporarily demote or remove walk effort/current penalties until the mapping is calibrated. Hardware shows total standing hold current around 0.59 A while walking averages roughly 0.33–0.45 A, which is the opposite of assumptions used in earlier reward reasoning. That is strong evidence that the present simulator's effort economics should not be trusted.
Do not immediately retune the coefficient from aggregate bus-current ratios; total current is not identical to the sim's per-joint effort proxy.
Use hardware traces to fit the relative economics of support, swing, loaded lowering, and movement before reintroducing strong walk-effort shaping.
For the new deployment-contract/tilt arms, prefer minimal effort shaping over confidently encoding the wrong ranking again.
6. Deterministic stance liftoff collapse
The repeated +roll collapse at the same belly-liftoff moment is now the best calibration case in the project. Reproduce it from the captured state under the exact deployment contract before doing broad walking diagnosis.
Replay a short window around liftoff from the exact real q/qdot, IMU reference, applied previous action and policy state.
Compare proposed action -&gt; post-SafetyLayer target -&gt; measured q/qdot -&gt; IMU -&gt; per-servo current.
Find the FIRST observable that diverges between sim and hardware, not just the final roll.
If sim stays level while hardware rolls, inspect side-to-side loaded actuator lag, foot unload timing and support-force asymmetry.
The air-fitted servo model is a prime suspect because measured loaded first-motion latency is 110–210 ms. A controller can shift weight under the assumption a support joint moved when hardware has barely started moving.
7. Start-state and logical-zero robustness
The successful scripted gait exposed a major process lesson: start-state fidelity is a first-class sim-to-real variable. The exact same gait looked broken when launched from stale/slumped logical stance and worked after fresh set_zero -&gt; stand.
Keep the planned placement-noise + bad-start composition.
Implement the new logical-zero-drift observation-side DR axis: sensors consistently offset by a few degrees while the physical robot is unchanged.
Hardware-candidate Gate 0 must include varied-start, slumped/park-bank, and logical-zero-drift panels.
Where possible, fix initialization deterministically in the runner too: re-anchor episode pose/IMU reference after a verified stand/settle sequence rather than asking PPO to absorb every software-reference error.
8. Multi-skill erosion
Treat the 50/40/10 quad/walk/hold regression as negative transfer, not as evidence the quadruped skill is bad. Continue the dose-response ladder, but add explicit specialist preservation if walk retention moves again.
First map the retention frontier with lower quad exposure, as already running.
If erosion persists, add a walk-only KL/action imitation anchor to the frozen walk champion on walk-mode states.
Do not force one monolithic actor to own every skill if that repeatedly damages the deployment-critical gait.
A skill-conditioned specialist set, shared backbone with skill heads, or distillation-based consolidation is acceptable. The current objective is reliable hardware walking, not architectural purity.
9. Posetrack
1/12 after +15M steps with a kernel reward looks like poor task shaping rather than insufficient steps. Stop extending the same run.
Use a curriculum over pose delta and target duration: start near the target with tiny offsets, require short holds, then expand displacement and goal changes.
Use dense grouped errors rather than one narrow pose kernel: joint/reference error, body roll/pitch, body height, and contact/foot consistency where applicable.
Gate progress at each curriculum rung before widening the target distribution.
If posetrack is not needed for the next hardware ladder, keep it a background line rather than P0.
10. Interpretation of DR results
Twelve-for-twelve single-axis DR passes and successful pair compositions demonstrate robustness around the simulator's parameterization; they do NOT prove the nominal simulator is physically correct. A policy can tolerate huge randomized ranges while still exploiting a systematic bias in contact, current economics, initialization, or actuator dynamics.
Continue pair composition only when it protects an actual hardware candidate or named deployment corner.
Do not let DR breadth substitute for real-to-sim calibration.
The new contract arms and liftoff reproduction have higher information value than another generic robustness axis.
11. Interpretation of cw-dep-fresh1
Treat cw-dep-fresh1 as unusually important. If allowing 25 deg body motion under the real deployment observation contract produces a visibly higher-amplitude tripod/weight-transfer gait rather than creep, take that qualitative change seriously even if some legacy scalar metrics initially worsen.
The campaign may have spent many cycles optimizing under an accidental constraint equivalent to: 'walk, but never execute the body motion the real robot uses to walk.' The 10 -&gt; 25 deg termination change could therefore be more important than another reward modification.
12. Gate 0 additions
Exact actor observation contract, including meas:=ref until a real estimator exists.
Exact previous-action semantics — audit whether training/history uses raw proposal or post-safety applied action.
Walk termination envelope consistent with measured real gait; include angle + rate safety evaluation.
Varied-start / bad-start / logical-zero-drift panel.
Fresh-reference initialization panel after set_zero -&gt; stand.
Scripted-gait plant-calibration check when simulator parameters change.
Supported stance-liftoff reproduction panel using the deterministic real failure trace.
No hardware promotion solely from DR success if these deployment-equivalence panels fail.
13. Cursor action list
1. Finish and verdict cw-dep-vref1-r1 and cw-dep-fresh1 before inventing new walk reward arms.
2. Audit prev-action semantics end-to-end: training env observation, SafetyLayer, exported runner and hardware logs.
3. Build a replay fixture for the deterministic stance liftoff trace and identify the first sim-vs-real divergence.
4. Compare the current air-fitted actuator model to loaded 2/5/10 deg step responses; fit the minimum additional load-dependent latency/response model needed.
5. Build scripted-gait replay calibration tooling now using q/qdot + IMU + current; leave distance/slip as an explicit missing calibration target and add it as soon as measured.
6. Reduce/remove strong walk effort penalties on hardware-target arms until current economics are calibrated.
7. Land logical-zero-drift DR and varied-start Gate 0 panels.
8. Preserve walk while composing quad skill; if the 30/60/10 rung still erodes walk, add walk-mode distillation/KL anchoring.
9. Replace posetrack's current +steps strategy with a small-to-large dense curriculum.
10. Prepare hardware attempt #2 as a supported deployment-contract experiment, not an unsupported floor demo.
14. Binding summary for RL_PLAN
HARDWARE TRANSFER UPDATE (08-10): The first night's evidence revises the prior post-mortem. The 1.5 deg/tick slew limiter was already present in training and is not itself a train/deploy gap. Confirmed gaps are actor velocity semantics (sim measured velocity vs hardware ref copy; contract arms now running), walk tilt envelope (real scripted gait rocks +/-10–20 deg while RL used a 10 deg kill), unaudited previous-action semantics, loaded actuator dynamics, logical-zero/start-state drift, and contact/current economics. P0 order = deployment-contract validation -&gt; deterministic stance-liftoff reproduction -&gt; loaded actuator ID -&gt; scripted-gait real-to-sim calibration -&gt; corrected-policy supported hardware ladder. Broad anti-slip reward search and generic full-DR retraining remain closed as primary moves. If contract-exact warm-start shows no erosion, velocity estimator/history is not a prerequisite for hardware attempt #2.
15. Bottom line
The project is in a better position than after the first failed walk. The real robot has now supplied: a known working gait, measured dynamic tilt requirements, inverted current economics, loaded servo timing, a reproducible stance-transfer failure, and a concrete start-state failure mode. Those are exactly the measurements needed to turn sim-to-real from broad policy search into a sequence of falsifiable calibration problems.
