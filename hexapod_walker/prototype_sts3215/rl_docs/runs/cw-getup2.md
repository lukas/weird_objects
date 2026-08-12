# cw-getup2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-12T01:45:04+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-stand-holdbc1-hard1

**hardware_ready**: False

**hypothesis**: Teach the from-scratch unified get-up-and-walk policy to actually stand, by giving it a head start instead of a blank brain. cw-getup1 (fresh init, identical reward) never learned to rise -- it just slumped into a held pose and farmed cheap untangle/load credit while the real stand-quality score stayed near zero the whole run. ONE variable vs cw-getup1: warm-start from the existing rise+hold specialist champion (ppo_goal_cw_stand_holdbc1_hard1, BC-anchor trained, hardened, knows how to reach and hold a genuine six-foot stand from crouch/flat/bridge starts) instead of a random network; same getup reward/task/start-mix/goal-mix, same seed. Prediction-if-true: env/getup_S on floor-adjacent starts (crouch/plant/park, 30% of the mix) rises well above cw-getup1's ~0.02 band within 2M steps, and video shows deliberate rises from at least the near-plant start kinds. Prediction-if-false: getup_S stays in the same near-zero band even warm-started -- the specialist's stand skill does not transfer through the getup task's reward/observation wrapper (e.g. the reward's staged pipeline or the widened 60 deg fall envelope actively unlearns it), meaning the barrier is the reward/task wiring, not exploration -- next lever would be a direct getup-mode BC anchor (hw track allows it) or start-mix reweighting toward crouch/plant. Strongest alternative: the specialist's knowledge only carries over for its own start kinds (crouch/plant/park) and floor-starts (tangle/zero/partial, 70% of the mix) remain exactly as stuck as cw-getup1 -- visible as a bimodal getup_S split by start kind in eval.

**gate**: Same MDP_PREFLIGHT GETUP bank (green, commit fa84a39). PASS if by 2M steps env/getup_S on crouch/plant/park-start episodes rises decisively above cw-getup1's ~0.02 band (target >0.3) AND video shows at least one credible rise-to-stand from a near-plant start with no flag-leg/park exploit; informative FAIL either way (bimodal-by-start-kind vs uniform-stuck) tells us whether the next lever is BC-anchor-on-getup or start-mix reweighting.

**verdict**: FAILED — crashed at launch, zero training steps, no evidence for/against the warm-start hypothesis. Root cause: observation-space mismatch between the parent (cw-stand-holdbc1-hard1, task=joint_goal, 68-dim obs) and the getup task (task=joint_walk, 72-dim obs: +2 vx/vy_ref in the goal encoding, +2 measured-velocity tail) — SB3's strict check_for_correct_spaces refused to load. Structural, deterministic (always 68 vs 72 for this task pair), and foreseeable from the code (joint_task.py/walk_task.py obs-width formulas) before launch; not a scientific result. Fix is mechanical and already exists in the codebase: --obs-pad-transplant 4 (all 4 extra dims are appended at the tail of the vector — goal9 is an exact prefix match — so the existing zero-pad transplant mechanism applies cleanly). Relaunching corrected respec cw-getup2-r1 with that flag added, same hypothesis, this cycle.

**failed_reason**: run never appeared as 'running' in W&B within 240s

