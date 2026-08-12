# cw-getup2-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-12T01:56:11+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-getup2

**wandb_id**: dox68fp7

**hypothesis**: Teach the from-scratch unified get-up-and-walk policy to actually stand, by giving it a head start instead of a blank brain -- retry of cw-getup2, which crashed before training on an obs-space mismatch (parent trained on the 68-dim joint_goal task, getup runs on the 72-dim joint_walk task; fixed here with --obs-pad-transplant 4, which zero-pads the policy's first layer for the 4 extra tail dims so it starts bit-identical to the parent). cw-getup1 (fresh init, identical reward) never learned to rise -- it just slumped into a held pose and farmed cheap untangle/load credit while the real stand-quality score stayed near zero the whole run. ONE variable vs cw-getup1: warm-start from the existing rise+hold specialist champion (ppo_goal_cw_stand_holdbc1_hard1) instead of a random network; same getup reward/task/start-mix/goal-mix, same seed. Prediction-if-true: env/getup_S on floor-adjacent starts (crouch/plant/park, 30% of the mix) rises well above cw-getup1's ~0.02 band within 2M steps, and video shows deliberate rises from at least the near-plant start kinds. Prediction-if-false: getup_S stays in the same near-zero band even warm-started -- the specialist's stand skill does not transfer through the getup task's reward/observation wrapper, meaning the barrier is the reward/task wiring, not exploration -- next lever would be a direct getup-mode BC anchor or start-mix reweighting toward crouch/plant. Strongest alternative: the specialist's knowledge only carries over for its own start kinds (crouch/plant/park) and floor-starts (tangle/zero/partial, 70% of the mix) remain exactly as stuck as cw-getup1 -- visible as a bimodal getup_S split by start kind in eval.

**gate**: Same MDP_PREFLIGHT GETUP bank (green, commit fa84a39). PASS if by 2M steps env/getup_S on crouch/plant/park-start episodes rises decisively above cw-getup1's ~0.02 band (target >0.3) AND video shows at least one credible rise-to-stand from a near-plant start with no flag-leg/park exploit; informative FAIL either way (bimodal-by-start-kind vs uniform-stuck) tells us whether the next lever is BC-anchor-on-getup or start-mix reweighting. Also verify at startup that training does not crash and env/getup_S is non-degenerate (not identically 0) at the very first eval, confirming the transplant loaded.

