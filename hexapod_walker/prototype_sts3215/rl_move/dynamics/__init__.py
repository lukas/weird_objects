"""dynamics — self-supervised dynamics-representation pretraining (track: dynrep).

Learn a task-independent latent representation of the hexapod's body
dynamics from saved simulator rollouts (action-conditioned multi-horizon
prediction), then test whether PPO reusing that representation learns
new motor skills faster than PPO from scratch.

Design doc: rl_docs/DYNREP.md. Pipeline: collect -> train -> eval_model.
"""
