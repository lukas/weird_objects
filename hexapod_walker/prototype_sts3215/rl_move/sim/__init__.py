"""MuJoCo sim twin of the Phase-1 balance stack.

Modules
-------
servo_model     measured STS3215 params + profiled-target actuator emulation
fit_motor_model fit sim actuator params from logs/motor_model.json
sim_env         SimHexapodBalanceEnv — same obs/action/reward as hardware env
domain_rand     per-episode domain randomization
replay_compare  hardware battery CSV vs calibrated sim, per-metric deltas
train_ppo_sim   PPO harness (stable-baselines3) for the sim env
"""
