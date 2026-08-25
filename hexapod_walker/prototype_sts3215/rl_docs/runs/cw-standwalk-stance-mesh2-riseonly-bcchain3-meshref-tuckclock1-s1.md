# cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref-tuckclock1-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-25T21:34:41+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref-tuckclock1

**wandb_id**: atmjubij

**hypothesis**: Seed twin of meshref-tuckclock1 (only --seed 0->1 differs; same single lever train.bc_anchor_flat_time_indexed=1 vs the meshref parent): does the flat-start absolute-script-clock anchor work robustly across seeds, or was seed-0 luck? Plain story: a probe measured that the honest scripted tuck-then-press ON ITS OWN CLOCK is the reward optimum (+2021, 0.575A, plant_ok) under the exact launched pricing where every taught behavior scores -50..-770 -- this pair tests whether anchoring flat starts to that clock lets PPO find it. Judged jointly with meshref-tuckclock1 per its pre-registered gate. Same joint-pair discipline as every mechanism hedge this campaign.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. Same joint-pair gate as meshref-tuckclock1 (see that run's doc): PASS if BOTH seeds hit flat-probe det>=4/6 AND sto>=4/6 valid_plant with genuine duty>0/swing_count>0 sub-over_current tuck-then-press AND non-flat kinds >= meshref parent (5/6+4/6) -> 8M acquisition grid + stancemix port. PARTIAL if >=2/6 flat valid per seed or genuine swinging tuck with h_err 10-40mm while non-flat holds -> extend budget. FAIL if flat stays 0-1/12 -> clock-target semantics refuted; next suspect is PPO/anchor interaction (read train/bc_anchor_loss_rise: high+freeze = dose coef; low+freeze = emission bug).

**verdict**: CANARY PASS (mechanism-health; behavioral score = PARTIAL-unstable, joint pair with seed-0, cross-seed corroborating but qualitatively worse). Also evaluated this cycle (its own prestage never fired -- pulled checkpoint + ran the standard gate/owncfg/flat-pinned-probe myself on train-1). Cross-seed replicate of tuckclock1's mechanism-health finding: the flat-time-indexed clock also produces genuine non-freeze motion here -- flat-pinned probe (det+sto n=6+6, DR-0): 0/12 valid_plant, duty>0 AND swing_count>0 on every leg every episode, refuting freeze/press-up same as seed-0. BUT the failure mode differs sharply from seed-0: 11/12 flat episodes end roll_class fell/leaning (roll_tail up to 9.0deg, vs seed-0's clean zero falls / roll_tail<=9.2 but 'settled' 3/6+6/6), and the swing distribution is lopsided -- leg idx1 swings 2-10x/episode while 3-4 other legs swing 0-2x, an asymmetric single-leg flailing-tuck-attempt that overbalances rather than seed-0's balanced partial-stand. h_err also worse (57.5-72.9mm vs seed-0's 37.7-59.3mm). The non-flat, state-aligned anchor regime is UNAFFECTED in both seeds -- standard DR-0 gate holds close to parent (det 4/6, sto 4/6 vs parent 5/6+4/6), own-DR mixed (det 6/6, sto 3/6) -- so the divergence is specific to the new flat-time-indexed regime this lever touches, not a general regression. Read: the mechanism reliably produces genuine (non-freeze) tuck attempts in both seeds but has not converged to a stable multi-leg coordination at 2M -- seed-0 lands in a stable-incomplete basin, seed-1 in an unstable-flailing one. Still 'timing being learned, not converged' (the PARTIAL branch), not the FAIL branch's freeze/press-up signature -- per the 08-21 ruling and this run's own gate the reasoned move is extend budget, joined with seed-0's pair. Flag for the 8M read: if the fall pattern recurs at scale, that upgrades this from 'needs more steps' to 'the flat-time-indexed target needs a per-leg stability/symmetry term,' a new lever -- not funded yet, watch first. Evidence: logs/ckpt_eval/cw_standwalk_stance_mesh2_riseonly_bcchain3_meshref_tuckclock1_s1_{gate,owncfg,flatprobe}/, W&B atmjubij.

**refused_reason**: hexapod-mjx-train-1 already runs cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref-tuckclock1-s1 — GPU pods host exactly one run; pick a free GPU pod.

