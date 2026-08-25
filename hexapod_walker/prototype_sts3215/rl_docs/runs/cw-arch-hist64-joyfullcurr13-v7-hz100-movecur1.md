# cw-arch-hist64-joyfullcurr13-v7-hz100-movecur1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-25T04:59:05+00:00

**pod**: hexapod-mjx-train-4

**steps**: 2000000

**parent**: cw-arch-hist64-joyfullcurr13-v7-hz100-gaitgate-scratch1

**wandb_id**: oo9ts5ha

**hypothesis**: Plain English: does directly pricing sustained per-servo current on WALKING ticks (reward.k_walk_move_current=2.0, bank-proven this cycle: test_walk_move_current.py, honest cycling gait beats a locked-leg fight by >40 reward and beats a stall by >10) stop the mesh-family over_current death that made BOTH the MLP (hist64-mesh-acq1) and transformer (tf64-mesh-acq1) 40M acquisitions fail identically (zero tilt_pitch, over_current every DR-0 episode, Imax pinned 2.64-2.70A, rigid 3-legs-locked/3-legs-airborne duty pattern)? This is the single-lever direct-mechanism test: SAME V7/100Hz/hist64/mesh recipe as gaitgate-scratch1 (which tried the OTHER anti-sacrifice lever, walk_gait_gate, alone -- and made the held-out fall rate WORSE, 39/48 vs the ungated 12/48) but with walk_gait_gate turned back OFF and k_walk_move_current on instead.

**gate**: PASS: training reward improves through the from-scratch valley (matching the mesh valley reference shape) AND per-tick walk_move_current_max_a stays mostly under ~2.2A by 2M (no chronic near-2.64A plateau) AND in-training eval/walk/survived_frac shows real nonzero stretches (not the near-total 0 of every prior mesh arm). PARTIAL: reward improves and over_current frequency measurably drops vs the matched-step gaitgate-scratch1/acq1 trajectories but survived_frac stays mostly 0 -- continuation candidate per 08-21. FAIL: over_current termination rate and Imax profile are statistically indistinguishable from gaitgate-scratch1/hist64-mesh-acq1 at matched steps -- the charge is too weak/mis-thresholded, escalate to a higher k or lower threshold, or the mechanism itself is insufficient and needs a structural fix (e.g. reduce required stance duty via gait-phase shaping).

