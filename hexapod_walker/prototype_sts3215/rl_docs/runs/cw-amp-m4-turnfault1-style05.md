# cw-amp-m4-turnfault1-style05

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T01:13:08+00:00

**pod**: hexapod-mjx-train-4

**steps**: 2000000

**parent**: cw-amp-m2-turnclone-yawcmd0-r2

**wandb_id**: 5c5rmv82

**hypothesis**: Plain English: M4 fault-sight has only been tested on yaw-blind substrates (forward-only faultobs1, full-heading-no-turn faultobs2). cw-amp-m2-turnclone-yawcmd0-r2 showed a single checkpoint can combine full-circle heading translation AND turn-in-place cleanly. Does the fault_health obs-wiring mechanism (already proven safe+beneficial on two simpler substrates: faultobs1 +18%prog/-27%slip vs blind, faultobs2-headingsfull PASS/neutral-style) still produce a non-crashing, compensating (not statue) walker when grafted onto this turn-capable substrate, or does stacking THREE things at once (heading diversity + turning + faults) break down? Single lever vs faultobs2-headingsfull-style05: --init-from swapped to the turn-capable checkpoint + the matching yaw cfg block (obs-contract requirement so the checkpoint's 75-dim actor obs loads correctly before the +18 fault_health pad), same dr.fault_prob=1.0 / obs.fault_health=1 / --obs-pad-transplant 18 wiring, same amp 0.5/0.5, conservative 2M discovery budget (not 4M acquisition) given the added combination risk.

**gate**: Discovery (2M, DR-0, own cfg). Mechanism-safety bar first (per M4's new-mechanism discipline): gait_valid >=10/12 det+sto, faulted episodes show visible compensation on video (limping, not a sacrificed-leg statue), height_err stays in the walking band the whole run (no crouch), zero NaN/crash. If that bar clears, compare faulted-episode prog_ratio/slip against faultobs2-headingsfull-style05's own numbers (det prog med 1.089/slip med 3.084, sto 0.568/7.258) -- comparable = the combination composes; clearly worse = stacking axes costs fault tolerance, a real finding for a dig-in; also run eval_yaw manually to check turn tracking survived the fault-training on top.

