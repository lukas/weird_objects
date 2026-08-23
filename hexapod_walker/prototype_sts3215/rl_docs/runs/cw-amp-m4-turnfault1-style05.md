# cw-amp-m4-turnfault1-style05

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INFORMATIVE

**created**: 2026-08-23T01:13:08+00:00

**pod**: hexapod-mjx-train-4

**steps**: 2000000

**parent**: cw-amp-m2-turnclone-yawcmd0-r2

**wandb_id**: 5c5rmv82

**hypothesis**: Plain English: M4 fault-sight has only been tested on yaw-blind substrates (forward-only faultobs1, full-heading-no-turn faultobs2). cw-amp-m2-turnclone-yawcmd0-r2 showed a single checkpoint can combine full-circle heading translation AND turn-in-place cleanly. Does the fault_health obs-wiring mechanism (already proven safe+beneficial on two simpler substrates: faultobs1 +18%prog/-27%slip vs blind, faultobs2-headingsfull PASS/neutral-style) still produce a non-crashing, compensating (not statue) walker when grafted onto this turn-capable substrate, or does stacking THREE things at once (heading diversity + turning + faults) break down? Single lever vs faultobs2-headingsfull-style05: --init-from swapped to the turn-capable checkpoint + the matching yaw cfg block (obs-contract requirement so the checkpoint's 75-dim actor obs loads correctly before the +18 fault_health pad), same dr.fault_prob=1.0 / obs.fault_health=1 / --obs-pad-transplant 18 wiring, same amp 0.5/0.5, conservative 2M discovery budget (not 4M acquisition) given the added combination risk.

**gate**: Discovery (2M, DR-0, own cfg). Mechanism-safety bar first (per M4's new-mechanism discipline): gait_valid >=10/12 det+sto, faulted episodes show visible compensation on video (limping, not a sacrificed-leg statue), height_err stays in the walking band the whole run (no crouch), zero NaN/crash. If that bar clears, compare faulted-episode prog_ratio/slip against faultobs2-headingsfull-style05's own numbers (det prog med 1.089/slip med 3.084, sto 0.568/7.258) -- comparable = the combination composes; clearly worse = stacking axes costs fault tolerance, a real finding for a dig-in; also run eval_yaw manually to check turn tracking survived the fault-training on top.

**verdict**: Fails its own pre-registered mechanism-safety bar at 2M discovery: gait_valid 9/12 (6/6 det, only 3/6 sto) vs the >=10/12 bar, 3 sto episodes show a genuine sacrificed-leg statue (legs 2/4/4, video walk_sto_2: near-stationary shuffle, not a limp), progress collapses (det med 0.29, sto med 0.14) well below faultobs2-headingsfull-style05's own numbers (det 1.09/sto 0.57, though that ran at 4M not 2M) and slip is worse too (det 4.76 vs 3.08, sto 8.6 vs 7.26). Training reward is still climbing at budget end (quarters 46->99->170->203, not flat) -- per the 08-21 ruling this is UNDERTRAINED, not a proven-broken mechanism: stacking THREE axes at once (full-heading translation + turn-in-place + faults) on a fresh 2M budget is harder than any pair alone, and each axis individually composed fine on its own dedicated budget elsewhere in this track. Next (named, not spent): either a longer acquisition budget on this exact 3-way combo, or -- cleaner -- add fault AFTER turn+heading/push is acquisition-solid rather than stacking three brand-new axes simultaneously, matching the M2->M3->M4 sequential milestone design.

