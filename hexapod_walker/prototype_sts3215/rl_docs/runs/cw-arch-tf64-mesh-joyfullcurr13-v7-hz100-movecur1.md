# cw-arch-tf64-mesh-joyfullcurr13-v7-hz100-movecur1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-25T05:43:00+00:00

**pod**: hexapod-mjx-train-7

**steps**: 2000000

**parent**: cw-arch-tf64-mesh-joyfullcurr13-v7-hz100-acq1

**wandb_id**: cslqyj5d

**hypothesis**: Plain English: architecture-replication sibling of movecur1 -- does the same walking-tick current-dwell charge (reward.k_walk_move_current=2.0) that movecur1 tests on the MLP also stop the identical over_current death on the tf64 transformer (cw-arch-tf64-mesh-joyfullcurr13-v7-hz100-acq1, FAIL, same signature: zero tilt_pitch, over_current every DR-0 episode, Imax 2.64-2.70A)? tf64-mesh-acq1 already answered architecture does not matter, both fail identically -- this checks whether the FIX also transfers across architecture, same V7/100Hz/tf64/mesh recipe, single added lever.

**gate**: PASS: training reward improves AND walk_move_current_max_a stays mostly under ~2.2A AND eval/walk/survived_frac shows real nonzero stretches, matching movecur1 own MLP read at matched steps. PARTIAL: over_current frequency drops vs tf64-mesh-acq1 but survived_frac stays mostly 0. FAIL: indistinguishable from tf64-mesh-acq1 at matched steps -- would mean the fix is MLP-specific or transformer capacity/attention interacts badly with this charge, a genuine architecture-dependent finding worth its own dig-in.

**note**: RECONSTRUCTED: original ledger entry was never written (prestage log 05:39:36: pullckpt rc=1 no ledger entry). Reconstructed from rl_docs/runs/cw-arch-tf64-mesh-joyfullcurr13-v7-hz100-movecur1.md + logs/experiments/.../wandb_summary.json + orchestrator.log. extra_args/command not recreated (not needed for verdict); wandb config is authoritative for the actual cfg-set used.

