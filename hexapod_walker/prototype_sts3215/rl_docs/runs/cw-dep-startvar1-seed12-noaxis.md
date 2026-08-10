# cw-dep-startvar1-seed12-noaxis

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T20:03:49+00:00

**pod**: hexapod-mjx-train-10

**steps**: 18000000

**parent**: cw-dep-startvar1-placementonly

**wandb_id**: 81kgr7y3

**hypothesis**: Plain English: today's startvar1 failure line found that seed 12 (not any specific extra DR axis) might be the real reason 2/6 episodes degrade -- the noZDnoBS1 and placementonly isolation arms both showed the SAME det/3+det/4 degradation regardless of which axis was zeroed, but that pattern was ABSENT in the seed-11 standalone placement compose. This arm removes the LAST extra axis (placement_noise_deg 6->0), leaving nothing but seed=12 + the plain vref1-r1 recipe (18M steps, same warm start/eval panel as every isolation arm tonight). If-true: det/3+det/4 STILL degrade with zero extra axes -- confirms seed=12 itself (not any dep-line axis) drives this pattern, closing the startvar1 forensic question and clearing every already-PASSed axis of blame. If-false: seed=12 alone is clean (matches vref1-r1's own seed-11 band) -- an axis (or an axis x seed interaction) is still required, reopening the axis search.

**gate**: own-cfg (DR0.35, zero extra axes, seed=12) det+sto @15s: report whether det/3 and det/4 reproduce the isolation-arm fingerprint (prog <=0.8, slip >=1.5) or land clean like vref1-r1's own seed-11 band (slip 0.89-1.13 det); DR0 retention; frames watched det/3,4

