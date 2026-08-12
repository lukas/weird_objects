# cw-stand-footz1-hard1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-12T09:36:06+00:00

**pod**: hexapod-mjx-train-0

**steps**: 10000000

**parent**: cw-stand-footz1-r1

**wandb_id**: nd3u9txh

**hypothesis**: Teach the standing robot to plant all six feet AND hold that for longer, harder training -- does the just-landed foot-height BC anchor (which fixed the invisible one-foot park in 2M steps of discovery) keep working, and does the small rise regression clear, when hardened to 10M steps the same way bc1->bc1-hard1 and holdbc1->holdbc1-hard1 both consolidated cleanly on this identical discovery-then-hardening pattern? One variable vs the discovery run: step count only (2M->10M), same recipe, same warm start (holdbc1_hard1), same foot-z anchor.

**gate**: PASS if det hold: every foot duty >=0.5 in all 6 episodes (no regression from discovery) AND hold det+sto valid_plant >=10/12 (matching or beating discovery's 10/12, no new park) AND det rise >=6/6 valid_plant with no new falls (recovering the flat-start miss) AND det lower >= matched-parent probe (4/6 baseline) with no NEW outrigger pattern beyond the inherited one AND drag_m/roll_tail quoted vs both the frozen parent and the footz1-r1 discovery run (no worsening beyond noise). FAIL if any foot parks (duty <0.5 det hold) or any of the above regresses vs the discovery run; record train/bc_anchor_footz_loss trajectory either way (does it converge further with more steps).

