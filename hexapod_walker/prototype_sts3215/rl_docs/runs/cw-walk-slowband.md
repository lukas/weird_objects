# cw-walk-slowband

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: KILLED

**created**: 2026-08-09T16:55:39+00:00

**pod**: hexapod-mjx-train-7

**steps**: 20000000

**parent**: cw-walk-wander-dr05

**wandb_id**: 5wokiwk2

**hypothesis**: OPERATOR WISHLIST 8b (operator-tunable speed), the achievable half: joystick driving needs a runtime speed knob, and gentle stick = slow walk. The fast band 0.08-0.12 is REFUTED (cw-walk-fast: gait-limited, pricing root), but the slow side 0.02-0.05 is untested - and pose-track showed slow-command OVERSPEED (prog 1.76-2.22), so tracking below the trained band is a real open question. One variable off the steering-DR champion wander-dr05: widen goal.walk_speed band from 0.05-0.06 to 0.02-0.06 (resampler already redraws speed each 5s segment). If-true: prog_ratio stays in the 0.75-1.25 ruling band across the widened range incl. slow-only sub-band eval - speed becomes a runtime command. If-false: the policy floors at its ~0.05-0.065 gait speed regardless of command (slow-cmd overspeed, prog_ratio >1.25) - the speed knob is blocked on the same contact-pricing root as fast walking. Strongest alternative: slow commands get parked instead of tracked (prog near 0, park frames).

**gate**: Own-cfg DR0.5 det+sto 6/6 resampled cmds: gv 12/12, 0 term, prog_ratio med 0.75-1.25, slip/m med <=2.4; PLUS slow-only sub-band eval (walk_speed 0.02-0.035) det 6/6 prog_ratio med 0.75-1.25; DR0 det retention gv 6/6; frames watched det

**verdict**: No verdict on hypothesis — KILLED at ~3M/20M by its own cycle: duplicate of cw-walk-speedband-r1 (FINISHED, FAIL near-miss, "no requeue"), which already answered the achievable 0.02-0.06 band off this gait family: slow speeds ARE commandable coarsely (prog ~1.1, no parking) and PRECISION tracking is closed on the contact-pricing root. Queued because my dedupe scan missed the speedband* names; parent/DR differences (wander-dr05@DR0.5 vs anchorgate@DR0) do not change the refuted question. 8b speed-knob line stays CLOSED pending operator contact-pricing calibration.

