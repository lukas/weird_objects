# cw-stand-holdstill2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: DONE

**created**: 2026-08-11T06:39:08+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-stand-holdstill1

**wandb_id**: c4jqtav7

**hardware_ready**: False

**hypothesis**: Give the robot a slope to follow back to a quiet stand: holdstill1 proved that paying ZERO for parking a leg in the air is not enough — with everything nearby also earning zero, the learner never found out which way to move the parked leg and kept it parked for the whole run. This arm is identical except the ONE change reward.hold_flag_fade=1.0: instead of a hard cutoff, income now ramps up smoothly as the parked foot comes down below 12 cm (the observed 11 cm park earns 14% of the quiet stand's pay, with every centimeter lower paying more), so lowering the leg is always downhill-in-reward. Tests whether restoring the gradient lets the policy learn a quiet six-foot hold without losing its honest stand-up.

**gate**: Harness at 2M: hold-mode det episodes end quiet valid plant — worst-foot end_clear_mm < 20 and swing_count <= 2 per 15 s on >= 4/6 det (holdstill1: 0/6, leg parked 107-116mm, 12-23 swings) — AND rise retention det >= 3/6 with zero flag-leg on video (holdstill1 kept det 4/6 sto 6/6). Mechanism health: env/hold_feet_factor must LEAVE the 0.03-0.2 plateau band (holdstill1's whole-run signature); if it is still < 0.2 past 1M with hold income flat, the fade hypothesis is refuted and the next lever is BC supervision on hold ticks, not another pricing variant.

**verdict**: FAIL on the gate (hold 0/12 det+sto) but the fade mechanism is directionally confirmed and NOT flatly refuted: the parked leg came down 107-116mm -> 86-101mm (into the fade band), train env/hold_feet_factor tripled (0.1 -> 0.19-0.35, rising at end), track has episodes down to 29-56mm; video confirms a level stance with one front leg curled and 3 legs still cycling. Rise retention det 4/6 sto 4/6 (parent band 3/6). Decision: two pricing-lever misses in a row (hard zero, fade) = change the hypothesis, not the step count; discovery-phase rules forbid extending a run whose target behavior has never been seen. Pricing stays landed (it is correct: bank-proven quiet>stepping>park); next lever is BC-style supervision on hold ticks (target = the episode start pose - the bc_anchor mechanism already validated on rise), a SPEC/CODE item before any further stand-line launch.

