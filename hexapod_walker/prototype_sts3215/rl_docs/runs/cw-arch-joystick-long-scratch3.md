# cw-arch-joystick-long-scratch3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: DONE

**created**: 2026-08-15T22:29:48+00:00

**pod**: hexapod-mjx-train-3

**steps**: 40000000

**wandb_id**: a7rtr3lq

**hardware_ready**: False

**hypothesis**: From random weights, learn durable joystick walking over 60-second sessions. Each PPO rollout spans a full session and each episode contains many instantaneous, irregular command changes drawn from random headings, 180-degree back-and-forth, square, stop-go, circle, and local-jitter schedules. A walk-only 90 mm chassis-drop termination closes the seated-scooting loophole and charges the remaining-session fall cost.

**gate**: Fresh initialization with no init checkpoint. Final C-MuJoCo gate requires at least 5 of 6 deterministic and 4 of 6 stochastic trials to survive the full 60 seconds; mean direction error at most 20 degrees, p90 at most 45 degrees, wrong-direction fraction at most 0.10, velocity error at most 0.02 m/s, no walk_low_height termination, and video must show upright controlled responses across multiple instantaneous command switches.

**verdict**: FAIL (final; operator-ratified fb_20260817T005114) — 40M from-scratch joystick run collapsed after its best optimization point: reward/tick EMA peaked -2.16 @14.7M and ended -12.45, ep len ~1204 -> ~105 ticks, late updates approx_kl ~0.24 / clip ~0.40 despite target_kl=0.02, critic explained variance ~0 THROUGHOUT (frozen-rollout audit test_value_learning.py later proved the critic CAN fit returns — the unbounded -730 remaining-horizon terminal cliff blocked value learning, not a code bug). DR0 gate + own-DR 0/6 det + 0/6 sto; the 90mm low-height cutoff let it live 40-77mm low (walk_height_gate was OFF). DO NOT USE the final checkpoint; the 14-16M region was merely least-bad. Successor: cw-arch-joystick-canary1 (2.5M gated canary, redesigned update path/terminal cost/height gate/curriculum); promotion requires EV>0 rising + KL/clip health + ep-len stability + v_along/dir_err improving + video stepping — never reward alone.

