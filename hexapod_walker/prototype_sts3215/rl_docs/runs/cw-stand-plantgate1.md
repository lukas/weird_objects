# cw-stand-plantgate1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T20:42:45+00:00

**pod**: hexapod-mjx-train-6

**steps**: 2000000

**parent**: cw-stand-b2p1

**wandb_id**: goi0p4j9

**hypothesis**: Plain English: does turning on the new geometric stand-detector (checks feet actually down + body over them + no legs left dangling, not just torso height) stop the walking/stance policy from cheating its way to a fake stand-up? Every rise/lower attempt so far (b2p1, rfix-fresh1, riseproof1) hit the height target with 1-2 legs left up in the air (flag-leg/tripod) because the reward only checked torso height. We just landed a geometric check (PLANT_SPEC/valid_plant) that a flag-leg pose fails and an honest stand passes, proved on a pre-training bank (replay beats the stilt cheat and the freeze exploit by a wide margin), and this run is the FIRST time that check prices the live reward (reward.rise_plant_polygon_gate=1) instead of just the bank. Same warm start (stance champion) and same full reward stack as b2p1 -- the ONE new variable is the polygon gate. DISCOVERY budget (2M steps, binary question) per RESEARCH_RULES -- do not extend this into a hardening run without a video showing genuine feet-down rise/lower first. If-true: video at 2M shows a materially more honest stand (fewer/no flag-leg episodes, valid_plant true on a real fraction of endings) -- worth a hardening budget next. If-false: the SAME flag-leg/tripod cheat persists even with the geometric price on it -- the gate isn't strict enough (or the income structure elsewhere still out-prices honesty) and rise needs a different mechanism, not more steps on this one.

**gate**: DISCOVERY (binary): pull det+sto rise AND lower episodes at 2M, report valid_plant/plant_fail/plant_margin_mm (harness always reports these) alongside posture-strict success. PASS-the-question if video shows genuine feet-down plant (no pad >60mm, no leg-through-floor) on a real fraction of rise AND lower endings, materially better than b2p1's 0/12 baseline -- exact numeric threshold decided at hardening, not here. FAIL-the-question if the same flag-leg/tripod-at-height cheat from b2p1/fresh1/riseproof1 is still visually dominant.

