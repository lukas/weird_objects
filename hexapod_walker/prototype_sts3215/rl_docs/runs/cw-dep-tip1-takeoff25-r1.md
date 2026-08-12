# cw-dep-tip1-takeoff25-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: done

**created**: 2026-08-11T23:36:52+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-dep-tip1

**wandb_id**: xg6wrt13

**hardware_ready**: False

**hypothesis**: Make the walking champion survive the 20-25 degree takeoff wobble that knocks the real robot over right after gait start; this arm raises the tipped-start training regime to the tilt range actually measured on hardware. Correct relaunch of cw-dep-tip1-takeoff25 (the drained stub trained default DR — no science). Hardware evidence (camera bench 08-11 eve, bench_blast_20260811_18*/19*): BOTH deployed walkers show a 20-25deg takeoff roll transient; vref1 fell 3/3, tip1 clean 1/1 but tripped 2/3 fwd attended — every fall keeps the takeoff-transient shape. tip1 trained at dose 6-18deg prob 0.30, BELOW the measured regime. One variable vs tip1: dr.tipped_start_prob 0.30->0.5, dr.tipped_start_deg 6-18 -> 12-25 (sim_env dr.* override path verified, tuple parses).

**gate**: SCORE/tipped_recovery_success at 20-25deg injections det+sto materially above the matched parent under the IDENTICAL injection (eval_checkpoint --baseline cw-dep-tip1); DR0 walk retention unchanged (gait_valid 6/6, prog med >= 0.85, no paddle); frames watched det.

**verdict**: INFORMATIVE FAIL / NULL RESULT: raising the static tipped-start dose (prob 0.30->0.5, deg 6-18->12-25) gives zero measurable separation from parent cw-dep-tip1 on its own target metric. Matched-parent probe (forced static tip, det+sto, n=12 each, deg=12/17.5/20 all capped near 17.5 by 0.7x envelope): BOTH policies recover 100% at every dose tested (end_roll ~1.7-1.9 deg, z_drop ~67-69mm, 0 terminations) -- a hard ceiling effect, not an improvement. Clean DR0 walk retention (tipped_start disabled) is genuinely unchanged vs parent: gait_valid 8/8 both, prog_ratio 0.92/1.03 both, delta +0 on eval_checkpoint --baseline. The routine auto-gate/owncfg passes looked bad (det gait_valid 2/6, prog_med 0.43) only because they bake the arm's own harsher tipped_start cfg into the injected distribution -- not a real gait regression. Training-time periodic-eval's apparent tip1=0/2 vs r1=2/2 at 12deg was n=2 noise; replicated at n=12 both are 12/12. Second consecutive run (after tip1's own discovery arm) to find no sim separation on the STATIC tipped-start axis -- CLOSES it. The hardware failure is a DYNAMIC mid-gait roll transient after gait onset; a pre-tilted static spawn does not model that mechanism. tip1 stays the deploy checkpoint; no reason to swap in r1.

