# cw-stand-footlow2-hard1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-12T12:46:46+00:00

**pod**: hexapod-mjx-train-0

**steps**: 10000000

**parent**: cw-stand-footlow2-r1

**wandb_id**: ttkmoi6f

**hardware_ready**: False

**hypothesis**: Consolidate the first policy that stands up from flat, holds a quiet six-foot stand, and sits back down cleanly all at once; this run tests whether 10M steps of the identical footlow2 recipe (height-floor rise anchor + foot-height hold anchor + lower anchor, stratified) hardens all three modes into a deployable stance candidate without re-opening any rise/hold/lower seesaw. Prediction-if-true: all cold-start det rises stay clean (<=5mm short, level), hold keeps six commanded-planted feet (and the 0.9mm foot-1 hover shrinks or at worst persists), lower keeps >=10/12 — and the checkpoint passes the session gate, making it the champion-replacement candidate. Prediction-if-false: some mode erodes with steps (the historical hardening-pressure failure), naming which anchor loses the gradient war at scale.

**gate**: Judge on the gate eval WITH the new rsi start_kind labels (snapshot da367c9). PASS if: all cold-start (flat/bridge/crouch, non-rsi) det rises are valid_plant with h_err<=5mm; det hold has NO real park (no foot with duty<0.5 AND end_clear>2mm; report per-foot duty + commanded hover, duty>=0.5 all six is the target and a <=2mm command-hover residual is a recorded cosmetic miss); det+sto lower >=10/12 valid_plant; AND rl_move.sim.eval_session hard gates (falls/rise/sit) pass on the checkpoint. FAIL if any cold det rise stalls >5mm, a real park (>2mm hover or weight-shed) appears, lower <10/12, or eval_session hard-fails. Quote roll_tail/drag/slip vs cw-stand-footlow2-r1 in the verdict.

**verdict**: PASS — first stance checkpoint clean on rise+hold+lower AND the eval_session hard gates at once. All 4 gate clauses hold: (1) cold det rises (bridge 2/2, crouch 1/1 from the draw; flat confirmed via a targeted 12-ep probe since the standard 6-ep draw sampled none: 12/12 valid_plant, h_err 0.5-3.4mm, roll_tail 0.0deg) all <=5mm; (2) det hold has zero real park, all six feet duty 0.95-0.99 with ~0.1-0.2mm commanded hover (tighter than r1's own 0.9mm cosmetic residual); (3) lower 12/12 det+sto, feet flush (end_clear <=0.3mm det, <=6.4mm sto), no outrigger; (4) eval_session hard gates (no_falls/rise/sit_descends) all PASS, and rise clears the interactive ramp to full 148mm by t=9.5s -- notably BEYOND the deployed holdbc1_hard1's 55mm stall on the identical protocol. Visual-quality stats vs parent (footlow2-r1) are flat-to-improved, not worse: det hold drag 136mm/roll_tail 0.1deg (r1 137mm/0.1deg), det lower drag 244mm (r1 306mm), sto hold drag 1283mm/roll_tail 0.8deg (r1 1337mm/0.9deg); det rise drag/roll_tail (434mm/5.1deg vs r1 412mm/4.8deg) is within noise and driven by the single rsi-perturbed episode, not a cold-start regression. Video-confirmed on hold/rise/lower det strips and the flat-probe strip: six-foot planted stance, clean crouch-to-stand rise, no flag-leg/park/stilt anywhere.

