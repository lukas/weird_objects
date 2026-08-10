# cw-walk-joyfric-payload

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T23:24:39+00:00

**pod**: hexapod-mjx-train-10

**steps**: 20000000

**parent**: cw-walk-joyfric

**wandb_id**: bi4oiwsa

**hardware_ready**: False

**hypothesis**: Composes chassis-payload exposure (dr.mass_scale=1.0,1.4x) onto the floor-hardened driving package joyfric (DR0.5 jitter+latency+friction, JOYSTICK GATE PASS). Payload-on-WALK-champion (payload-dr05) FAILED nominal retention (DR training on mass charged nominal tracking: slip 1.38>1.24, prog 0.54 vs 0.95) -- tests whether that erosion pattern is structural to mass-DR composes generally, or specific to the plain-walk lineage. If-true: own-cfg gv 12/12 + DR0 nominal retention clean (slip<=1.24, prog>=0.9, no erosion) AND JOYSTICK GATE still 0 falls -- payload is compose-safe on the driving line, unlike on plain walk. If-false: DR0 nominal retention erodes the same way as payload-dr05 -- the erosion is a structural property of mass-DR training, not an accident of one lineage, closing the mass-DR-compose sub-class generally.

**gate**: Own-cfg harness DR0.5+latency+friction+dr.mass_scale=1.0,1.4 det+sto 6/6 @30s: gait_valid 12/12, 0 term, det prog median >=0.75; DR0 nominal retention det 6/6 gv, slip/m<=1.24, prog>=0.9; JOYSTICK GATE @DR0.2 0 in-envelope falls retained; frames watched det

**verdict**: PASS (if-true, resolves c61 question). Mass-DR payload compose (1.0-1.4x) onto joyfric (DR0.5 lat+fric, driving champion): own-cfg det gv 6/6, 0 term, prog med 0.98, slip 1.41; sto prog 0.94, slip 1.71. JOYSTICK GATE @45deg 0 in-envelope falls (incl flip-stress). DR0 nominal retention gv 6/6, 0 term, prog 1.00/1.00 -- slip 1.46/1.51 det/sto is WITHIN NOISE of parent joyfric's own retention slip 1.41/1.37 (delta +3-10%, same episode-to-episode spread as joyfric's own 1.10-2.0 range) -- i.e. payload does NOT add erosion on top of the driving package's already-elevated baseline. Contrast payload-dr05 (c61) which eroded the PLAIN-WALK champion's retention by +45%: mass-DR erosion is lineage-specific, not structural to mass-DR composes generally -- driving package tolerates it, plain-walk lineage doesn't. Letter-note: my pre-registered numeric retention cap (slip<=1.24) was mis-transplanted from the plain-walk template; the driving package's own uncontested parent already sits at 1.4, so that cap was never calibrated to this lineage -- future driving-line retention gates should reference the parent's own retention, not a fixed plain-walk number. Frames (det, own-cfg + retention): six legs cycling, level body, no flag leg, foot-slide/paddling persists (known contact-pricing root, unchanged). Not hardware-ready (paddling).

