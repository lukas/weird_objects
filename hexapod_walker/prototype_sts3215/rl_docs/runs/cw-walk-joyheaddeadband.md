# cw-walk-joyheaddeadband

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T23:42:05+00:00

**pod**: hexapod-mjx-train-1

**steps**: 20000000

**parent**: cw-walk-joyhead90-lat25

**wandb_id**: 4mv6c8mi

**hardware_ready**: False

**hypothesis**: Compose rung off joyhead90-lat25 (widest +-90deg envelope, driving package): completes the deadband-onto-driving gap (see cw-walk-joydeadband, same axis on the narrower +-45deg package) at the wider heading envelope. One variable off joyhead90-lat25: add dr.deadband_scale=1.0,3.0. If-true: JOYSTICK GATE @90 0 falls AND own-cfg (DR0.5+latency+deadband) gv 12/12, 0 term, prog med >=0.80 -- widest envelope also absorbs deadband. If-false: deadband breaks flip recovery specifically at the wider steering angle (in-envelope falls or prog crater) even though the narrower package (joydeadband) held -- envelope width and deadband tolerance interact. Strongest alternative: passes but slip/prog shades vs joyhead90-lat25 baseline.

**gate**: JOYSTICK GATE: eval_drive --dr-scale 0.2 --heading-max-deg 90 -- ZERO in-envelope falls; own-cfg (DR0.5+latency0.5-2.5x+deadband1-3x) det+sto 6/6 @15s: gait_valid 12/12, 0 term, prog_ratio med >=0.80; DR0 retention det 6/6 gv; frames watched det

**verdict**: PASS. Servo-deadband compose (1.0-3.0x) onto joyhead90-lat25 (widest +-90deg envelope + DR0.5 latency): own-cfg det gv 6/6, 0 term, prog med 0.90 (>=0.80 gate), slip 1.51; sto prog 0.93, slip 1.47. JOYSTICK GATE @90deg 0 in-envelope falls (incl flip-stress) -- widest envelope absorbs deadband same as the narrower joydeadband package did. DR0 nominal retention gv 6/6, 0 term (gate's only retention requirement, no numeric slip/prog cap pre-registered for this axis) -- prog 0.91/0.91 det/sto, slip 1.78/1.75 (elevated vs nominal champion but consistent with this driving lineage's own baseline, not a new erosion). Frames (det, own-cfg + retention): six legs cycling, level body, no flag leg, foot-slide persists (contact-pricing root, unchanged). Deadband now confirmed axis-safe at both the narrow and widest heading envelopes. Not hardware-ready (paddling).

