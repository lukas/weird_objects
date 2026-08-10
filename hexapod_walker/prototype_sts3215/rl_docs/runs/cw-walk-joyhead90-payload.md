# cw-walk-joyhead90-payload

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-09T23:56:11+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-walk-joyhead90-lat25

**hypothesis**: Composes chassis-payload exposure (dr.mass_scale=1.0,1.4x, the same spread as joyfric-payload) onto the WIDEST driving package (joyhead90-lat25: +-90deg abrupt flips + DR0.5 + latency jitter), distinct envelope from joyfric-payload's narrower +-45deg base. Tests whether the payload-on-driving compose-safety finding (or its absence) generalizes across envelope width, not just one driving line. If-true: JOYSTICK GATE @90 0 falls AND own-cfg gv 12/12, 0 term, prog med >=0.80, DR0 nominal retention clean (slip<=1.24, prog>=0.9) -- payload is compose-safe on driving generally. If-false: DR0 retention erodes (payload-dr05-style) or JOYSTICK GATE falls appear at the wide envelope specifically -- wide-angle steering + payload is a harder combination than narrow steering + payload. Strongest alternative: passes letter but shades vs joyhead90-lat25 baseline on the hardest heading draws -- compare per-episode.

**gate**: JOYSTICK GATE: eval_drive --dr-scale 0.2 --heading-max-deg 90 own cfg - ZERO in-envelope falls; own-cfg DR0.5+latency+dr.mass_scale=1.0,1.4 det+sto 6/6 @15s: gait_valid 12/12, 0 term, prog_ratio med >=0.75; DR0 nominal retention det 6/6 gv, slip/m<=1.24, prog>=0.9; frames watched det

**failed_reason**: run never appeared as 'running' in W&B within 240s

