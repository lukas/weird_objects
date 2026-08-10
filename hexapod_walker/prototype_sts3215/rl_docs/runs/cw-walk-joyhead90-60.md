# cw-walk-joyhead90-60

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T23:43:34+00:00

**pod**: hexapod-mjx-train-2

**steps**: 20000000

**parent**: cw-walk-joyhead90-lat25

**wandb_id**: 749jyr8x

**hardware_ready**: False

**hypothesis**: Duration rung off joyhead90-lat25 (widest +-90deg envelope + DR0.5 + latency jitter, JOYSTICK GATE PASS at 15s): joylat60 already proved the narrower +-45deg driving package holds for 60s with no decay -- this tests whether the widest envelope also survives minute-scale driving without degradation. One variable: episode-seconds 15 -> 60 (matches joylat60's pattern). If-true: own-cfg 60s panel gv 12/12, 0 term, no first/second-half decay (swing counts even across all six legs), JOYSTICK GATE @90 0 falls, DR0 retention clean -- widest envelope is endurance-hardened too. If-false: late-episode decay or falls emerge only at the wider heading envelope over a full minute (narrower package didn't show this) -- envelope width and endurance interact, needs its own hardening. Strongest alternative: passes but slip creeps late in the episode -- compare first-half vs second-half slip.

**gate**: Own-cfg 60s harness (DR0.5+latency, +-90deg envelope) det+sto 6/6: gait_valid 12/12, 0 term, prog med >=0.80, no first/second-half decay (frames watched full 60s); JOYSTICK GATE @90 0 in-envelope falls; DR0 retention det 6/6 gv

**verdict**: PASS. Widest +-90deg envelope + DR0.5 latency package survives a full 60s drive, matching joylat60's 15s->60s endurance pattern at the wider heading. Own-cfg 60s det+sto gv 12/12, 0 term, prog med 0.97/0.94 (>=0.80 gate), slip 1.65/1.58; per-episode swing counts even across all six legs (39-58 range, no leg outlier) and progress_ratio stable across all 6 episodes (0.92-0.98) -- no first/second-half decay. JOYSTICK GATE @90deg 0 in-envelope falls (incl 3 flip-stress episodes, stop-go, fwd-back-flip). DR0 nominal retention gv 6/6, 0 term, prog med 0.96/0.95, slip 1.63/1.75 -- matches parent joyhead90-lat25's own retention band (1.64/1.73) almost exactly, no erosion from the duration extension. Frames (det, full 60s): six legs cycling throughout, level body, no flag leg, no late-episode degradation visible; foot-slide/paddling persists (contact-pricing root, unchanged). Not hardware-ready (paddling).

