# cw-dep-vref1-r1-latency-comshift

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T18:54:12+00:00

**pod**: hexapod-mjx-train-1

**steps**: 20000000

**parent**: cw-dep-vref1-r1-latency

**wandb_id**: od09vqbd

**hypothesis**: Plain English: does the checkpoint headed for tonight's hardware attempt still walk cleanly if comms/bus delay (already PASSed alone) and an off-center chassis CoM (already PASSed alone) happen together -- a laggy control loop trying to correct a lopsided body? Both PASSed individually but never combined, and they stress the SAME feedback-timing pathway (a delayed correction matters more when the thing needing correcting, body lean, is persistent rather than transient). Per P0 rule 3, k_current=0 (inherited). If-true: own-cfg det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band -- composes free like every other pairing tonight. If-false: delayed correction of a persistent lean degrades the gait beyond either alone -- flag as a real pre-attempt-#2 risk.

**gate**: own-cfg (DR0.35 + dr.latency_scale=0.5,2.5 + dr.com_offset_m=0.03) det+sto 6/6 @15s gait_valid 12/12, 0 term, slip/m within vref1-r1's own band +-20%; DR0-no-override retention det 6/6 gv; video frames watched det+sto

