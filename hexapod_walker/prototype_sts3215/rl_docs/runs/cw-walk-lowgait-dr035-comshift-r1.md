# cw-walk-lowgait-dr035-comshift-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T03:29:52+00:00

**pod**: hexapod-mjx-train-4

**steps**: 18000000

**parent**: cw-walk-lowgait-dr035

**wandb_id**: 5g8hj91c

**hardware_ready**: no

**hypothesis**: Retry (r1) of cw-walk-lowgait-dr035-comshift, which died 0-step to the fleet launch-collision storm (infra, not science). Same hypothesis: crouch stance (-50mm, DR0.35) x off-center CoM payload (0.03m) compose, untried pairing.

**gate**: Own-cfg (DR0.35+dr.com_offset_m=0.03) det+sto 6/6 @15s: gait_valid 12/12, 0 term, mean end-height err<=10mm, slip/m med<=1.6; DR0 no-offset retention det 6/6 gv, mean height err<=8mm, slip/m med<=1.15; frames watched det

**verdict**: PASS (dig-in) — off-center CoM (dr.com_offset_m=0.03) composes onto crouch-50mm+DR0.35. OBSERVATIONS: own-cfg det+sto gv 12/12, 0 term, slip med 1.02/1.36 (<=1.6), mean end-height err 4.8/4.1mm (<=10); DR0.35 tail matches parent's gate episode-for-episode (det/5 slip 2.43 vs parent 2.18; sto/0-1 ~1.8 vs 2.0-2.1). Flagged DR0 det/4 stall (slip/m 17.5, fwd 0.11m) is a MARCH-IN-PLACE, not a freeze/fall: all 6 legs cycle (13-18 swings/leg, gv True), body level (h_err 7.5mm), stride 2cm — video looks normal because the legs never stop. Fresh-draw panel (24 eps/side, DR0 seeds 1+2, identical cfg): child 2/36 degraded eps (prog 0.11 slip 17.5; 0.59/2.47) vs parent 2/36 (0.48/2.94; 0.49/3.23); on the one shared bad fresh draw (s2 det/0) the PARENT is WORSE (0.49/3.23 vs child 0.59/2.47); all medians match parent. INTERPRETATION: the stall is the known lineage rare fixed-draw paddling attractor (champion canary-only class since c52; same class fricvar dig-in root-caused), not comshift-induced brittleness — comshift training only moved WHICH draw lands on the knife edge (parent stumbles on s2-det/0, child on s0-det/4). Root-cause chain: in-place paddling <- walk income admits a low-transport attractor on rare reset draws <- contact/current pricing (operator P0 ruling class) <- no run-specific sim defect. VERDICT: PASS, hardware-ready NO (lineage paddling + deployment-pipeline gap). HYPOTHESIS STATUS: if-true confirmed.

