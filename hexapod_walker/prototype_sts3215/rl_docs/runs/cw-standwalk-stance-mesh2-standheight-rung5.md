# cw-standwalk-stance-mesh2-standheight-rung5

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-26T04:38:30+00:00

**pod**: hexapod-mjx-train-6

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stancemix-tuckclock-scratch8m

**wandb_id**: 7sg3akct

**hypothesis**: Plain English: can the just-promoted mesh policy (which can already sit up, stand quietly, and lie back down) also learn to raise/lower its stand height ON COMMAND while doing rise->hold->lower in one continuous sequence, without breaking the rise/lower skills it already has? Mechanism: this cycle wired goal.mode_seq_hold_height_cmd (new, default off) so a mid-sequence 'hold' segment of the existing goal.mode_seq_stance planner can carry the same hold/ramp/sine/pulse height-command script a standalone hold episode already uses (STAND_HEIGHT rungs 1-3, already proven on the isolated hold specialist), instead of always being flat. Warm-started from the just-promoted from-scratch stancemix checkpoint (rise+hold+lower all clean, JOINT PASS this cycle) with train.bc_anchor_hold_height_aware=1 added (the rung-1 fix for a height-blind anchor fighting a moving command). Prediction-if-true: reward learns (rises), a custom mode_seq+height-cmd probe shows real per-episode height tracking during the hold segment with rise/lower either preserved or only mildly softened. Prediction-if-false: the sharp new distribution (first exposure to both mode-sequencing AND height commands at once, on top of a checkpoint that never saw either) collapses rise/lower back toward the pre-stancemix pathologies (splay/pin) -- MECHANISM canary, not a behavior close either way.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY (2M): does composing rise->hold(height-cmd)->lower sequencing on top of the promoted stance skill learn without collapsing it. PASS if reward rises over the run AND a custom probe (--cfg-set goal.mode_seq_stance=1.0 mode_seq_hold_height_cmd=1.0 hold_height_cmd_frac=1.0, det+sto 6+6) shows: no majority over_current/fall (<=6/12), the hold segment's height error trends toward the commanded target rather than flat-ignoring it, and rise/lower segments still reach a valid plant / low herr on most episodes (not a full regression to splay/press-pin). PARTIAL if reward rises but height tracking during the hold segment is flat/ignored (anchor still fighting the command) or rise/lower softens only mildly -- next lever is loosening bc_anchor_coef during height-cmd hold segments specifically, or more budget. FAIL if reward is flat OR rise/lower collapse to a majority over_current/fall (structural interference between mode-sequencing and height-commanding) -- next lever is a staged introduction (mode_seq alone first, then add height-cmd from that checkpoint) rather than both novelties at once.

