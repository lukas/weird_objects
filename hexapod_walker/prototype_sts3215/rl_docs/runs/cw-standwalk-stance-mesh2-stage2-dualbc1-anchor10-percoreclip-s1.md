# cw-standwalk-stance-mesh2-stage2-dualbc1-anchor10-percoreclip-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-27T13:52:46+00:00

**pod**: hexapod-mjx-train-5

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor9-gradnorm-s1

**wandb_id**: txxn1c54

**hypothesis**: Seed1 twin of cw-standwalk-stance-mesh2-stage2-dualbc1-anchor10-percoreclip (see that run's own hypothesis for the full mechanism writeup). IMPORTANT UPDATE since seed0 was launched: a concurrent cycle's own anchor9-gradnorm-s1 read (the gradnorm diagnostic's seed1 twin) does NOT replicate seed0's clean support for the grad-clip-coupling hypothesis -- seed1's own b/a ratio is comparable-to-inverted at the trough (median 0.73, core A/walk often the LARGER raw gradient there), only 10/30 rollouts clear the >=3x bar cross the full run. The joint gradnorm read is therefore DIVERGENT, not a clean cross-seed SUPPORT. This seed1 percore-clip launch is still warranted as a direct BEHAVIORAL test (does decoupling the clip rescue walk from the anchor4-class freeze, independent of whether the diagnostic evidence for the mechanism was clean) using the same init-from anchor2_s1 / seed=1 recipe anchor9-gradnorm-s1 itself used, with ONLY train.bc_anchor_percore_clip=1 added. Per the pre-registered joint-close discipline this campaign has used throughout (anchor2/3/6b/7 etc.), the two seeds' walk-survival reads get closed together, not singly.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY: do not judge skill acquisition or require a mature gait at 2M steps -- judge only whether decoupling the clip rescues walk from the anchor4-class catastrophe (or fails to). WIRING CHECK FIRST: before reading gait_valid, confirm via a direct checkpoint/code check that bc_anchor_percore_clip actually fired on the dual-core branch this run (cfg_set has train.bc_anchor_percore_clip=1 AND the policy has mlp_extractor_b) -- a wiring miss here (e.g. a plain load path that fails to attach the dual policy_kwargs) has bitten this exact campaign before (anchor6-logstdsplit CANARY FAIL - INFRASTRUCTURE). Read (WALK-SURVIVES): det gait_valid_frac at DR-0 -- >=5/6 with no persistent 2+-leg sacrifice = mechanism WORKS, promote per-core clip as the standing default for this recipe family; 1-4/6 with a milder/partial drag (matching anchor7-detachtrunk's rescued-seed1 shape) = PARTIAL, informative, dose/placement follow-up; 0/6 with the SAME total leg-sacrifice-freeze signature as anchor4/5/6/6b/7/8/9's own baseline = REFUTED -- every dual-core architecture/optimizer-sharing coupling mechanism has now been tested and refuted; escalate to a reward/task-level audit of the walk objective itself (is walk simply the harder-to-optimize half of this exact goal-mix/DR recipe, independent of any cross-mode coupling), not another architecture-sharing arm. Joint call across both seeds per the campaign's standard practice (seed-level noise vs a real cross-seed effect) -- do not verdict off one seed alone unless the other seed's own eval is still mid-computation and the wiring/read is otherwise unambiguous.
 CAVEAT (seed1-specific): the gradnorm diagnostic's own seed1 read (anchor9-gradnorm-s1) was comparable/inverted at the trough, not supportive like seed0 -- so neither this run's pass NOR fail should be read as confirming/refuting grad-clip-coupling as THE mechanism; read it purely as walk-survival behavior, and close jointly with anchor10-percoreclip (seed0) per the campaign's standard joint-call practice.

