# cw-amp-m2-pilot-noamp-c1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-22T11:51:18+00:00

**pod**: hexapod-mjx-train-0

**steps**: 38000000

**parent**: cw-amp-m2-pilot-noamp

**wandb_id**: 6rt39hx4

**hardware_ready**: no

**hypothesis**: Matched no-AMP control continuation to 40M total for cw-amp-m2-pilot-style05-c1: identical config minus every amp flag, warm-started from its own 2M pilot checkpoint — isolates what the style reward buys at the gate's real comparison budget. Prediction: without a motion prior the policy reaches SOME locomotion by 40M but with from-scratch pathologies (paddle-creep/dragging/non-tripod contact) the AMP arm should visibly beat; if BOTH arms still fail to locomote at 40M, the pilot's branch (iii) fires: narrow the command envelope (brief §6 curriculum 30-50% start) before touching AMP knobs.

**gate**: Control arm: judged only as the comparison baseline for cw-amp-m2-pilot-style05-c1 at 40M total (videos + per-leg gait metrics + falls at equal budget). No SKILLS/champion updates. Both-arms-fail-to-locomote => branch (iii) of the pilot hypothesis.

**verdict**: MISALIGNED (08-21 ruling), wave-1 NO-GO on this reward config. 38M more steps polished the statue, not walking: ALL reward growth came from stand-income (rise_finish 0.09->0.86/tick, posture/height task kernel 0.09->0.59) while walk_speed stayed flat 0.029->0.035 m/s; the sigma-0.05 velocity kernel pays a statue ~0.45/tick across the low/stop command fraction so locomotion income is unreachable from scratch. gait_valid 0/12, prog med 0.02/-0.00, slip med 11-14/m, new tilt_pitch/over_current terms from the ever-more-extreme lean. Cheat encoded in bank (test_slipwalk_stork_statue_is_priced_out: stork -238 vs gait +558 under freeprog stack). Fix arm: cw-amp-m2-freeprog-noamp, from-scratch re-init per the 08-22 init-basin rule.

