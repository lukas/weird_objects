# cw-amp-m2-pilot-noamp-c1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-22T11:51:18+00:00

**pod**: hexapod-mjx-train-0

**steps**: 38000000

**parent**: cw-amp-m2-pilot-noamp

**wandb_id**: 6rt39hx4

**hypothesis**: Matched no-AMP control continuation to 40M total for cw-amp-m2-pilot-style05-c1: identical config minus every amp flag, warm-started from its own 2M pilot checkpoint — isolates what the style reward buys at the gate's real comparison budget. Prediction: without a motion prior the policy reaches SOME locomotion by 40M but with from-scratch pathologies (paddle-creep/dragging/non-tripod contact) the AMP arm should visibly beat; if BOTH arms still fail to locomote at 40M, the pilot's branch (iii) fires: narrow the command envelope (brief §6 curriculum 30-50% start) before touching AMP knobs.

**gate**: Control arm: judged only as the comparison baseline for cw-amp-m2-pilot-style05-c1 at 40M total (videos + per-leg gait metrics + falls at equal budget). No SKILLS/champion updates. Both-arms-fail-to-locomote => branch (iii) of the pilot hypothesis.

