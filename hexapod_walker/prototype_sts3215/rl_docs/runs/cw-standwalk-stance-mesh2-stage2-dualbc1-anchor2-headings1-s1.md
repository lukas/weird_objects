# cw-standwalk-stance-mesh2-stage2-dualbc1-anchor2-headings1-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-27T03:49:36+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor2-s1

**hypothesis**: Seed-1 twin of anchor2-headings1 (see that entry for the full hypothesis): opens a +/-45 deg heading command cone (goal.walk_heading_max_rad 0.0 -> 0.7854) on anchor2-s1's own leak-fixed dual-core checkpoint, testing on the second seed whether the walk core learns to follow commanded directions without destabilizing.

**gate**: MECHANISM-HEALTH CANARY ONLY. Same panel and WALK-SURVIVES / DIRECTION-LEARNS / STANCE-UNHARMED clauses as anchor2-headings1; JOINT call with the seed-0 twin per that entry's promote/close branches.

**refused_reason**: hexapod-mjx-train-1 code marker 099671613e43532328e9b659095d1b5e6a0d8719-dirty != local HEAD 099671613e43532328e9b659095d1b5e6a0d8719 and the delta is not benign-orchestrator-only. Sync first: snapshot.sh --sync hexapod-mjx-train-1 (and snapshot/commit before that if the tree is dirty).

