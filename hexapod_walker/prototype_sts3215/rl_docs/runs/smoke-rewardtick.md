# smoke-rewardtick

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-15T13:46:25+00:00

**pod**: hexapod-mjx-train-4

**steps**: 2048

**hypothesis**: smoke: optimization/reward_per_tick logging change boots and trains

**gate**: boots, trains, no crash, optimization/reward_per_tick* keys appear in stdout/log

**refused_reason**: hexapod-mjx-train-4 code marker 882a2c423935b80e29c0b28fb30d171fcb29328f != local HEAD d057d22dc00ce0acd598fc3365c82af2eebd15bd. Sync first: snapshot.sh --sync hexapod-mjx-train-4 (and snapshot/commit before that if the tree is dirty).

