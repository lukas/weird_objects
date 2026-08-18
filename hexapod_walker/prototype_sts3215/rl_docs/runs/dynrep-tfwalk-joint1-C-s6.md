# dynrep-tfwalk-joint1-C-s6

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-16T21:30:52+00:00

**pod**: hexapod-mjx-train-4

**steps**: 1000000

**git_sha**: b91c16ba64aad6c8f9d64c1e083dad0781538ad4

**wandb_id**: mjhi4knh

**hypothesis**: Teach the walking policy and its pretrained dynamics transformer TOGETHER, properly: after a 50k-step encoder-frozen head warmup, the future-state prediction loss trains INSIDE every PPO minibatch (same backward/optimizer, transformer at 0.1x LR) on fresh online rollout windows + 25% rehearsal from the recovered v5 corpus, with the total policy movement action-KL guarded (0.02 target / 0.04 rollback guard). Tests whether fixing the out-of-band anchor instability (metrics1-C: led at 1M, last at 2M, approx_kl 4x A/B) finally makes the pretrained representation beat frozen reuse (B) and scratch (A).

**gate**: Matched triple at the PRE-REGISTERED 1M decision checkpoint (metrics1 seed-5 A/B reused at their 1M eval points; config equivalence mechanically verified vs W&B configs jf0tfsqh/psiz3y6x - identical trainer config except budget). Corrected C must: (1) preserve phase-1 heldout prediction quality (aux/heldout total ~within 15% of the pretrained start value on the corpus val split); (2) keep total action-KL of the combined update near the 0.02 target without late regression (guard rejections not saturating, aux not permanently stopped); (3) beat B on walk return AND gait quality (slip_m, peak_roll_deg, slew_sat) across seeds, with rise/hold retention + heldout DR rollouts reported. If corrected C cannot beat B across seeds: STOP adding complexity and record that conclusion. Extension past 1M only after this gate; best checkpoint selected by heldout walking eval, not final step.

**verdict**: CRASHED at 500k, then gate-FAIL on the evidence available: aux permanently stopped, approx_kl 0.107 / action-KL total 0.175 (4-8x guard), heldout pred total 5.62 vs pretrained 2.286, walk 287.2 with slew_sat 0.79/slip 0.80m. best@350k + ck250k CRC-OK; policies extracted to controller artifacts/tfwalk-joint1; ck500000.zip 0-byte husk.

**note**: Script-owned cohort (pod_tfwalk_joint.sh, manifest tfwalk-joint1_manifest.jsonl on-pod). Operator directive fb_20260816T203212_af7c64: corrected joint PPO+auxiliary condition C (code 60922150 + scan-fix 5d12019c, tag exp/cw-dynrep-tfwalk-joint1). Rehearsal corpus = per-file md5-verified copy of train-11's untouched v5_mjx_fresh; canonical aggregate sha256 (cat shard_*.npz, name order) a0bb722ecdd38ebb013ff8dcedf28edd6ffb38510e17e0f7fb570540c03a717a (directive's quoted 6762fe81... not reproducible - see OPERATOR_QUESTIONS q_20260816T2140Z).

