# dynrep-tfwalk-joint1-B-s6

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-16T21:07:14+00:00

**pod**: hexapod-mjx-train-7

**steps**: 1000000

**git_sha**: 60922150bc390cadeb1add8686ba41a0cdc70b72

**wandb_id**: emhdcamd

**hypothesis**: Frozen-encoder arm of the corrected joint-PPO cohort: the G1/G1.1+G3-passing transformer feeds frozen latents to learning PPO heads for the whole run (a head warmup that never ends), new seed for the 3-seed panel. Config-matched to metrics1-B (reused as seed 5).

**gate**: Matched triple at the PRE-REGISTERED 1M decision checkpoint (metrics1 seed-5 A/B reused at their 1M eval points; config equivalence mechanically verified vs W&B configs jf0tfsqh/psiz3y6x - identical trainer config except budget). Corrected C must: (1) preserve phase-1 heldout prediction quality (aux/heldout total ~within 15% of the pretrained start value on the corpus val split); (2) keep total action-KL of the combined update near the 0.02 target without late regression (guard rejections not saturating, aux not permanently stopped); (3) beat B on walk return AND gait quality (slip_m, peak_roll_deg, slew_sat) across seeds, with rise/hold retention + heldout DR rollouts reported. If corrected C cannot beat B across seeds: STOP adding complexity and record that conclusion. Extension past 1M only after this gate; best checkpoint selected by heldout walking eval, not final step.

**note**: Script-owned cohort (pod_tfwalk_joint.sh, manifest tfwalk-joint1_manifest.jsonl on-pod). Operator directive fb_20260816T203212_af7c64: corrected joint PPO+auxiliary condition C (code 60922150, tag exp/cw-dynrep-tfwalk-joint1). Rehearsal corpus = untouched v5_mjx_fresh on train-11 lineage; canonical aggregate sha256 (cat shard_*.npz, name order) a0bb722ecdd38ebb013ff8dcedf28edd6ffb38510e17e0f7fb570540c03a717a (directive's quoted 6762fe81... not reproducible - see OPERATOR_QUESTIONS q_20260816T2140Z).

