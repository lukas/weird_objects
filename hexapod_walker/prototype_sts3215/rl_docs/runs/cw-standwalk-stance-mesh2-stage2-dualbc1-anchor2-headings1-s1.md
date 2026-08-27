# cw-standwalk-stance-mesh2-stage2-dualbc1-anchor2-headings1-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: CANARY FAIL - MECHANISM

**created**: 2026-08-27T03:54:39+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor2-s1

**wandb_id**: ryzjxw2m

**hypothesis**: Seed-1 twin of anchor2-headings1 (see that entry for the full hypothesis): opens a +/-45 deg heading command cone (goal.walk_heading_max_rad 0.0 -> 0.7854) on anchor2-s1's own leak-fixed dual-core checkpoint, testing on the second seed whether the walk core learns to follow commanded directions without destabilizing.

**gate**: MECHANISM-HEALTH CANARY ONLY. Same panel and WALK-SURVIVES / DIRECTION-LEARNS / STANCE-UNHARMED clauses as anchor2-headings1; JOINT call with the seed-0 twin per that entry's promote/close branches.

**verdict**: CANARY FAIL - MECHANISM (own-seed read; JOINT call pends anchor2-headings1/seed0, not yet synced). Result: WALK-SURVIVES clause PASSES cleanly (det gait_valid 6/6 at BOTH DR-0 and own-DR(0.5), no leg sacrifice, prog_ratio 0.20-0.42, video confirms real forward translation) but DIRECTION-LEARNS clause FAILS outright -- direction_err_mean_deg median ~55deg at DR-0 and 62-82deg under own-DR, NOT below the ~52deg no-heading baseline (worse, if anything), far from the <=35deg full-pass bar and short of even the >=10deg-drop partial bar. STANCE-UNHARMED holds (hold/sto 6/6 term both DR, identical to the anchor2 baseline, not a regression). Why: opening the +/-45deg heading command cone on the walk core (goal.walk_heading_max_rad 0->0.7854) at this 2M discovery budget does not teach the walk core to use the heading channel at all -- it keeps walking as if heading were still pinned near 0, which is exactly what would happen if the recipe's fixed-forward training left no reachable gradient toward reading vx_ref/vy_ref for direction (the walk core never independently practiced non-zero headings). What's next: this seed's read matches the gate's own pre-registered 'either collapses -> close static cone, use walk_cmd_stage curriculum' fallback in spirit (learning did not happen, even though survival did) -- do not fund a longer-budget continuation of the bare static-cone opener; the track's Next queue should pursue the walk_cmd_stage-style curriculum (graded heading exposure) instead. Awaiting anchor2-headings1 (seed0)'s own read (still mid-eval, prestage timed out at 3300s on both twins -- manual copy-back needed by whichever cycle sees it next) before writing the formal JOINT close in STATUS.md.

