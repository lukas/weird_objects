# cw-amp-m2-bcinit-sec5-noamp-headingsfull

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-22T22:09:43+00:00

**pod**: hexapod-mjx-train-3

**steps**: 2000000

**parent**: cw-amp-m2-bcinit-sec5-noamp-headings20

**wandb_id**: 6d8qdu6s

**hypothesis**: Task-only twin of cw-amp-m2-bcinit-sec5-style05-headingsfull (PASS: full-circle including backward walking works in one jump from +/-25deg, no intermediate rung needed, dir_err med det 33deg/sto 50deg, gait_valid 6/6 both). Does the noamp BC-init walker also handle full-circle commands including backward -- the hardest test yet since backward is never demonstrated by the forward-only BC teacher -- or is style specifically necessary for the backward/full-circle generalization (the strongest candidate so far for a REAL functional style benefit, since headings20/90 showed the style edge was cosmetic/washed out)? Continues from noamp-headings20 (--init-from-source), single lever: goal.walk_heading_max_rad 0.4363 -> -1 (full circle), identical to the style05 sibling.

**gate**: Discovery continuation (2M, judged on det video + DR-0 gate harness at full-circle range, read JOINTLY with the style05-headingsfull sibling). INFORMATIVE-PASS = gait_valid >=5/6 det+sto, height_err stays in the 14-26mm band, dir_err not stuck near the ~90deg ignore-baseline (i.e. rear hemisphere actually followed, not just forward+lateral). FAIL-collapse/partial = gait degrades, or forward/lateral commands are followed but rear-hemisphere (backward) commands are ignored (bimodal dir_err) -- if noamp shows this while style05 does not, style earns its first real functional win (enabling backward generalization, not just margin polish).

**verdict**: noamp twin ALSO handles full-circle commands including backward in one jump from +/-25deg, matching the style05-headingsfull sibling (PASS: no intermediate rung needed). DR-0 gate: gait_valid 6/6 det+sto, zero sacrificed legs/terminations; det dir_err mean 29-41deg per episode (wrong_direction_frac 0.05-0.15) -- NOT bimodal/stuck at the ~90deg ignore-baseline, closely matching style05's own det dir_err ~33deg/wrong-frac<=0.11; sto dir_err 49-57deg (wrong_direction_frac 0.17-0.24), again matching style05's ~50deg/p90-146 band. det prog med 1.10/slip 2.90/fwd 0.44m, sto prog med 0.87/slip 4.14/fwd 0.63m -- comparable to style05's 1.19/2.77/0.47m det and 0.87/3.57/0.56m sto. Frame-strip (walk_det_4) shows clean six-leg alternating-tripod cycling throughout, no drag/skate/flag-leg, real off-axis/rear-hemisphere displacement. CLOSES the 'is style necessary for backward generalization' question in the negative -- the strongest remaining candidate for a functional (not cosmetic) style benefit does not pan out either: noamp handles the full command circle, including commands the BC teacher never demonstrated, just as well as the style arm. Batch conclusion across the whole heading curriculum (0/25/90/full, 4 stages x 2 arms = 8 runs, 8/8 PASS): the BC-init walking basin is robust to the ENTIRE heading command space with or without AMP style; style's only measured effect anywhere in this curriculum was a small, non-monotonic margin (helped at 0/25deg, washed out at 90deg, matched at full-circle) -- not yet earning a real functional role. hardware-ready: no (2M discovery, DR-0, full-circle envelope only just opened -- speed range/stops and yaw-rate commands remain untested, already in flight on the style05 line via a concurrent cycle).

