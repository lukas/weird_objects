# cw-amp-m2-bcinit-sec5-noamp-headingsfull

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-22T22:09:43+00:00

**pod**: hexapod-mjx-train-3

**steps**: 2000000

**parent**: cw-amp-m2-bcinit-sec5-noamp-headings20

**wandb_id**: 6d8qdu6s

**hypothesis**: Task-only twin of cw-amp-m2-bcinit-sec5-style05-headingsfull (PASS: full-circle including backward walking works in one jump from +/-25deg, no intermediate rung needed, dir_err med det 33deg/sto 50deg, gait_valid 6/6 both). Does the noamp BC-init walker also handle full-circle commands including backward -- the hardest test yet since backward is never demonstrated by the forward-only BC teacher -- or is style specifically necessary for the backward/full-circle generalization (the strongest candidate so far for a REAL functional style benefit, since headings20/90 showed the style edge was cosmetic/washed out)? Continues from noamp-headings20 (--init-from-source), single lever: goal.walk_heading_max_rad 0.4363 -> -1 (full circle), identical to the style05 sibling.

**gate**: Discovery continuation (2M, judged on det video + DR-0 gate harness at full-circle range, read JOINTLY with the style05-headingsfull sibling). INFORMATIVE-PASS = gait_valid >=5/6 det+sto, height_err stays in the 14-26mm band, dir_err not stuck near the ~90deg ignore-baseline (i.e. rear hemisphere actually followed, not just forward+lateral). FAIL-collapse/partial = gait degrades, or forward/lateral commands are followed but rear-hemisphere (backward) commands are ignored (bimodal dir_err) -- if noamp shows this while style05 does not, style earns its first real functional win (enabling backward generalization, not just margin polish).

