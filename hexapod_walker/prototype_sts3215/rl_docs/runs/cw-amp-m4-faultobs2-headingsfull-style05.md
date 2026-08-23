# cw-amp-m4-faultobs2-headingsfull-style05

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-23T00:29:34+00:00

**pod**: hexapod-mjx-train-4

**steps**: 4000000

**parent**: cw-amp-m2-bcinit-sec5-style05-headingsfull

**wandb_id**: ihl3kqu6

**hypothesis**: Plain English: style-vs-noamp twin of cw-amp-m4-faultobs2-headingsfull-noamp -- does the AMP style channel help or hurt fault compensation (a genuinely off-distribution, non-nominal-gait situation the motion library never demonstrated)? Every M2 heading/speed axis so far found style functionally neutral (a wash vs noamp); fault recovery is a plausible place for the discriminator to instead actively PENALIZE the compensating limp as off-style, which would be the first real negative style finding. Single lever vs faultobs2-headingsfull-noamp: same fault wiring/budget, +amp-task-weight=0.5/amp-style-weight=0.5 (fresh disc) from the style05-headingsfull checkpoint instead of the noamp-headingsfull one.

**gate**: Acquisition (4M, DR-0). Joint read with the noamp sibling: style faulted-episode prog_ratio/slip within noise of noamp = neutral (matches every prior M2 axis); style clearly worse = first real negative style finding (discriminator vetoing off-distribution compensation, worth a dedicated dig-in); style clearly better = first real positive style finding. Disc health (amp/d_real vs d_fake, style_reward_mean) must stay unsaturated regardless of the behavioral read.

**verdict**: PASS (neutral branch of the pre-registered joint read): AMP style neither helps nor hurts fault compensation on the full-heading substrate -- the 'discriminator vetoes the off-style limp' branch did NOT fire. DR-0 gate on own cfg (same eval seed as the noamp sibling => identical per-episode fault draws, paired read valid): gait_valid 12/12 det+sto (det prog med 1.09/slip 3.08, sto 0.57/7.26) vs noamp's 11/12 (1.14/3.08, 0.59/6.03) -- every axis within 6-ep noise, and the hard fault episodes (sto ep0/3/4) degrade in BOTH arms with the same shape, so the deltas are the fault draw, not style. Heights 2-16mm, zero terminations. Disc health bar met: d_real 0.78 / d_fake -0.93, gp 0.007, style_reward_mean 0.121 (>0.1, unsaturated). This extends the M2-wide 'style is functionally neutral' series to the first genuinely off-distribution axis (fault recovery transients absent from teacher_v2). Blind-vs-sighted remains the open question on this substrate -- blind control launched from the noamp twin this cycle.

