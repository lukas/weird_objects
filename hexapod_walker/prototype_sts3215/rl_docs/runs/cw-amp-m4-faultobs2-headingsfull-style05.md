# cw-amp-m4-faultobs2-headingsfull-style05

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T00:29:34+00:00

**pod**: hexapod-mjx-train-4

**steps**: 4000000

**parent**: cw-amp-m2-bcinit-sec5-style05-headingsfull

**wandb_id**: ihl3kqu6

**hypothesis**: Plain English: style-vs-noamp twin of cw-amp-m4-faultobs2-headingsfull-noamp -- does the AMP style channel help or hurt fault compensation (a genuinely off-distribution, non-nominal-gait situation the motion library never demonstrated)? Every M2 heading/speed axis so far found style functionally neutral (a wash vs noamp); fault recovery is a plausible place for the discriminator to instead actively PENALIZE the compensating limp as off-style, which would be the first real negative style finding. Single lever vs faultobs2-headingsfull-noamp: same fault wiring/budget, +amp-task-weight=0.5/amp-style-weight=0.5 (fresh disc) from the style05-headingsfull checkpoint instead of the noamp-headingsfull one.

**gate**: Acquisition (4M, DR-0). Joint read with the noamp sibling: style faulted-episode prog_ratio/slip within noise of noamp = neutral (matches every prior M2 axis); style clearly worse = first real negative style finding (discriminator vetoing off-distribution compensation, worth a dedicated dig-in); style clearly better = first real positive style finding. Disc health (amp/d_real vs d_fake, style_reward_mean) must stay unsaturated regardless of the behavioral read.

