# cw-stand-bc1-coef03

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-11T04:39:54+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-stand-bc1

**wandb_id**: 93fqjm27

**hypothesis**: cw-stand-bc1 (coef=1.0) fixed the rise flag-leg cheat (valid_plant 3/6 det gate, honest six-foot plants video-confirmed) but its OWN training diagnostic showed possible drag on nearby modes it does not directly supervise (raise 0/2 and tipped-recovery 0/2 at 2M vs the identical-recipe parent cw-stand-rsi3's 1-2/2 and 2/2; hold/track angle error also worse, 3.0 deg vs 1.2-1.6 deg) - weak evidence (n=2 probe samples) but worth a direct dose-response check before committing a big hardening budget. This arm tests whether a LOWER anchor coefficient (0.3 vs 1.0, the only change) still stops the rise cheat while costing less cross-mode interference - the anchor's own loss log (train/bc_anchor_loss) and rise geometry are the same signal, cheaper interference is the new axis.

**gate**: env/rise_feet_factor holds >=0.5 through 2M like coef=1.0 did (recovers off its own mid-run dip, does not stay collapsed <0.4); harness at 2M: rise valid_plant on bridge/crouch starts >=2/6 (comparable to coef=1.0's 3/6); hold/track_err_mean_deg <=2.0 (better than coef=1.0's 3.0); training periodic-eval raise/tipped both >=1/2 by 2M (recovered vs coef=1.0's 0/2). If rise collapses back to the pre-anchor cheat (feet-factor <0.4 sustained) at coef=0.3, the anchor needs coef>=1.0 and cross-mode interference is the accepted cost - hardening should keep coef=1.0 and add an explicit anti-interference term instead.

