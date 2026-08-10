# cw-dep-vref1-r1-payload125

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T15:47:36+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-dep-vref1-r1-payload

**wandb_id**: tjbzfh0g

**hypothesis**: Near-miss follow-up (one, per rules) on the ONLY non-free DR axis found on the hardware candidate: payload 1.0-1.5x broke vref1-r1 (reward collapsed 665->342, det prog 0.57, real sacrificed-leg episode) while 10 sibling axes composed free. Halving the range to 1.0-1.25x asks the deployment-relevant question: is there a SAFE payload envelope for the real battery/harness mass uncertainty, or does any mass DR break the contract-exact velocity obs (obs=ref no longer matching heavier-body dynamics)? Same init/seed/DR0.35/k_current=0 as the payload arm, one variable (range). If-true: own-cfg (DR0.35+mass1.0-1.25x) gv 12/12, 0 term, slip in vref1-r1 band -> operator gets a stated payload tolerance for hardware. If-false: mass DR is fundamentally incompatible with meas:=ref velocity obs at any range -> payload-aware retraining or measured-velocity obs before adding any physical payload.

**gate**: own-cfg (DR0.35 + dr.mass_scale=1.0,1.25) det+sto 6/6 @15s: gait_valid 12/12, 0 term, slip/m within vref1-r1 own band; DR0 no-payload retention clean; frames watched det

