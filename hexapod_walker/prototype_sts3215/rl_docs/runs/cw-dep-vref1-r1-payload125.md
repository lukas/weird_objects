# cw-dep-vref1-r1-payload125

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-10T15:47:36+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-dep-vref1-r1-payload

**wandb_id**: tjbzfh0g

**hardware_ready**: False

**hypothesis**: Near-miss follow-up (one, per rules) on the ONLY non-free DR axis found on the hardware candidate: payload 1.0-1.5x broke vref1-r1 (reward collapsed 665->342, det prog 0.57, real sacrificed-leg episode) while 10 sibling axes composed free. Halving the range to 1.0-1.25x asks the deployment-relevant question: is there a SAFE payload envelope for the real battery/harness mass uncertainty, or does any mass DR break the contract-exact velocity obs (obs=ref no longer matching heavier-body dynamics)? Same init/seed/DR0.35/k_current=0 as the payload arm, one variable (range). If-true: own-cfg (DR0.35+mass1.0-1.25x) gv 12/12, 0 term, slip in vref1-r1 band -> operator gets a stated payload tolerance for hardware. If-false: mass DR is fundamentally incompatible with meas:=ref velocity obs at any range -> payload-aware retraining or measured-velocity obs before adding any physical payload.

**gate**: own-cfg (DR0.35 + dr.mass_scale=1.0,1.25) det+sto 6/6 @15s: gait_valid 12/12, 0 term, slip/m within vref1-r1 own band; DR0 no-payload retention clean; frames watched det

**verdict**: FAIL -- near-miss follow-up to the payload FAIL refuted: halving the mass-DR range (1.0-1.5x -> 1.0-1.25x) fixes the WIDE-range symptom (parent's det/3 sacrificed-leg episode is gone; 5/6 det + 6/6 sto episodes now match vref1-r1's own slip/prog band almost exactly, med slip 1.11 det / 1.01 sto vs band 0.89-1.13/1.13-1.36) but leaves a second, independent failure untouched: det/4 (same eval seed+index as the parent's own crater) is a near-total stall -- prog_ratio 0.004, slip/m 28.9, return -8.2, frame strip shows a frozen splayed-leg stance held the FULL 15s with zero gait cycling (not a fall, not gv-flagged, but plainly not walking). This lands in the DETERMINISTIC (deployment) mode; vref1-r1's own known crater only ever hits sto/4 and is accepted as stochastic-sampling noise since det stays clean 6/6 -- here mass DR pushes a genuinely new failure into det. Gate's literal det+sto 6/6 not met (5/6 det). RULING: no safe range rescues this axis -- the mass-DR / contract-exact-velocity-obs interaction (if-false branch) is confirmed, not a range-tuning problem. Payload/mass DR stays OFF the hardware-candidate default; near-miss-follow-up quota (one, per rules) is used; class closed pending payload-aware retraining or measured-velocity obs (per the original payload FAIL ruling), not a further narrower-range respec.

