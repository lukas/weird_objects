# cw-walk-placementnoise6-payload

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-10T03:45:08+00:00

**pod**: hexapod-mjx-train-7

**steps**: 20000000

**parent**: cw-walk-placementnoise6-r3

**hypothesis**: Compose: payload (1.0-1.5x mass, the payload envelope that composed cleanly onto groundtilt5/joyheadfric/deadband30) onto placementnoise6 (hand-placement-slop 6deg, PASSED r3). Untried pairing -- extra mass could amplify the balance correction demanded by noisy initial foot placement. If-true: own-cfg det+sto 6/6 gv, 0 term, det med fwd>=1.2m matching placementnoise6's own band; DR0 flat-no-payload retention clean. If-false: payload on placement-slop compounds into a fall/flag-leg tail the noise-alone axis didn't show.

**gate**: Own-cfg (dr.placement_noise_deg=6.0 + dr.mass_scale=1.0,1.5) det+sto 6/6 @30s: gait_valid 12/12, 0 term, det med fwd>=1.2m; DR0 flat retention det 6/6 gv, slip/m<=1.24, prog>=0.90; frames watched det

