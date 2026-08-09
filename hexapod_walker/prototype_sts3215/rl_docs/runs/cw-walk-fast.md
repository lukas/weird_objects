# cw-walk-fast

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T12:01:04+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-walk-anchorgate

**wandb_id**: zxo9fi90

**hardware_ready**: no

**hypothesis**: OPERATOR WISHLIST: faster walking. Champion 35234ddc retrained on 0.08-0.12 m/s command band (vs 0.05-0.06). If-true: det tracking err <= champion band-relative, gait_valid at 0.08+; if-false: gait degrades to lurch/skate at higher speed (shows speed ceiling is gait-limited not command-limited).

**gate**: DR0 det+sto 6/6 at 0.08-0.12 band: gait_valid, zero terminations, det slip/m <= 1.24

**verdict**: FAIL (gate: tracking) / hypothesis REFUTED on if-false: policy ignored the 0.08-0.12 band and kept the champion's ~0.065 m/s paddle gait for all 20M steps (det speed 0.065-0.066 all 6 eps, prog_ratio 0.53-0.72 < 0.75). Gait retained: 12/12 gait_valid, 0 term, det slip/m 1.14 ~ champion. One sto blowout (prog 0, slip/m 17) = known sto brittleness (c39). Speed ceiling is gait-limited, not command-limited — faster walking blocked on the same contact-pricing root as skating.

