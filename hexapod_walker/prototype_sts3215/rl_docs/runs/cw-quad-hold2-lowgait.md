# cw-quad-hold2-lowgait

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T06:49:39+00:00

**pod**: hexapod-mjx-train-9

**steps**: 10000000

**parent**: cw-quad-hold2

**wandb_id**: c803p6r3

**hypothesis**: Quad-hold (30% mix, walk champion) x crouched stance (-50mm, the verified lowgait height) -- new SKILL COMBINE, not a DR compose (walk_height_off_mm is the mode-agnostic height ref per WISHLIST 18b, quad-hold2 already PASSED at neutral height). Untried whether the four-leg-hold mechanism (front clearance + planted-fraction reward) still works when the body is already lowered -- lower CoM could help stability during weight transfer OR could reduce front-leg ground clearance margin (30mm clearance cap is a fixed absolute, not relative to height). If-true: quad survived_frac>=0.9, height_err_end within the lowgait line's own tolerance (~10mm), walk retention slip/m<=1.25 (matches quad-hold2 band). If-false: front-leg clearance margin shrinks and legs drag/catch during the lift transient (visible in video) -- crouch and quad-hold don't compose for free.

**gate**: own-cfg (quad=0.3/walk=0.6/hold=0.1 + height-50mm) det+sto gv 6/6, 0 term; quad-mode telemetry survived_frac>=0.9, height_err_end<=10mm (lowgait's own gate); walk-mode det slip/m<=1.25 (quad-hold2's own retention band); frames watched det on both walk and quad segments

