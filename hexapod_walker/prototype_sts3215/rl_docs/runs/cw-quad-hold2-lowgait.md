# cw-quad-hold2-lowgait

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T06:49:39+00:00

**pod**: hexapod-mjx-train-9

**steps**: 10000000

**parent**: cw-quad-hold2

**wandb_id**: c803p6r3

**hardware_ready**: False

**hypothesis**: Quad-hold (30% mix, walk champion) x crouched stance (-50mm, the verified lowgait height) -- new SKILL COMBINE, not a DR compose (walk_height_off_mm is the mode-agnostic height ref per WISHLIST 18b, quad-hold2 already PASSED at neutral height). Untried whether the four-leg-hold mechanism (front clearance + planted-fraction reward) still works when the body is already lowered -- lower CoM could help stability during weight transfer OR could reduce front-leg ground clearance margin (30mm clearance cap is a fixed absolute, not relative to height). If-true: quad survived_frac>=0.9, height_err_end within the lowgait line's own tolerance (~10mm), walk retention slip/m<=1.25 (matches quad-hold2 band). If-false: front-leg clearance margin shrinks and legs drag/catch during the lift transient (visible in video) -- crouch and quad-hold don't compose for free.

**gate**: own-cfg (quad=0.3/walk=0.6/hold=0.1 + height-50mm) det+sto gv 6/6, 0 term; quad-mode telemetry survived_frac>=0.9, height_err_end<=10mm (lowgait's own gate); walk-mode det slip/m<=1.25 (quad-hold2's own retention band); frames watched det on both walk and quad segments

**verdict**: FAIL on the walk-retention leg of the compound gate (quad-hold mechanism itself stays solid). Own-cfg (quad=0.3/walk=0.6/hold=0.1, crouch -50mm) harness: walk-mode det gv 6/6, 0 term, slip/m med 1.33 -- ABOVE the pre-registered <=1.25 cap (quad-hold2's own band, neutral height), and systematic across all 6 det draws (1.19-1.44, no single outlier driving it) not noise; sto med 1.42 with one known lineage fixed-draw march-in-place stall (prog 0.06/slip 20.86, not new). Quad-mode training telemetry: eval/quad/survived_frac 1.0 at every logged eval point through training, height_err_end_mm 3.8-13.0mm across training, final 5.9mm (<=10mm gate); W&B rollout video (step ~10M) confirms clean level 4-leg stance with both fronts clearly lifted, feet not touching ground, no tipping -- the trick itself transfers to the crouched height fine. Same dose-response pattern as cw-walk-joyquad30 (quad-mix erodes walk-mode slip even when the quad mechanism is unaffected) -- 3rd data point for P0 ruling 7 (quad-mix costs walk economy, is not free); crouch height is an additional, independent cost on top of the mix ratio itself. Not hardware-ready. Next lever per the pre-registered if-false: an explicit anti-slip/economy term active specifically during quad-mix episodes, not further height/mix laddering alone.

