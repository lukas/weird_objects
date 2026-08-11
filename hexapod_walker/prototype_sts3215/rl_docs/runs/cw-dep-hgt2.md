# cw-dep-hgt2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-11T12:32:35+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-dep-hgt1

**wandb_id**: ykyalqjd

**hardware_ready**: no

**hypothesis**: DISCOVERY (2M, warm from cw-dep-tip1, same warm start as hgt1): the hgt1 FAIL has a specific mechanism-level suspect — at the policy's operating crouch (-55 to -77 mm) the sigma-30 Gaussian gate sits in its FLAT TAIL (factor ~0.1-0.2, d(factor)/d(height) ~ 0), so PPO sees the discount as a constant tax with no local gradient toward upright; it books the loss and stays low (hgt1: walk_height_factor flat 0.24-0.26 for 2M steps, walk otherwise healthy). ONE variable vs hgt1: reward.walk_height_sigma_mm 30 -> 80, which puts factor ~0.75 and a live gradient at the -55 mm operating point while still charging the crouch ~25% of income. Second and final variant per the two-miss rule; if the policy still won't surface, the crouch is being paid by something outside walk income (likely tipped-start survival value) and the lever closes — the sag stays a documented cosmetic.

**gate**: PASS if env/walk_height_factor rollout mean ends >= 0.85 under sigma-80 accounting (hgt1 managed 0.24-0.26 under sigma-30; comparable crouch depth would read ~0.75 under sigma-80, so require clearly above that) AND harness height: eval walk episodes end z within 35 mm of spawn settle (vs 50-77 mm crouch) AND walk retention: progress_ratio 0.75-1.25, walk_vel_err within 15% of frozen cw-dep-tip1, SCORE/tipped_recovery_success and SCORE/roll_trap_pass not worse than tip1. FAIL if height factor plateaus at the crouch-equivalent reading or walk breaks -> close the height-gate lever (two-miss rule), sag is accepted as cosmetic.

**verdict**: FAIL (pre-registered): sigma-80 gave the crouch a live gradient yet walk_height_factor sank 0.98->0.775 and plateaued from ~0.4M steps (end height -52.5mm = hgt1 crouch depth; the crouch-equivalent reading under sigma-80 is ~0.75). The policy books the height tax regardless of gradient -> the crouch is paid by something outside walk income. Height-gate lever CLOSED per two-miss rule; walk-mode sag accepted as cosmetic.

