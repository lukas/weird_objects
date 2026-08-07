# Nesting dolls

Classic split-half matryoshka set sized for a **Bambu Lab H2D**
(325 × 320 × 325 mm). Each doll is a hollow egg that presses together
at the waist; the next size down sits inside with clearance.

## Result

**18 dolls** nest under the conservative print settings below
(largest 300 × 160 mm → smallest 19.9 × 10.6 mm).

Sizing is limited by the **waist opening** of each open bottom half —
the next doll’s widest bulge must actually drop through that bore.

Joint features **scale down** with doll diameter (full size above
~80 mm diameter, floors near ~0.5 mm for wall / rim / lip) so smaller
dolls keep a usable bore. If a hollow joint would fail, a solid
innermost core is used instead (not needed in the current set — all
22 are hollow).

Estimated PLA for the full set: ~1.2 kg of shell plastic
(before paint / infill choices in the slicer).

| # | H (mm) | D (mm) | wall | # | H (mm) | D (mm) | wall |
|---|-------:|-------:|-----:|---|-------:|-------:|-----:|
| 0 | 300.0 | 160.0 | 1.20 | 9 | 115.4 | 61.6 | 1.02 |
| 1 | 276.1 | 147.2 | 1.20 | … |  |  |  |
| … |  |  |  | 16 | 28.8 | 15.3 | 0.6 |
| 8 | 132.2 | 70.5 | 1.11 | 17 | 19.9 | 10.6 | 0.5 |

Full table: `stl/nest_report.txt` after a generate run.

## Constraints

| Parameter | Value |
|---|---|
| Printer | Bambu H2D — 325 × 320 × 325 mm |
| Largest doll | H = 300 mm, D = 160 mm (aspect locked) |
| Wall / gaps (full size) | 1.2 mm / 0.6 mm |
| Feature floors | wall/rim/lip ≈ 0.5 mm; clearance ≈ 0.15 mm |
| Lip joint | male lip on bottom, female socket on top; **0.12 mm** radial clearance (snug press-fit) |
| Min outer size | H ≥ 12 mm, D ≥ 8 mm |

Each doll has a **flat circular base** (~42% of max diameter).

**Joint:** bottom has a raised male lip; top (hat) has a cylindrical
female socket with a shoulder. Looking into the hat you should see a
rim, a straight socket band, then a step into the cavity.

## Files

| File | Purpose |
|---|---|
| `nesting_dolls.py` | Nest sizing solver + half-shell mesh export |
| `stl/` | (gitignored) `doll_XX_top.stl` / `doll_XX_bottom.stl` + `nest_report.txt` |

## Build

```bash
./run.sh nesting_dolls/nesting_dolls.py
# size table only (no meshes):
./run.sh nesting_dolls/nesting_dolls.py --report-only
```
