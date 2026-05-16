# Bambu X1 / X1 Carbon trays — hexapod prototype

Generated for a **256 x 256 x 256 mm** build volume.

## Plates

- `plate_01_chassis_top.stl` (PLA/PETG rigid): 1 x `chassis_top.stl`
- `plate_02_chassis_bottom.stl` (PLA/PETG rigid): 1 x `chassis_bottom.stl`
- `plate_03_rigid_tibia_links.stl` (PLA/PETG rigid): 6 x `tibia_link.stl`
- `plate_04a_rigid_femur_links_1of2.stl` (PLA/PETG rigid): 3 x `femur_link.stl`
- `plate_04b_rigid_femur_links_2of2.stl` (PLA/PETG rigid): 3 x `femur_link.stl`
- `plate_05_rigid_battery_electronics.stl` (PLA/PETG rigid): 1 x `battery_holder.stl`, 1 x `electronics_tray.stl`
- `plate_06_rigid_coxa_brackets_links.stl` (PLA/PETG rigid): 6 x `coxa_bracket.stl`, 6 x `coxa_link.stl`
- `plate_07_rigid_servo_horn_adapters.stl` (PLA/PETG rigid): 18 x `servo_horn_adapter.stl`
- `plate_08_tpu_foot_pads.stl` (TPU 95A): 6 x `foot_pad.stl`

Import one plate STL at a time into Bambu Studio. Parts are oriented for FDM.
Edge margin: 12 mm; nominal part spacing: 5 mm.

`layout_manifest.csv` records each copy's XY centre and Z rotation.

## Print reliability (dense plates / “spaghetti”)

Large tray STLs are many separate shells on one plate. Reliability is mostly **slicer and environment**, not the STL:

- **Brim (recommended)** — not stored in the STL; turn it on in Bambu Studio. Start with **auto brim** or **outer brim**, **3–5 mm** width, especially on **`foot_pad.stl`** (round bases) and **`servo_horn_adapter.stl`** (small footprint). Per-object: select the part → **Others** → brim.
- **First layer** — slightly lower **initial layer speed** (e.g. 80–120 mm/s effective vs printing max) and confirm **Z offset / bed mesh** so nothing grazes loose mid-print.
- **Draft / cooling** — avoid fans blasting the bed corner on tall skinny features; keep enclosure closed when using one.
- **Filament / moisture** — wet PETG/TPU strings badly; dry filament if you see random blobs or snapped threads.
- **Still failing** — split into fewer parts per plate (we already split X1 vs H2D), slow outer walls for tiny pieces, or print the worst offenders alone from `stl_prototype/`.
