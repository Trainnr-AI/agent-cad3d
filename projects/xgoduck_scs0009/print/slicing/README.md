# Slicing check of the 29 printed parts (2026-11-22)

Made by `tools/slicer/slice_project.sh projects/xgoduck_scs0009/data/build_args/slicing.json` (OrcaSlicer 2.5.2, ~6 min; see
`tools/slicer/README.md`). Printer **Bambu X1C, 0.4 mm nozzle, Bambu PLA Basic**, with the test-print guide's settings: 0.2 mm
layers, 3 walls, 4 top / bottom, 40 % gyroid, tree supports (auto). Another printer = change the profile names in `slicing.json`
and re-run.

**Not a print package yet**: batch 2 waits for the coupon results (fits may change) and for the decision below.

| File | What |
|---|---|
| `xgoduck_scs0009_parts.3mf` | all 29 parts in their chosen orientation on 3 plates, sliced - open in OrcaSlicer (Preview + layer slider shows supports and bridges) |
| `stl/` | the 29 STLs (mm), turned to the chosen orientation and on the bed |
| `summary.json` | totals and plates |
| `orientation_table.md` | the chosen orientation per part and what its supports still touch |
| `orientation_scores.json` | all 6 orientations per part (support grams, time, fit / mating contacts, alignment error) |
| `chosen_orientation.json` | part -> orientation tag (`px` = model +X up, `mx` = -X up, ...) |
| `fit_surface_contacts.json` | per part and fitted item: support on its axial face (shoulder) or radial face (bore / boss), radius from its axis |

## Totals

13.2 h on 3 plates (6.2 + 5.5 + 1.5 h), **276 g**, of which 34 g support. Extruded **219 cm3** (27 cm3 support); the parts as solid
models are 238 cm3.

## Orientation rule

Least support is not the best: it put support into bearing seats. Each part takes the orientation with the fewest support contacts
on FIT surfaces (next to a bearing, servo case, horn, nut, board), then on other mating faces, then the least support. Two
overrides (reasons in `slicing.json`): electronics enclosure upright (+Z: 4 g support, only the IMU slot, instead of 19 g over the
battery pocket), face +X.

## Clean (14 parts): no support on any surface that meets another part

Dome, jaw, feet, thighs (left: 2 points at the knee servo pocket), knee caps, the four bearing plates, switch plate, Uno cover;
the lid and visor have support only under screw heads.

## Support still on fit surfaces - OPEN, user decision

| Part | Where | Why |
|---|---|---|
| body_frame | shoulder of both hip-yaw 6702 seats (r 8.8-10.1) | seats open downward |
| shins | shoulder of the knee 6702 seat (r 7.5-9.7, ~450-480 points) | same; meant to bridge - the knee-rotor coupon prints "bearing-seat face down as the real shin" with no support, so it tests this |
| L-brackets, yaw connectors, yaw block, head-pitch U | underside of the Ø15 boss through the 6702 (r 7.4) + its horn pocket (r 6.2) | two bearing axes at right angles: one boss always prints lying down |
| head_base | top of the head-roll 6702 bore (r 10.5) | bore prints sideways |
| head_face | the two camera nut pockets | minor clean-up |
| neck cradle, enclosure | servo pocket ends; IMU slot walls | minor clean-up |

Options: if the knee-rotor coupon bridges cleanly, slice the shoulders with support blockers (no support there). For the lying-down
bosses and the head-base bore no coupon exists: a coupon for a lying-down Ø15 boss, or a design change (split parts so every seat
prints standing up, or a printable boss shape).

Not checked: fit tolerance (only the coupon decides), how cleanly support snaps off, strength.
