# Print packages for the printing service

`slicing/` - the slicing check of all 29 robot parts (OrcaSlicer: time, grams, cm3, chosen orientations, where supports still touch
fit surfaces). Not a package for the service yet - see `slicing/README.md`.

## `test_batch/` - batch 1, send this first

The whole folder goes to the printing service: `nanoduck_test_print_guide.pdf` (2 pages: files, settings, what to send back, a
picture of how each piece sits on the bed) and the eight STLs (mm), **already turned to their print orientation and placed on the
bed** - the service prints them as supplied:

| File | Print orientation | What it tests |
|---|---|---|
| coupon_knee_rotor.stl | bearing-seat face down (as the real shin prints) | 6702ZZ press fit, full X-horn pocket with crush ribs |
| coupon_knee_cap.stl | top face down (as the real cap) | boss in the bearing bore |
| coupon_trim_plate.stl | pocket up | trimmed-horn pocket |
| coupon_bar.stl | as modelled: the 28 mm channel underneath prints as a bridge | 693ZZ seat, M3 nut trap, M2 / M3 pilots, M2 clearance, bridging |
| coupon_servo_collar.stl | as modelled | servo body in thin saddle walls |
| coupon_link_boss_sup.stl (1 dimple) | on its foot, joint axis lying down (as the L-brackets, yaw connectors, yaw block, head-pitch U); **WITH tree supports** | Ø14.9 boss in the 6702 bore + trimmed-horn pocket, printed sideways with the support the slicer puts there (0.56 g, as the yaw connector) |
| coupon_link_boss_nosup.stl (2 dimples) | the same, **no supports** | the same unsupported: if it fits, those 5 parts can print without support there |
| horn_trim_jig_r6.stl | cup up | the tool for cutting 9 servo horns (not a test) |

**2026-11-25 - reprint 4 pieces** (first print: the servo's screw did not reach the shaft - the horn's top face has a raised ring
round its hole): `coupon_knee_rotor`, `coupon_trim_plate`, `coupon_link_boss_sup`, `coupon_link_boss_nosup` now have a Ø5.2 x 0.35
recess in the pocket roof for it (0.6 mm floor under the screw head). Same settings and orientation; the other 4 files are unchanged.
Test: horn in the pocket, onto the servo, kit screw - count the turns it takes before it is tight.

When the pieces come back: test them as in ASSEMBLY.md section 0, item 2, and give the agent caliper measurements of anything
tight or loose (the fit constants are in `tools/blender/parts/joint_v2.py`). Batch 2 (the robot, ~30 parts) gets its own guide
after that, printed with the same settings the service reports back.

## Regenerate (after any change to the coupon or the joint code)

```sh
# 1. STLs, oriented and on the bed (Blender running with the project; or headless via tools/blender/run_headless.py)
python3 scripts/blender_doctor.py tools/blender/export_print.py --args '{"collections": ["coupon"], "extra": ["horn_trim_jig_r6"],
  "out_dir": "'$PWD'/exports/xgoduck_scs0009/print_test_batch",
  "print_up": {"coupon_knee_rotor": "-z", "coupon_knee_cap": "-z", "coupon_trim_plate": "-z",
  "coupon_link_boss_sup": "+y", "coupon_link_boss_nosup": "+y"}, "on_bed": true}'
cp exports/xgoduck_scs0009/print_test_batch/*.stl projects/xgoduck_scs0009/print/test_batch/
# 2. pictures (clean headless Blender), then JPEG
blender -b --factory-startup --python tools/print_guide/render_stl_thumbs.py -- exports/xgoduck_scs0009/print_test_batch /tmp/thumbs
uv run --with pillow python -c "from PIL import Image; import glob; [Image.open(f).convert('RGB').save('projects/xgoduck_scs0009/print/img/' + f.split('/')[-1][:-4] + '.jpg', quality=85) for f in glob.glob('/tmp/thumbs/*.png')]"
# 3. the PDF (reads exports/.../print_test_batch/manifest.json)
uv run --with reportlab python tools/print_guide/make_test_print_guide.py
```
