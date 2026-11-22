"""Printing-service guide for the nanoduck TEST batch (tolerance coupon + horn trim jig), as a PDF.

  uv run --with reportlab python tools/print_guide/make_test_print_guide.py

Inputs: exports/xgoduck_scs0009/print_test_batch/manifest.json (from export_print.py with print_up / on_bed) and the thumbnails in
projects/xgoduck_scs0009/print/img/ (render_stl_thumbs.py, then saved as JPEG). Output: projects/xgoduck_scs0009/print/test_batch/nanoduck_test_print_guide.pdf
(the package folder: the PDF + the six print-ready STLs), and a copy next to the exported STLs.
"""
import json
import os
import shutil

from reportlab.lib import colors
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import mm
from reportlab.platypus import Image, PageBreak, Paragraph, SimpleDocTemplate, Spacer, Table, TableStyle

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
BATCH = os.path.join(REPO, "exports/xgoduck_scs0009/print_test_batch")
OUT_DIR = os.path.join(REPO, "projects/xgoduck_scs0009/print")
IMG = os.path.join(OUT_DIR, "img")
PDF = os.path.join(OUT_DIR, "test_batch", "nanoduck_test_print_guide.pdf")   # inside the package folder, next to its STLs
DATE = "2026-11-22"

man = {p["part"]: p for p in json.load(open(os.path.join(BATCH, "manifest.json")))["parts"]}
PIECES = [  # file stem, what it is, how it sits on the bed (already oriented in the file), what we test with it
    ("coupon_knee_rotor", "Knee rotor test piece",
     "Large round recess (bearing seat) DOWN on the bed; the X-shaped pocket faces up. Inside, the roof over the 21 mm recess is "
     "printed as a bridge - that is intended.", "Ball-bearing press fit, horn pocket with crush ribs"),
    ("coupon_knee_cap", "Knee cap test piece", "Wide flat face down, the stepped boss pointing up.", "Boss in the bearing bore"),
    ("coupon_trim_plate", "Trimmed-horn plate", "Flat face down, the small X pocket facing up.", "Pocket for a trimmed horn"),
    ("coupon_bar", "Hole and bridge bar",
     "Flat, as supplied. The long rectangular channel is UNDERNEATH and prints as a 28 mm bridge - do NOT add supports there; "
     "how flat that bridge comes out is one of the things we measure.", "Small bearing seat, nut trap, screw holes, bridging"),
    ("coupon_servo_collar", "Servo collar", "Flat ring, either way up (as supplied).", "Thin walls round a servo body"),
    ("coupon_link_boss_sup", "Lying-down boss, WITH supports (1 dimple)",
     "Standing on its foot, the round boss lying sideways. Print this one WITH tree supports (auto), the only piece that gets "
     "supports.", "Boss and horn pocket printed sideways, supported"),
    ("coupon_link_boss_nosup", "Lying-down boss, NO supports (2 dimples)",
     "Same piece standing on its foot. NO supports: how the underside of the boss and the pocket roof come out is the test.",
     "Boss and horn pocket printed sideways, unsupported"),
    ("horn_trim_jig_r6", "Horn trim jig (a tool)", "Cup opening up, as supplied.", "Used to cut the servo horns - not a test"),
]

ss = getSampleStyleSheet()
H1 = ParagraphStyle("h1", parent=ss["Title"], fontSize=17, leading=20, spaceAfter=2)
H2 = ParagraphStyle("h2", parent=ss["Heading2"], fontSize=12, leading=14, spaceBefore=5, spaceAfter=2, textColor=colors.HexColor("#1f3b57"))
B = ParagraphStyle("b", parent=ss["BodyText"], fontSize=9.2, leading=12, spaceAfter=2)
SMALL = ParagraphStyle("s", parent=B, fontSize=8.2, leading=10.2, spaceAfter=0)
BOX = ParagraphStyle("box", parent=B, backColor=colors.HexColor("#fff4d6"), borderColor=colors.HexColor("#e0b84a"), borderWidth=0.8,
                     borderPadding=6, spaceBefore=4, spaceAfter=8)


def bullets(items):
    return [Paragraph("&bull;&nbsp; " + t, B) for t in items]


def table(rows, widths, head=True):
    t = Table(rows, colWidths=widths, repeatRows=1 if head else 0)
    st = [("GRID", (0, 0), (-1, -1), 0.4, colors.HexColor("#b8c2cc")), ("VALIGN", (0, 0), (-1, -1), "TOP"),
          ("LEFTPADDING", (0, 0), (-1, -1), 4), ("RIGHTPADDING", (0, 0), (-1, -1), 4),
          ("TOPPADDING", (0, 0), (-1, -1), 2), ("BOTTOMPADDING", (0, 0), (-1, -1), 2)]
    if head:
        st += [("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#1f3b57")), ("TEXTCOLOR", (0, 0), (-1, 0), colors.white)]
    t.setStyle(TableStyle(st))
    return t


story = [Paragraph("Nanoduck - test print: instructions for the printing service", H1),
         Paragraph(f"Batch 1 of 2 (test pieces) &nbsp;|&nbsp; {DATE} &nbsp;|&nbsp; 8 files, 1 of each, about 20 g of PLA in total", SMALL),
         Spacer(1, 4),
         Paragraph("<b>Please read this first.</b> These eight small pieces test how your printer reproduces holes, press fits and "
                   "pockets <b>before</b> we order the robot's ~30 parts. We measure them against the real bearings, screws and "
                   "servos and adjust our design to YOUR printer. That only works if the robot parts are later printed <b>exactly "
                   "the same way</b>: same printer, nozzle, material and slicer settings. Please note what you use (section 4).", BOX),
         Paragraph("1. Files", H2)]

SIZES = {"coupon_knee_rotor": "27.9 x 27.9 x 7.3", "coupon_knee_cap": "20.0 x 20.0 x 7.3", "coupon_trim_plate": "16.0 x 16.0 x 3.8",
         "coupon_bar": "78.0 x 16.0 x 4.8", "coupon_servo_collar": "26.3 x 15.1 x 8.0", "horn_trim_jig_r6": "11.3 x 11.3 x 5.7",
         "coupon_link_boss_sup": "20.9 x 9.4 x 26.9", "coupon_link_boss_nosup": "20.9 x 9.4 x 26.9"}
rows = [[Paragraph("<b>File</b>", SMALL), Paragraph("<b>Piece</b>", SMALL), Paragraph("<b>Qty</b>", SMALL),
         Paragraph("<b>Size (mm)</b>", SMALL), Paragraph("<b>Tests</b>", SMALL)]]
for stem, name, _, tests in PIECES:
    rows.append([Paragraph(stem + ".stl", SMALL), Paragraph(name, SMALL), Paragraph("1", SMALL), Paragraph(SIZES[stem], SMALL),
                 Paragraph(tests, SMALL)])
story.append(Spacer(1, 2))
story.append(table(rows, [44 * mm, 36 * mm, 10 * mm, 28 * mm, 52 * mm]))
story += [Spacer(1, 4), Paragraph("Units are <b>millimetres</b>. Do not scale. Every file is one closed, watertight solid.", B)]

story += [Paragraph("2. Orientation - print as supplied", H2),
          Paragraph("Every file is <b>already turned the right way up and placed on the bed</b> (lowest point at z = 0). Please "
                    "<b>do not rotate or flip</b> them, and <b>do not add supports</b> - the overhangs and bridges inside are part "
                    "of the test. <b>One exception: coupon_link_boss_sup gets tree supports (auto)</b>; its twin "
                    "coupon_link_boss_nosup gets none. The pictures on page 2 show how each piece should sit. Several pieces fit on one plate.", B)]

story += [Paragraph("3. Settings (use the SAME ones for the robot parts later)", H2)]
srows = [[Paragraph("<b>Setting</b>", SMALL), Paragraph("<b>Please use</b>", SMALL)],
         ["Material", "PLA (note brand and colour; keep the same for the robot)"],
         ["Nozzle / layer height", "0.4 mm nozzle, 0.2 mm layers"],
         ["Walls (perimeters)", "3 walls (about 1.2 mm)"],
         ["Top / bottom", "4 layers each"],
         ["Infill", "40 %, gyroid or grid"],
         ["Supports", "None - except coupon_link_boss_sup: tree supports (auto)"],
         ["Brim", "Only if a piece will not stay down (the long bar may need one)"],
         ["Hole / XY / elephant-foot compensation", "Whatever you will use for the robot - but tell us which (section 4)"],
         ["Scaling", "None (100 %, millimetres)"]]
story.append(table([[c if not isinstance(c, str) else Paragraph(c, SMALL) for c in r] for r in srows], [62 * mm, 108 * mm]))
story += [Spacer(1, 4), Paragraph("If you normally use different settings for small precise parts, please tell us before printing "
                                  "rather than change them silently: the robot parts must match this test.", SMALL)]

story += [Paragraph("4. Please send back", H2)] + bullets([
    "The eight pieces, <b>as printed</b>: remove only the brim, loose strings and the supports of coupon_link_boss_sup (by hand or pliers; tell us how easily). Please do not sand, drill, ream or clean out holes "
    "and pockets - we need to measure them exactly as the printer made them.",
    "A note of: printer model, nozzle size, material brand and colour, slicer and profile name, layer height, walls, top/bottom "
    "layers, infill, and any hole / XY / elephant-foot compensation. We will ask you to print the robot with the same.",
])

story += [Paragraph("5. What comes next", H2),
          Paragraph("Batch 2 is the robot itself: 29 printed parts (around 280 g of PLA), sent with its own guide after we have "
                    "checked these pieces. Contact for questions: ____________________", B)]

story += [PageBreak(), Paragraph("How each piece sits on the bed", H2),
          Paragraph("As in the files (grey = the print bed). The pieces are shown at different scales.", SMALL), Spacer(1, 4)]
cells, row = [], []
for stem, name, how, _ in PIECES:
    img = Image(os.path.join(IMG, stem + ".jpg"), width=46 * mm, height=46 * mm * 700 / 900)
    row.append([img, Paragraph(f"<b>{name}</b> - {stem}.stl", SMALL), Paragraph(how, SMALL), Spacer(1, 3)])
    if len(row) == 2:
        cells.append(row)
        row = []
if row:
    cells.append(row + [""])
grid = Table(cells, colWidths=[88 * mm, 88 * mm])
grid.setStyle(TableStyle([("VALIGN", (0, 0), (-1, -1), "TOP")]))
story.append(grid)


def footer(c, d):
    c.saveState()
    c.setFont("Helvetica", 7.5)
    c.setFillColor(colors.HexColor("#6b7785"))
    c.drawString(18 * mm, 10 * mm, f"Nanoduck test print guide - batch 1 - {DATE}")
    c.drawRightString(192 * mm, 10 * mm, f"page {d.page}")
    c.restoreState()


doc = SimpleDocTemplate(PDF, pagesize=A4, leftMargin=18 * mm, rightMargin=18 * mm, topMargin=13 * mm, bottomMargin=16 * mm,
                        title="Nanoduck - test print guide (batch 1)", author="nanoduck project")
doc.build(story, onFirstPage=footer, onLaterPages=footer)
shutil.copy(PDF, os.path.join(BATCH, os.path.basename(PDF)))
print("wrote", PDF, "and a copy in", BATCH)
