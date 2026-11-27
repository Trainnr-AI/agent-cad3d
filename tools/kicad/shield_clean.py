"""UNO Q shield: remove every track, arc and via (before re-placing / re-routing). Separate process on purpose: in KiCad
10.0.8's SWIG, board.Remove() leaves later calls with unwrapped objects, so removal is the only thing this run does.
    <KiCad python> tools/kicad/shield_clean.py <board.kicad_pcb>
"""
import sys

import pcbnew

b = pcbnew.LoadBoard(sys.argv[1])
items = list(b.GetTracks())
for t in items:
    b.Remove(t)
b.Save(sys.argv[1])
print("removed", len(items), "tracks / vias")
