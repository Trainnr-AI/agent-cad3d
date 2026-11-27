"""UNO Q shield: fill the copper zones, autoroute the rest with Freerouting, refill, save. Re-runnable.

Uses the KiCad MCP server's own autoroute code (vendor/KiCAD-MCP-Server/python/commands/freerouting.py): it pushes the project's
net classes into the DSN and repairs net names on the SES round-trip. Runs outside the MCP process so JAVA_HOME can point at
Homebrew's keg-only Java 21 (Freerouting 2.3.0, ~/.kicad-mcp/freerouting.jar).
    vendor/KiCAD-MCP-Server/venv/bin/python tools/kicad/shield_route.py projects/xgoduck_scs0009/pcb/unoq_shield/unoq_shield.kicad_pcb [attempts]
"""
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
os.environ.setdefault("JAVA_HOME", "/opt/homebrew/opt/openjdk@21")
sys.path.insert(0, os.path.join(ROOT, "vendor", "KiCAD-MCP-Server", "python"))

import pcbnew  # noqa: E402
from commands.freerouting import FreeroutingCommands  # noqa: E402

BOARD = os.path.abspath(sys.argv[1])
ATTEMPTS = int(sys.argv[2]) if len(sys.argv) > 2 else 3

b = pcbnew.LoadBoard(BOARD)
pcbnew.ZONE_FILLER(b).Fill(b.Zones())                 # planes present, so Freerouting connects pads to them
fc = FreeroutingCommands(b)
r = fc.autoroute({"boardPath": BOARD, "attempts": ATTEMPTS, "timeout": 300,
                  "signalLayers": ["F.Cu", "B.Cu"]})   # In1 = solid GND plane, In2 = V_SERVO / GND planes: no tracks there
keep = {k: v for k, v in r.items() if k not in ("attemptsDetail", "log")}
print(json.dumps(keep, indent=1, default=str)[:4000])
b = pcbnew.LoadBoard(BOARD)                            # re-read what autoroute saved, refill the zones round the new tracks
pcbnew.ZONE_FILLER(b).Fill(b.Zones())
b.Save(BOARD)
print("tracks:", sum(1 for t in b.GetTracks() if t.GetClass() == "PCB_TRACK"), "vias:", sum(1 for t in b.GetTracks() if t.GetClass() == "PCB_VIA"))
