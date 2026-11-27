# UNO Q shield - design spec (2026-11-28, branch unoq-shield-pcb-2026-11-28)

One board that plugs onto the Arduino UNO Q's headers and replaces the Waveshare Bus Servo Adapter (A), the UBEC-8A, the 7Semi
BNO086 breakout + Qwiic cable, the MTS-1 switch wiring and the 3 splices. Designed in KiCad 10 (KiCad MCP server,
`vendor/KiCAD-MCP-Server`), **fabricated and assembled by JLCPCB** (user's choice). In the bay it hangs **under** the upside-down
Uno Q; the bay is redesigned around the Uno + shield stack afterwards.

User decisions (2026-11-28): JLCPCB fab + assembly; the shield takes over the servo bus adapter, the power wiring + switch, the IMU
(chip-down BNO086) and the 8 A servo regulator; it sits under the Uno, bay redesigned. Standing: no fuse / BMS / TVS (owner's
choice, BOM); kids' robot - accuracy matters; minimal parts.

## Blocks

### 1. Battery in + main switch
- **XT30PW-M** (right-angle male, PCB, 15 A) LCSC C431092 - the 3S pack plugs straight in (11.1 V, 9.6-12.6 V).
- High-side **P-FET** load switch, source at the battery: AGM30P05A (C6719407, 30 V, 5.5 mOhm, PDFN5x6) or AO4407A (C16072,
  SOIC-8, hotter). 100 k gate-source, 47 k from the gate to GND through the switch (Vgs about -8.6 V at 12.6 V, -6.5 V at 9.6 V),
  22 nF gate-source (soft turn-on into the input capacitors). The switch carries ~0.1 mA: a small on-board slide switch at the
  board edge AND a 2-pin header in parallel for a panel switch (the bay decides which is used).
- Everything after the switch = **VBAT_SW**: the Uno's VIN, the buck's input, the battery sense.

### 2. Servo supply: 8 A synchronous buck
- First choice **SY8368AQQC** (Silergy, LCSC C207642): 4-28 V in, 8 A continuous / 16 A peak (ILMT), 800 kHz, integrated FETs,
  QFN3x3-12. Alternative **TPS568230RJER** (TI, C2878938): 0.6-7 V out, but its valley current limit is 12 A at most.
- Output **5.2 V** (the servo torque / power model and the UBEC setting; SCS0009 4.8-8.4 V). Silergy's product page lists
  Vout 0.6-5 V although the datasheet states no maximum - 5.2 V to be confirmed against the datasheet's duty / feedback limits
  before layout; 6.0 V only after a bench test (or with the TPS568230).
- Inductor 2.2 uH, Isat >= 20-25 A, DCR <= 5-6 mOhm (Pulse PA4346.222NLT C3221880, or SXN SMMS1050-2R2M C149538).
- Input >= 40 uF X7R 25 V + an electrolytic for damping (no TVS: the XT30 hot-plug rings the ceramics); output 4-6 x 22 uF
  X5R 16 V + a few hundred uF bulk near the servo connectors.
- **4-layer board** (both makers rate their thermal resistance on 4 layers; ~3-4 W loss at 8 A).

### 3. Servo bus (copy of the Waveshare Bus Servo Adapter (A) interface, its schematic read 2026-11-28)
- 3.3 V bus: SN74LVC1G126 (C7834; A = TXD, OE = TXEN, Y = DATA) + SN74LVC1G125 (C23654; A = DATA, /OE = TXEN, Y = RXD),
  10 k pull-ups on DATA and RXD; TXEN made from TXD by an MMBT3906 (emitter 3.3 V, 10 k base from TXD, 10 k base-emitter,
  collector 20 k to GND). No direction pin.
- **Plus a bus enable from a GPIO** (gating the '126's OE): the UNO Q prints its Zephyr boot message on Serial1 (D0/D1) at every
  MCU reset - it must not reach the servos. Default off until the sketch enables it.
- UART: D1 = PB6 USART1_TX -> TXD, D0 = PB7 USART1_RX <- RXD (`Serial1`), 1 Mbps.
- Servo connectors: Molex 5264-type, 2.5 mm, 3-pin, **G-V-S** (pin 1 GND per Seeed's drawing; Waveshare numbers it the other
  way - check against a real cable before ordering). LCSC clone CJT A2501WV-3P (C225389, fit unconfirmed - buy a sample).
  4 connectors: left leg, right leg, neck, spare.

### 4. IMU: BNO086 chip-down (CEVA BNO08X datasheet v1.17, I2C reference)
- I2C on **D20/D21 (PB11/PB10, `Wire`)** - not A4/A5 (a separate bus on the UNO Q). PS0 = PS1 = GND (I2C), SA0 -> 0x4A,
  BOOTN 10 k pull-up (+ GPIO), NRST and H_INTN to GPIOs, 2 x 0.1 uF on VDD / VDDIO, 0.1 uF on CAP, 2.2 k pull-ups on the
  unused ENV_SCL / ENV_SDA, ~2.2-4.7 k on SDA / SCL. No crystal (internal oscillator is allowed in I2C mode).
- Supply 3.3 V from a small local LDO off the Uno's 5 V or from the buck (the UNO Q publishes no current figure for its 3V3
  pin) - decide in the schematic.
- **Placement: as far as possible from the buck inductor, the XT30, the switch FET and the high-current traces; no copper pour
  or traces under it**; near the robot's rotation centre if the layout allows.
- **Sourcing risk:** LCSC C5264422 shows 0 stock / MOQ 458 (~$28); DigiKey ~4,400. JLCPCB global sourcing or consigned parts;
  otherwise fall back to a footprint for the breakout.

### 5. Other
- Battery sense: 100 k / 27 k divider after the switch -> **A2**, 100 nF (2.68 V at 12.6 V; ADC full scale ~15.5 V).
- Uno power: VBAT_SW -> **VIN** (7-24 V). Leave JANALOG pin 1 (**BOOT**) unconnected. Logic is 3.3 V everywhere.
- Outline: UNO R3 shield outline and header positions (the UNO Q's are compatible); bottom parts of the UNO Q stay < 2 mm.

## Pins used (UNO Q)
| Pin | Use |
|---|---|
| D0 (PB7, USART1_RX) | servo bus RXD |
| D1 (PB6, USART1_TX) | servo bus TXD |
| D2 | servo bus enable (high = on) |
| D3 | IMU H_INTN |
| D4 | IMU NRST |
| D5 | IMU BOOTN (firmware update only) |
| D20 / D21 (I2C2) | IMU SDA / SCL |
| A2 | battery voltage |
| VIN | VBAT_SW |
| GND | ground |

## Open before layout
1. Servo voltage 5.2 V (assumed) vs 6.0 V - decides the buck IC.
2. BNO086 sourcing (JLCPCB global sourcing / consignment).
3. 5264 mating header at LCSC + pin-1 orientation (check a real servo cable).
4. Whether the Zephyr core lets Serial1 run quiet / single-wire half-duplex (the bus enable covers it either way).
5. The UNO Q's 3V3 / 5V current budget for the shield (no published figure) -> local LDO for the IMU.

## Sources (2026-11-28)
UNO Q datasheet https://docs.arduino.cc/resources/datasheets/ABX00162-ABX00173-datasheet.pdf ·
pinout https://docs.arduino.cc/resources/pinouts/ABX00162-full-pinout.pdf ·
Serial1 boot output https://forum.arduino.cc/t/uno-q-can-we-use-serial1-as-uart-normally/1448726 ·
Waveshare Bus Servo Adapter (A) schematic https://files.waveshare.com/wiki/Bus-Servo-Adapter-(A)/Bus%20Servo%20Adapter%20(A)-Sch.pdf ·
SC09 servo datasheet https://download.mikroe.com/documents/datasheets/SC09_Servo_datasheet.pdf ·
Seeed 5264 drawing https://files.seeedstudio.com/Bazaar/product_pdf/100067882.pdf ·
BNO086 at JLCPCB https://jlcpcb.com/partdetail/CEVA_TECHNOLOGIES_INC-BNO086/C5264422 ·
SY8368A https://www.lcsc.com/product-detail/C207642.html · TPS568230 https://www.ti.com/lit/ds/symlink/tps568230.pdf ·
XT30PW-M https://lcsc.com/product-detail/plug_Changzhou-Amass-Elec-XT30PW-M_C431092.html

## State 2026-11-28 (schematic + routed 4-layer board, `pcb/unoq_shield/`)

Changes against the plan above (user / design decisions on 2026-11-28):
- **IMU external via Qwiic** (user): the chip-down BNO086 is gone (0 LCSC stock, LGA assembly risk). J11 = JST-SH 4-pin Qwiic
  (SM04B-SRSS-TB, C160404) on I2C2 (D20 SDA / D21 SCL, `Wire`), 3V3 from the shield's XC6206, 2.2 k pull-ups. The 7Semi BNO086
  breakout plugs in with its Qwiic cable (the UNO Q's own Qwiic socket still works too). D3-D5 are free.
- **ILMT floating = 12 A low-side current limit** (was "high" = 16 A): ILMT (pin 10) and VCC (pin 8) sit either side of FB (pin 9)
  in the 0.45 mm row, so tying them crosses FB. 12 A covers walking peaks; 15 stalled SCS0009 ~13-15 A is the only case above.
- **Servo rail 5.2 V** (R3 76.8 k / R4 10 k, Vout = 0.6 x (1 + R3/R4) = 5.21 V); EN from D6 via 1 k with a 100 k pull-down (servos
  unpowered until the sketch enables them); PG -> D7. Servo-bus enable on D2 (PNP emitter): the Zephyr boot text on Serial1 never
  reaches the servos.
- **XT30PW-M polarity: pad 2 = BAT+, pad 1 = GND** (the footprint's own "+" / "-" silk; caught on the 3D render).
- Parts (LCSC): SY8368AQQC C207642, inductor Changjiang FXL1040-2R2-M C167228 (2.2 uH, 18 A sat, 7 mOhm), AO4407A C16072, slide
  switch C&K OS102011MA1QN1 C226259 (right angle, lever out of the edge), servo headers HCTL HC-5264-3A C2845815 (Mini-SPOX
  5264, vertical), XT30PW-M C431092, 74LVC1G126 C7834 / 74LVC1G125 C23654, MMBT3906 C7420354, XC6206P332MR C5446, pin headers
  C492407 (1x8) / C492405 (1x6) / C492409 (1x10), 22 uF 25 V 1206 C12891, 100 uF 25 V C72477, 220 uF 16 V polymer C881921;
  passives JLCPCB basic. Every symbol carries its LCSC field.
- **SY8368A footprint drawn by hand** from Silergy's recommended land pattern (custom polygon pads) - **compare with JLCPCB's
  C207642 footprint before ordering**.

Board: UNO R3 outline (68.58 x 53.34, beveled corners), headers on B.Cu at the official positions (0.000 mm error), 4 x M3 holes
(board-only), all parts on F.Cu. 4 layers: F.Cu parts + BAT+ / VBAT_SW / LX / V_SERVO pours + GND fill; In1 solid GND; In2
V_SERVO under the left half (the servo THT pins pick it up) + GND; B.Cu signals + GND. 343 GND stitching vias, 19 V_SERVO vias,
two 0.3 mm vias in U1's boxed-in GND pads, a locked 0.45 mm LX track through the only gap in the land pattern.
Layout: the input caps straight under U1 across GND pad 3 / IN pin 4 (smallest hot loop), inductor left of U1 (LX exits left),
power entry at the right edge, servo connectors / slide switch / Qwiic at the left edge, bus logic by D0-D2.
Routing: Freerouting 2.5.0 (Java 25+; 2.3.0 ignored the inner-layer restriction), F.Cu + B.Cu only.

Re-run: `tools/kicad/shield_clean.py` -> `shield_layout.py` (KiCad's python) -> `shield_route.py` (MCP venv python) ->
`kicad-cli pcb drc --refill-zones --save-board`.

Checks: ERC 0. DRC: all nets routed, schematic parity 0; left: 2 sub-0.15 mm SERVO_DATA neck-downs at a pad (Freerouting), 1 small
F.Cu GND fragment, H2's courtyard over J6's SCL pin (the UNO's own hole spacing), silkscreen overlaps (warnings).
Not done: those DRC items, silkscreen tidy, JLCPCB outputs (Gerbers, drill, BOM, CPL), the bay redesign round the UNO + shield.
