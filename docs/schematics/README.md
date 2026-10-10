# System schematics (DRAFT, 2026-10-10)

Two drawings of Tom's N2 generator, made from the owner's descriptions and the site photos (`docs/photos/`):

| File | What |
|---|---|
| `plumbing_schematic_DRAFT_20261010.pdf` / `.svg` | pneumatic: air in, towers, valves, N2 LOW tank, compressor, N2 HIGH tank, gauges, transducers |
| `electrical_schematic_DRAFT_20261010.pdf` / `.svg` | electrical / electronic: mains, 24 V supply, Arduino pins, switches, sensors, valve and SSR outputs, I2C devices |

Solid lines = verified or seen. **Dashed lines and "?" = assumed or unknown**: Tom, please correct them. Edit `make_schematics.py` (Python 3; `rsvg-convert -f pdf` makes the PDF) and regenerate; do not hand-edit the SVG.

Known unknowns: valve drivers and coil voltages, where the flush valve is, what the pilot regulator solenoid does, how the Arduino is powered, the O2 buffer tank connection, tower inlet/outlet sides.
