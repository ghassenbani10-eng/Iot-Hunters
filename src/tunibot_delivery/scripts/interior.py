"""Detailed interiors of the staff kitchen and the customer toilet (imported by build_world.py).

Kitchen layout follows the usual commercial-kitchen flow, one way, no back-tracking:
receiving (staff door, north-east) -> storage (walk-in cooler / freezer, dry goods, north
wall) -> prep (three tables, colour-coded boards, prep + hand sink) -> cooking line (island
under a hood overhanging the appliances by 15 cm) -> the PASS behind the robot's serving
windows (heat lamps, ticket rail, order screen). Dirty dishes take their own route along
the east wall (dish drop -> dishwasher -> clean racks), away from food prep. Aisles are
0.9-1.25 m; there is a hand sink in the prep area, at the pass and in the dish area.

The robot's lidar (18 cm) never sees any of this: the partition is solid below the
window sills. Everything is static; only the large pieces have collision geometry.

All coordinates are world frame, metres. Each object is one link with several visuals.
"""
import math

# kitchen interior faces (walls are 0.15 m thick)
KX0, KX1, KY0, KY1 = -10.415, -2.575, 3.075, 10.34

STEEL = (0.74, 0.76, 0.79)
STEEL_DARK = (0.42, 0.44, 0.47)
IRON = (0.07, 0.07, 0.08)
WHITE = (0.95, 0.95, 0.95)
CERAMIC = (0.97, 0.97, 0.98)
QUARRY_TILE = (0.58, 0.30, 0.22)
WALL_TILE = (0.93, 0.94, 0.93)
WC_FLOOR = (0.80, 0.81, 0.83)
WC_WALL = (0.80, 0.88, 0.93)
WOOD = (0.55, 0.37, 0.20)
CARDBOARD = (0.66, 0.50, 0.32)
BLACK = (0.03, 0.03, 0.03)
RED = (0.78, 0.10, 0.08)
GREEN = (0.15, 0.55, 0.22)
BLUE = (0.12, 0.30, 0.70)
YELLOW = (0.90, 0.75, 0.10)
GLASS = (0.70, 0.82, 0.88)


def _mat(rgb, spec=0.15, emissive=None):
    r, g, b = rgb
    s = '<material><ambient>%.2f %.2f %.2f 1</ambient><diffuse>%.2f %.2f %.2f 1</diffuse>' \
        '<specular>%.2f %.2f %.2f 1</specular>' % (r * 0.6, g * 0.6, b * 0.6, r, g, b, spec, spec, spec)
    if emissive:
        s += '<emissive>%.2f %.2f %.2f 1</emissive>' % emissive
    return s + '</material>'


class Part:
    """One link: visuals (+ optional collision boxes), absolute world poses."""

    def __init__(self, name):
        self.name, self.vis, self.col = name, [], []

    def _v(self, pose, geo, rgb, spec, em):
        self.vis.append("<visual name='%s_v%d'><pose>%s</pose><geometry>%s</geometry>%s</visual>" % (
            self.name, len(self.vis), pose, geo, _mat(rgb, spec, em)))
        return self

    def box(self, x, y, z, sx, sy, sz, rgb, spec=0.15, em=None, yaw=0.0, collide=False):
        """Box given by its centre (x, y) and its bottom z."""
        geo = '<box><size>%.3f %.3f %.3f</size></box>' % (sx, sy, sz)
        if collide:
            self.col.append("<collision name='%s_c%d'><pose>%.3f %.3f %.3f 0 0 %.4f</pose><geometry>%s"
                            "</geometry></collision>" % (self.name, len(self.col), x, y, z + sz / 2, yaw, geo))
        return self._v('%.3f %.3f %.3f 0 0 %.4f' % (x, y, z + sz / 2, yaw), geo, rgb, spec, em)

    def span(self, x0, x1, y0, y1, z0, z1, rgb, spec=0.15, em=None, collide=False):
        """Box given by its extents."""
        return self.box((x0 + x1) / 2, (y0 + y1) / 2, z0, x1 - x0, y1 - y0, z1 - z0, rgb, spec, em,
                        collide=collide)

    def cyl(self, x, y, z, r, length, rgb, axis='z', spec=0.15, em=None):
        """Cylinder: bottom z for a vertical one, centre for a horizontal one (axis x or y)."""
        rot = {'z': '0 0 0', 'x': '0 1.5708 0', 'y': '1.5708 0 0'}[axis]
        zc = z + length / 2 if axis == 'z' else z
        return self._v('%.3f %.3f %.3f %s' % (x, y, zc, rot),
                       '<cylinder><radius>%.3f</radius><length>%.3f</length></cylinder>' % (r, length),
                       rgb, spec, em)

    def sphere(self, x, y, z, r, rgb, spec=0.2):
        return self._v('%.3f %.3f %.3f 0 0 0' % (x, y, z), '<sphere><radius>%.3f</radius></sphere>' % r,
                       rgb, spec, None)

    def xml(self):
        return "\n      <link name='%s'>%s%s\n      </link>" % (
            self.name, ''.join('\n        ' + c for c in self.col), ''.join('\n        ' + v for v in self.vis))


# ------------------------------------------------------------------ helpers
def steel_table(p, x0, x1, y0, y1, h=0.9, shelf=True):
    p.span(x0, x1, y0, y1, h - 0.04, h, STEEL, spec=0.8)
    p.col.append("<collision name='%s_c%d'><pose>%.3f %.3f %.3f 0 0 0</pose><geometry><box><size>"
                 "%.3f %.3f %.3f</size></box></geometry></collision>" % (
                     p.name, len(p.col), (x0 + x1) / 2, (y0 + y1) / 2, h / 2, x1 - x0, y1 - y0, h))
    for lx in (x0 + 0.04, x1 - 0.04):
        for ly in (y0 + 0.04, y1 - 0.04):
            p.cyl(lx, ly, 0.0, 0.02, h - 0.04, STEEL, spec=0.6)
    if shelf:
        p.span(x0 + 0.02, x1 - 0.02, y0 + 0.02, y1 - 0.02, 0.22, 0.24, STEEL, spec=0.6)


def wire_shelving(p, x0, x1, y0, y1, levels=(0.15, 0.6, 1.05, 1.5), h=1.8):
    p.col.append("<collision name='%s_c%d'><pose>%.3f %.3f %.3f 0 0 0</pose><geometry><box><size>"
                 "%.3f %.3f %.3f</size></box></geometry></collision>" % (
                     p.name, len(p.col), (x0 + x1) / 2, (y0 + y1) / 2, h / 2, x1 - x0, y1 - y0, h))
    for px in (x0 + 0.015, x1 - 0.015):
        for py in (y0 + 0.015, y1 - 0.015):
            p.cyl(px, py, 0.0, 0.013, h, STEEL_DARK, spec=0.5)
    for z in levels:
        p.span(x0, x1, y0, y1, z, z + 0.02, STEEL_DARK, spec=0.4)


def plate_stack(p, x, y, z, n, r=0.13):
    p.cyl(x, y, z, r, 0.018 * n, CERAMIC, spec=0.6)


def wall_panel(p, x0, x1, y0, y1, z1, rgb):
    p.span(x0, x1, y0, y1, 0.0, z1, rgb, spec=0.5)


# ------------------------------------------------------------------ kitchen
def kitchen_parts():
    parts = []

    # floor (quarry tiles), drains, anti-fatigue mats, tiled walls (1.8 m)
    p = Part('k_floor')
    p.span(KX0, KX1, KY0, KY1, 0.0, 0.003, QUARRY_TILE, spec=0.3)
    for x, y in ((-6.7, 4.3), (-3.75, 7.2), (-9.7, 6.7)):
        p.span(x - 0.15, x + 0.15, y - 0.15, y + 0.15, 0.003, 0.005, STEEL_DARK, spec=0.6)
    p.span(-8.9, -4.5, 4.12, 4.78, 0.003, 0.016, BLACK)          # cooks' mat
    p.span(-3.98, -3.52, 5.6, 8.6, 0.003, 0.016, BLACK)          # dishwasher's mat
    wall_panel(p, KX0, KX0 + 0.012, KY0, KY1, 1.8, WALL_TILE)
    wall_panel(p, KX0, KX1, KY1 - 0.012, KY1, 1.8, WALL_TILE)
    wall_panel(p, KX1 - 0.012, KX1, KY0, 9.1 - 0.0, 1.8, WALL_TILE)
    parts.append(p)

    # ---- THE PASS: plating counter behind the two robot windows (x -6.5 and -5.0)
    p = Part('k_pass')
    p.span(-7.3, -4.2, KY0, 3.675, 0.0, 0.95, STEEL, spec=0.8, collide=True)
    p.span(-7.3, -4.2, KY0, 3.675, 0.25, 0.27, STEEL_DARK)
    for wx in (-6.5, -5.0):                                            # heat lamps over each window
        p.span(wx - 0.45, wx + 0.45, 3.3, 3.45, 1.62, 1.68, STEEL_DARK)
        p.span(wx - 0.42, wx + 0.42, 3.33, 3.42, 1.60, 1.62, (1.0, 0.45, 0.1), em=(1.0, 0.35, 0.05))
        for k, dx in enumerate((-0.25, 0.22)):                         # plates ready for the robot
            p.cyl(wx + dx, 3.32, 0.95, 0.13, 0.02, CERAMIC, spec=0.6)
            p.cyl(wx + dx, 3.32, 0.97, 0.07, 0.03, ((0.55, 0.30, 0.12), (0.85, 0.65, 0.20))[k])
            p.sphere(wx + dx + 0.04, 3.30, 0.995, 0.025, (0.20, 0.55, 0.15))
    p.span(-7.3, -4.2, 3.12, 3.42, 1.86, 1.89, STEEL, spec=0.8)       # over-shelf with clean plates
    for x in (-7.05, -6.75, -4.75, -4.45):
        plate_stack(p, x, 3.27, 1.89, 9, r=0.12)
    p.span(-7.25, -4.25, 3.64, 3.67, 1.45, 1.47, STEEL, spec=0.8)     # ticket rail (cooks' side)
    for i, x in enumerate((-7.0, -6.6, -6.2, -5.4, -4.9, -4.5)):
        p.span(x - 0.04, x + 0.04, 3.655, 3.66, 1.29, 1.45, WHITE)
    p.span(-5.98, -5.52, 3.5, 3.55, 1.9, 2.18, BLACK)                  # kitchen order screen
    p.span(-5.95, -5.55, 3.555, 3.56, 1.93, 2.15, (0.1, 0.3, 0.6), em=(0.15, 0.35, 0.7))
    parts.append(p)

    # hand sink at the pass (east wall) + takeaway counter details
    p = Part('k_handsink_pass')
    hand_sink(p, KX1, 3.8, 4.25, wall='east')
    p.span(-3.6, -3.3, 3.25, 3.5, 1.0, 1.05, BLACK)                    # POS terminal
    p.box(-3.45, 3.42, 1.05, 0.26, 0.03, 0.2, (0.1, 0.2, 0.4), em=(0.1, 0.25, 0.5), yaw=0.0)
    for i, x in enumerate((-3.05, -2.85)):                            # takeaway bags
        p.span(x - 0.09, x + 0.09, 3.28, 3.45, 1.0, 1.28, CARDBOARD)
    parts.append(p)

    # ---- COOKING LINE (island, cooks face south towards the pass): front y 4.9, back y 5.75
    fy, by = 4.9, 5.75
    p = Part('k_line')
    p.col.append("<collision name='k_line_c0'><pose>-6.7 %.3f 0.45 0 0 0</pose><geometry><box><size>"
                 "4.8 %.3f 0.9</size></box></geometry></collision>" % ((fy + by) / 2, by - fy))
    # charbroiler -9.1..-8.2: glowing coals under a cast-iron grate, two steaks
    p.span(-9.1, -8.2, fy, by, 0.0, 0.84, STEEL, spec=0.7)
    p.span(-9.05, -8.25, fy + 0.08, by - 0.12, 0.84, 0.86, (1.0, 0.3, 0.05), em=(0.9, 0.22, 0.02))
    for i in range(9):
        p.span(-9.03 + i * 0.095, -9.0 + i * 0.095, fy + 0.08, by - 0.12, 0.86, 0.89, IRON)
    for x in (-8.85, -8.5):
        p.span(x - 0.08, x + 0.08, 5.2, 5.38, 0.89, 0.915, (0.40, 0.16, 0.10))
    # 6-burner range with oven -8.2..-7.3
    p.span(-8.2, -7.3, fy, by, 0.0, 0.88, STEEL, spec=0.7)
    p.span(-8.12, -7.38, fy - 0.006, fy, 0.12, 0.62, STEEL_DARK, spec=0.6)            # oven door
    p.cyl(-7.75, fy - 0.04, 0.66, 0.012, 0.6, STEEL, axis='x', spec=0.9)            # door handle
    for i in range(6):
        p.cyl(-8.1 + i * 0.13, fy - 0.02, 0.78, 0.022, 0.03, BLACK, axis='y')        # knobs
    for i, gx in enumerate((-8.05, -7.75, -7.45)):
        for j, gy in enumerate((5.08, 5.45)):
            p.span(gx - 0.13, gx + 0.13, gy - 0.15, gy + 0.15, 0.88, 0.9, IRON)
            lit = (i, j) in ((0, 0), (1, 1))
            p.cyl(gx, gy, 0.9, 0.06, 0.008, (0.2, 0.4, 1.0) if lit else BLACK,
                  em=(0.15, 0.35, 1.0) if lit else None)
    p.cyl(-8.05, 5.08, 0.91, 0.16, 0.26, STEEL, spec=0.9)                            # stock pot
    p.cyl(-7.75, 5.45, 0.91, 0.12, 0.07, STEEL_DARK, spec=0.8)                       # saute pan
    p.cyl(-7.75, 5.25, 0.945, 0.012, 0.22, BLACK, axis='y')                          # pan handle
    # flat-top griddle -7.3..-6.4 with burgers
    p.span(-7.3, -6.4, fy, by, 0.0, 0.86, STEEL, spec=0.7)
    p.span(-7.25, -6.45, fy + 0.1, by - 0.05, 0.86, 0.885, IRON, spec=0.5)
    p.span(-7.25, -6.45, fy + 0.02, fy + 0.1, 0.84, 0.86, STEEL_DARK)                # grease trough
    p.span(-7.3, -7.27, fy + 0.1, by, 0.86, 1.0, STEEL)
    p.span(-6.43, -6.4, fy + 0.1, by, 0.86, 1.0, STEEL)
    for x, y in ((-7.1, 5.2), (-6.9, 5.25), (-6.7, 5.15), (-6.9, 5.5)):
        p.cyl(x, y, 0.885, 0.055, 0.02, (0.35, 0.18, 0.10))
    # two fryers -6.4..-5.6, oil wells and baskets
    for i, x0 in enumerate((-6.4, -6.0)):
        p.span(x0, x0 + 0.4, fy, by, 0.0, 0.9, STEEL, spec=0.7)
        p.span(x0 + 0.05, x0 + 0.35, fy + 0.15, by - 0.15, 0.89, 0.9, (0.55, 0.40, 0.08), em=(0.25, 0.17, 0.0))
        # wire basket lowered in the oil: rim, bottom, handle towards the cook
        bx0, bx1, by0, by1 = x0 + 0.08, x0 + 0.32, fy + 0.2, by - 0.25
        for ex0, ex1, ey0, ey1 in ((bx0, bx1, by0, by0 + 0.01), (bx0, bx1, by1 - 0.01, by1),
                                   (bx0, bx0 + 0.01, by0, by1), (bx1 - 0.01, bx1, by0, by1)):
            p.span(ex0, ex1, ey0, ey1, 0.87, 0.94, STEEL_DARK, spec=0.4)
        for k in range(7):                                                             # fries
            p.box(bx0 + 0.04 + (k % 4) * 0.055, by0 + 0.08 + (k // 4) * 0.08, 0.9, 0.012, 0.1, 0.012,
                  (0.93, 0.78, 0.30), yaw=0.3 * (k % 3 - 1))
        p.cyl(x0 + 0.2, fy + 0.1, 0.97, 0.008, 0.22, BLACK, axis='y')
    # fry dump with its own heat lamp -5.6..-5.1
    p.span(-5.6, -5.1, fy, by, 0.0, 0.88, STEEL, spec=0.7)
    p.span(-5.55, -5.15, fy + 0.1, by - 0.1, 0.86, 0.885, STEEL_DARK, spec=0.8)          # dump pan
    for k in range(18):                                                               # fries
        p.box(-5.5 + (k % 6) * 0.06, fy + 0.18 + (k // 6) * 0.17, 0.885 + 0.01 * (k % 2), 0.012, 0.11, 0.012,
              (0.93, 0.78, 0.30), yaw=0.5 * ((k * 7) % 5 - 2))
    p.span(-5.55, -5.15, 5.25, 5.4, 1.5, 1.55, (1.0, 0.45, 0.1), em=(1.0, 0.35, 0.05))
    # steam table (bain-marie) -5.1..-4.3: rice, tomato sauce, beans, greens
    p.span(-5.1, -4.3, fy, by, 0.0, 0.88, STEEL, spec=0.7)
    for k, col in enumerate(((0.96, 0.95, 0.88), (0.75, 0.12, 0.06), (0.40, 0.22, 0.10), (0.25, 0.55, 0.15))):
        x = -5.0 + (k % 2) * 0.4
        y = 5.12 + (k // 2) * 0.32
        p.span(x - 0.15, x + 0.15, y - 0.13, y + 0.13, 0.86, 0.88, STEEL_DARK)
        p.span(x - 0.13, x + 0.13, y - 0.11, y + 0.11, 0.88, 0.885, col)
    # back over-shelf with squeeze bottles and plates, the whole line length
    p.span(-9.1, -4.3, 5.6, 5.75, 0.9, 1.3, STEEL, spec=0.7)
    p.span(-9.1, -4.3, 5.55, 5.75, 1.3, 1.32, STEEL, spec=0.8)
    for i, x in enumerate((-8.9, -8.8, -6.3, -6.2, -4.6, -4.5)):
        p.cyl(x, 5.66, 1.32, 0.03, 0.2, (RED, YELLOW)[i % 2])
    plate_stack(p, -7.0, 5.65, 1.32, 10, r=0.11)
    plate_stack(p, -5.4, 5.65, 1.32, 10, r=0.11)
    parts.append(p)

    # island hood: 15 cm overhang on every side, baffle filters, lights, duct, fire suppression
    p = Part('k_hood')
    hx0, hx1, hy0, hy1 = -9.25, -4.15, fy - 0.15, by + 0.15
    p.span(hx0, hx1, hy0, hy1, 2.0, 2.45, STEEL, spec=0.8)
    for i in range(16):
        x = hx0 + 0.2 + i * 0.31
        p.span(x, x + 0.22, hy0 + 0.25, hy1 - 0.25, 1.99, 2.0, STEEL_DARK)
    for x in (-8.6, -7.3, -6.0, -4.8):
        p.span(x - 0.15, x + 0.15, hy0 + 0.05, hy0 + 0.18, 1.985, 2.0, WHITE, em=(0.9, 0.9, 0.85))
    p.cyl(-6.7, 5.32, 2.45, 0.25, 0.4, STEEL, spec=0.7)
    p.cyl(-6.7, hy0 + 0.1, 2.47, 0.015, hx1 - hx0, RED, axis='x')
    parts.append(p)

    # ---- PREP: three steel tables, colour-coded boards (green veg, red meat, blue fish, white bakery)
    py0, py1 = 6.9, 7.65
    p = Part('k_prep_veg')
    steel_table(p, -8.9, -7.3, py0, py1)
    p.span(-8.75, -8.25, 7.05, 7.4, 0.9, 0.92, GREEN)
    p.span(-8.6, -8.35, 7.3, 7.33, 0.92, 0.93, STEEL, spec=1.0)                       # chef knife
    for i, (x, y) in enumerate(((-8.05, 7.1), (-7.97, 7.17), (-8.12, 7.18), (-7.9, 7.06))):
        p.sphere(x, y, 0.94, 0.038, (0.85, 0.12, 0.08))                               # tomatoes
    p.sphere(-7.6, 7.35, 0.98, 0.08, (0.35, 0.65, 0.20))                              # lettuce
    p.sphere(-7.6, 7.12, 0.94, 0.04, (0.90, 0.85, 0.65))                              # onion
    p.cyl(-7.8, 7.45, 0.9, 0.14, 0.09, STEEL, spec=0.9)                               # mixing bowl
    for x in (-8.6, -8.1, -7.6):
        p.cyl(x, 7.27, 0.24, 0.12, 0.2, WHITE)                                        # food tubs
    parts.append(p)

    p = Part('k_prep_meat')
    steel_table(p, -7.1, -5.5, py0, py1)
    p.span(-7.0, -6.5, 7.05, 7.4, 0.9, 0.92, RED)
    p.span(-6.9, -6.65, 7.15, 7.3, 0.92, 0.95, (0.55, 0.12, 0.10))                    # beef
    p.span(-6.4, -5.95, 7.05, 7.4, 0.9, 0.92, BLUE)
    p.span(-6.32, -6.02, 7.18, 7.28, 0.92, 0.95, (0.78, 0.80, 0.82), spec=0.6)        # fish
    p.span(-5.9, -5.6, 7.1, 7.5, 0.9, 1.02, STEEL, spec=0.8)                          # slicer base
    p.cyl(-5.75, 7.15, 1.12, 0.12, 0.02, STEEL_DARK, axis='y', spec=0.9)              # slicer blade
    parts.append(p)

    p = Part('k_prep_pastry')
    steel_table(p, -5.3, -4.1, py0, py1)
    p.span(-5.2, -4.75, 7.05, 7.4, 0.9, 0.92, WHITE)
    p.sphere(-4.95, 7.22, 0.97, 0.07, (0.93, 0.85, 0.65))                             # dough
    p.span(-4.55, -4.2, 7.05, 7.5, 0.9, 1.0, WHITE)                                   # stand mixer
    p.span(-4.48, -4.32, 7.38, 7.5, 1.0, 1.45, WHITE)
    p.span(-4.48, -4.25, 7.05, 7.5, 1.38, 1.5, WHITE)
    p.cyl(-4.4, 7.2, 1.0, 0.12, 0.2, STEEL, spec=0.9)
    parts.append(p)

    # prep sink (two bowls + drainboard) and hand sink on the west wall
    p = Part('k_prep_sink')
    sx1 = KX0 + 0.6
    p.span(KX0, sx1, 6.0, 7.4, 0.0, 0.9, STEEL, spec=0.6, collide=True)
    for y in (6.4, 6.95):
        p.span(KX0 + 0.08, sx1 - 0.08, y - 0.22, y + 0.22, 0.86, 0.905, STEEL_DARK, spec=0.8)
        p.cyl(KX0 + 0.06, y, 0.9, 0.015, 0.4, STEEL, spec=1.0)                       # gooseneck
        p.cyl(KX0 + 0.16, y, 1.3, 0.012, 0.22, STEEL, axis='x', spec=1.0)
    p.span(KX0, sx1, 6.0, 6.15, 0.9, 1.15, STEEL, spec=0.8)                           # splash guard
    hand_sink(p, KX0, 7.5, 7.95, wall='west')
    p.span(KX0, KX0 + 0.02, 7.55, 7.9, 1.55, 1.8, BLUE)                               # "wash hands" sign
    parts.append(p)

    # reach-in refrigerator (glass doors) next to the line, K-class extinguisher, bins
    p = Part('k_reachin')
    p.span(KX0, KX0 + 0.8, 3.15, 4.45, 0.0, 2.0, STEEL, spec=0.7, collide=True)
    for y0, y1 in ((3.2, 3.78), (3.82, 4.4)):
        p.span(KX0 + 0.8, KX0 + 0.81, y0, y1, 0.25, 1.85, GLASS, spec=0.9, em=(0.08, 0.1, 0.12))
        p.span(KX0 + 0.81, KX0 + 0.84, (y0 + y1) / 2 - 0.01, (y0 + y1) / 2 + 0.01, 0.9, 1.4, STEEL_DARK)
    p.span(KX0 + 0.8, KX0 + 0.81, 3.2, 4.4, 1.9, 1.97, (0.1, 0.1, 0.1), em=(0.0, 0.25, 0.05))
    p.cyl(KX0 + 0.1, 5.2, 0.0, 0.075, 0.55, RED, spec=0.6)                           # K-class extinguisher
    p.span(KX0 + 0.01, KX0 + 0.02, 5.1, 5.3, 1.1, 1.4, RED)
    p.cyl(KX0 + 0.3, 4.85, 0.0, 0.22, 0.75, (0.30, 0.32, 0.34))                       # waste
    p.cyl(KX0 + 0.3, 5.75, 0.0, 0.22, 0.75, (0.10, 0.45, 0.20))                       # food waste/compost
    parts.append(p)

    # ---- STORAGE: walk-in cooler and freezer (north-west), dry goods (north wall)
    for name, x0, x1, led in (('k_walkin_cooler', KX0, -8.0, (0.0, 0.8, 0.2)),
                              ('k_walkin_freezer', -7.9, -6.2, (0.2, 0.5, 1.0))):
        p = Part(name)
        p.span(x0, x1, 8.6, KY1, 0.0, 2.3, WHITE, spec=0.3, collide=True)
        dx = (x0 + x1) / 2
        p.span(dx - 0.45, dx + 0.45, 8.585, 8.6, 0.05, 2.0, STEEL, spec=0.8)            # door
        p.span(dx + 0.25, dx + 0.32, 8.54, 8.585, 0.9, 1.3, STEEL_DARK, spec=0.9)      # handle
        p.span(dx - 0.55, dx - 0.45, 8.585, 8.6, 0.0, 2.05, STEEL_DARK)                 # hinge jamb
        p.span(dx + 0.55, dx + 0.75, 8.59, 8.6, 1.95, 2.05, BLACK)                      # thermometer
        p.span(dx + 0.57, dx + 0.73, 8.588, 8.59, 1.97, 2.03, led, em=led)
        p.span(x0 + 0.2, x0 + 1.1, 9.0, 9.8, 2.3, 2.6, (0.6, 0.62, 0.64))               # condensing unit
        parts.append(p)

    p = Part('k_dry_storage')
    wire_shelving(p, -6.0, -4.0, 9.84, KY1)
    goods = [(0.15, [((0.90, 0.85, 0.70), 0.40, 0.25)] * 4),                              # flour/rice sacks
             (0.6, [(CARDBOARD, 0.30, 0.30)] * 5),
             (1.05, [((0.75, 0.10, 0.08), 0.10, 0.14)] * 12),                             # tomato tins
             (1.5, [(YELLOW, 0.14, 0.30)] * 6)]                                           # oil jugs
    for z, items in goods:
        x = -5.95
        for rgb, w, h in items:
            if x + w > -4.05:
                break
            if w <= 0.14 and h <= 0.14:
                p.cyl(x + w / 2, 10.05, z + 0.02, w / 2, h, rgb, spec=0.6)
            else:
                p.span(x, x + w, 9.9, 10.28, z + 0.02, z + 0.02 + h, rgb)
            x += w + 0.04
    p.cyl(-5.0, KY1 - 0.01, 2.1, 0.15, 0.02, WHITE, axis='y')                            # clock
    p.span(-5.005, -4.995, KY1 - 0.03, KY1 - 0.02, 2.1, 2.2, BLACK)
    parts.append(p)

    # receiving: platform scale and clipboard by the staff door (the door itself stays clear)
    p = Part('k_receiving')
    p.span(-4.05, -3.5, 9.75, 10.3, 0.0, 0.08, STEEL_DARK, spec=0.6)
    p.span(-3.8, -3.75, 10.2, 10.25, 0.08, 1.1, STEEL_DARK)
    p.span(-3.88, -3.67, 10.17, 10.27, 1.1, 1.25, BLACK)
    p.span(-3.86, -3.69, 10.165, 10.17, 1.13, 1.22, (0.1, 0.5, 0.2), em=(0.05, 0.4, 0.1))
    p.cyl(KX1 - 0.09, 8.95, 0.0, 0.08, 0.5, RED, spec=0.6)                              # extinguisher
    p.span(KX1 - 0.02, KX1, 8.88, 9.02, 1.3, 1.45, RED)                                 # fire alarm pull
    parts.append(p)

    # ---- DISH STATION along the east wall: drop (north) -> dishwasher -> clean racks (south)
    dx0 = -3.3
    p = Part('k_dish')
    steel_table(p, dx0, KX1, 7.6, 8.8, shelf=False)
    p.span(dx0 + 0.1, KX1 - 0.1, 7.75, 8.25, 0.86, 0.905, STEEL_DARK, spec=0.8)         # sink / disposal
    p.cyl(KX1 - 0.12, 8.0, 0.9, 0.018, 0.75, STEEL, spec=1.0)                           # pre-rinse unit
    p.cyl(KX1 - 0.12, 8.0, 1.65, 0.035, 0.25, STEEL_DARK)
    p.cyl(KX1 - 0.3, 8.0, 1.5, 0.012, 0.36, STEEL, axis='x', spec=1.0)
    for y in (8.4, 8.65):                                                               # bus tubs, dirty plates
        p.span(dx0 + 0.08, KX1 - 0.08, y - 0.11, y + 0.11, 0.9, 1.05, (0.35, 0.36, 0.38))
        p.cyl(-2.95, y, 1.0, 0.11, 0.06, CERAMIC)
    p.span(dx0, KX1, 6.8, 7.6, 0.0, 1.45, STEEL, spec=0.8, collide=True)               # door-type dishwasher
    p.span(dx0 + 0.05, KX1 - 0.05, 6.85, 7.55, 1.45, 1.95, STEEL, spec=0.8)
    p.span(dx0 - 0.006, dx0, 6.85, 7.55, 0.85, 1.42, STEEL_DARK, spec=0.8)              # lift door
    p.span(dx0 - 0.04, dx0 - 0.006, 6.9, 7.5, 1.30, 1.34, STEEL, spec=1.0)              # door handle
    p.span(dx0 - 0.006, dx0, 6.85, 7.55, 0.15, 0.6, STEEL_DARK, spec=0.6)               # tank panel
    p.span(dx0 - 0.005, dx0, 6.9, 7.1, 1.6, 1.75, BLACK)
    p.span(dx0 - 0.007, dx0 - 0.005, 6.93, 7.07, 1.63, 1.72, (0.1, 0.8, 0.2), em=(0.05, 0.6, 0.1))
    steel_table(p, dx0, KX1, 5.6, 6.8, shelf=True)                                      # clean table
    for y in (5.85, 6.4):
        p.span(-3.2, -2.68, y - 0.25, y + 0.25, 0.9, 1.0, (0.45, 0.47, 0.50))           # dish racks
        for k in range(5):
            p.span(-3.15 + k * 0.1, -3.14 + k * 0.1, y - 0.12, y + 0.12, 0.92, 1.15, CERAMIC)
    wire_shelving(p, dx0 + 0.1, KX1, 4.35, 5.5)                                          # clean plates, glasses
    for z in (0.17, 0.62, 1.07):
        for y in (4.6, 5.0, 5.3):
            plate_stack(p, -2.95, y, z, 7, r=0.12)
    for k in range(6):
        p.cyl(-3.1 + (k % 2) * 0.25, 4.5 + (k // 2) * 0.35, 1.52, 0.04, 0.13, GLASS, spec=1.0)
    parts.append(p)
    return parts


def hand_sink(p, wall_x, y0, y1, wall='east'):
    d = 0.38
    x0, x1 = (wall_x - d, wall_x) if wall == 'east' else (wall_x, wall_x + d)
    p.span(x0, x1, y0, y1, 0.75, 0.9, STEEL, spec=0.8)
    p.span(x0 + 0.05, x1 - 0.05, y0 + 0.05, y1 - 0.05, 0.86, 0.905, STEEL_DARK, spec=0.9)
    xw = x1 - 0.04 if wall == 'east' else x0 + 0.04
    p.cyl(xw, (y0 + y1) / 2, 0.9, 0.012, 0.22, STEEL, spec=1.0)
    p.span(min(xw, wall_x) - 0.01, max(xw, wall_x) + 0.01, y0 - 0.1, y0 - 0.02, 1.15, 1.35, WHITE)  # soap
    p.span(min(xw, wall_x) - 0.03, max(xw, wall_x) + 0.03, y1 + 0.02, y1 + 0.3, 1.2, 1.5, WHITE)    # towels


# ------------------------------------------------------------------ customer toilet
def toilet_parts(x0, x1, y0, y_mid, y1, door, niches=()):
    """x0..x1 interior, lobby y0..y_mid (wall), WC compartment y_mid..y1; door = (cx, w);
    niches: (x, y_from, y_to) of the corridor niches whose backs bulge into the toilet."""
    parts = []
    p = Part('wc_finish')
    p.span(x0, x1, y0, y1, 0.0, 0.004, WC_FLOOR, spec=0.4)
    for nx, ny0, ny1 in niches:                    # corridor niches bulging in: tiled on this side
        if ny0 < y1 and ny1 > y0:
            p.span(x0 + 0.005, nx + 0.085, max(y0, ny0 - 0.085), min(y1, ny1 + 0.085), 0.0, 2.505, WC_WALL, spec=0.5)
    wall_panel(p, x0, x0 + 0.012, y0, y1, 2.0, WC_WALL)
    wall_panel(p, x1 - 0.012, x1, y0, y1, 2.0, WC_WALL)
    wall_panel(p, x0, x1, y1 - 0.012, y1, 2.0, WC_WALL)
    for i in range(int((y1 - y0) / 0.3)):                                          # floor tile joints
        p.span(x0, x1, y0 + (i + 1) * 0.3 - 0.003, y0 + (i + 1) * 0.3, 0.004, 0.005, (0.6, 0.6, 0.62))
    p.span(x0 + 0.1, x1 - 0.1, y0 + 0.4, y0 + 0.5, 2.3, 2.32, WHITE, em=(0.9, 0.9, 0.9))   # light strip
    p.span(x0 + 0.1, x1 - 0.1, y1 - 0.6, y1 - 0.5, 2.3, 2.32, WHITE, em=(0.9, 0.9, 0.9))
    parts.append(p)

    # entrance door, standing open against the east wall (hinge on the east jamb)
    dx0, dx1 = door[0] - door[1] / 2, door[0] + door[1] / 2
    p = Part('wc_doors')
    p.span(x1 - 0.06, x1 - 0.02, y0 + 0.02, y0 + 0.02 + (dx1 - dx0) - 0.04, 0.0, 2.05, WOOD, spec=0.3)
    p.span(x1 - 0.1, x1 - 0.06, y0 + 0.7, y0 + 0.76, 1.0, 1.04, STEEL, spec=1.0)          # lever handle
    # compartment door (0.7 m), open into the compartment against the east wall, with a lock
    p.span(x1 - 0.06, x1 - 0.02, y_mid + 0.1, y_mid + 0.75, 0.0, 2.0, WOOD, spec=0.3)
    p.span(x1 - 0.1, x1 - 0.06, y_mid + 0.68, y_mid + 0.72, 0.98, 1.03, (0.8, 0.1, 0.1))
    parts.append(p)

    # lobby: vanity + basin + mirror on the west wall, hand dryer on the east wall
    p = Part('wc_lobby')
    vy0, vy1 = y0 + 0.45, y0 + 1.05
    p.span(x0, x0 + 0.42, vy0, vy1, 0.0, 0.83, (0.35, 0.25, 0.18), spec=0.2, collide=True)
    p.span(x0, x0 + 0.44, vy0 - 0.02, vy1 + 0.02, 0.83, 0.86, (0.25, 0.25, 0.27), spec=0.7)
    p.cyl(x0 + 0.24, (vy0 + vy1) / 2, 0.82, 0.17, 0.05, CERAMIC, spec=0.8)
    p.cyl(x0 + 0.06, (vy0 + vy1) / 2, 0.86, 0.015, 0.18, STEEL, spec=1.0)
    p.cyl(x0 + 0.13, (vy0 + vy1) / 2, 1.03, 0.012, 0.15, STEEL, axis='x', spec=1.0)
    p.span(x0 + 0.012, x0 + 0.02, vy0, vy1, 1.05, 1.75, GLASS, spec=1.0, em=(0.1, 0.12, 0.14))  # mirror
    p.span(x0 + 0.012, x0 + 0.08, vy1 + 0.05, vy1 + 0.15, 1.1, 1.3, WHITE)               # soap
    p.span(x1 - 0.2, x1 - 0.012, y0 + 1.2, y0 + 1.5, 1.1, 1.4, WHITE, spec=0.6)          # hand dryer
    p.span(x0 + 0.012, x0 + 0.02, y0 + 0.08, y0 + 0.36, 1.45, 1.75, BLUE)                # notice
    parts.append(p)

    # WC compartment: wall-hung pan, cistern, grab bar, paper, brush, bin, coat hook
    p = Part('wc_compartment')
    cx = (x0 + x1) / 2 - 0.05
    p.span(cx - 0.19, cx + 0.19, y1 - 0.72, y1 - 0.17, 0.0, 0.40, CERAMIC, spec=0.8, collide=True)
    p.span(cx - 0.2, cx + 0.2, y1 - 0.74, y1 - 0.17, 0.40, 0.43, WHITE, spec=0.5)         # seat
    p.span(cx - 0.21, cx + 0.21, y1 - 0.2, y1 - 0.01, 0.4, 1.05, CERAMIC, spec=0.8, collide=True)
    p.span(cx - 0.05, cx + 0.05, y1 - 0.205, y1 - 0.2, 0.92, 0.98, STEEL, spec=1.0)      # flush plate
    p.cyl(x0 + 0.05, y1 - 0.45, 0.85, 0.016, 0.7, STEEL, axis='y', spec=1.0)             # grab bar
    for y in (y1 - 0.8, y1 - 0.1):
        p.span(x0 + 0.012, x0 + 0.05, y - 0.02, y + 0.02, 0.82, 0.88, STEEL)
    p.cyl(x1 - 0.08, y1 - 0.75, 0.7, 0.055, 0.1, WHITE, axis='y')                         # toilet paper
    p.span(x1 - 0.03, x1 - 0.012, y1 - 0.82, y1 - 0.68, 0.68, 0.72, STEEL, spec=1.0)
    p.cyl(x0 + 0.2, y1 - 0.12, 0.0, 0.05, 0.35, STEEL, spec=0.8)                          # brush
    p.cyl(x1 - 0.18, y1 - 0.18, 0.0, 0.1, 0.3, STEEL, spec=0.8)                           # bin
    p.span(x1 - 0.05, x1 - 0.012, y_mid + 1.0, y_mid + 1.05, 1.65, 1.7, STEEL)           # coat hook
    parts.append(p)
    return parts
