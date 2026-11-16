"""Stacked-ball dead-weight relief valve, FDM-friendly.

- 1-5 steel balls (15 mm) stack in a loose tube on a 6.5 mm seat edge:
  1 ball ~4 kPa, 2 ~8, 3 ~12, 4 ~16, 5 ~20.
- Loose tube (FDM holes print small) + a 15-degree funnel only at the
  bottom so the lowest ball always rolls back onto the seat.
- Flat bottom, side hose barb: add slicer supports under the barb only.
- One X-shaped base printed flat; the body's bottom plug drops into its hub,
  the hose barb passes through a slot in the hub.

Usage: python3 relief.py <out_dir>   (units: mm, z up)
"""
import math, sys
import manifold3d as m3d
import trimesh

OUT = sys.argv[1] if len(sys.argv) > 1 else "."
SEG = 128

BALL_D = 15.0
MAX_BALLS = 5
TUBE_D = BALL_D + 1.2       # FDM: holes shrink ~0.2-0.4 mm, still free-running
WALL = 2.4
BODY_R = TUBE_D / 2 + WALL
SEAT_D = 5.5                # printed as-is (no drilling); smaller seat -> higher pressure per ball
BORE_D = 5.5                # straight bore, sharp edge at the funnel
FUNNEL_DEG = 15.0           # R*sin(15) = 1.9 < 3.25 -> ball sits on the edge, not the funnel
SEAT_Z = 14.0
LIFT = 5.0
PLUG_LEN = 3.0
FIT = 0.25                  # FDM press fit

# base ring + side barb
RING_R = 14.0              # bottom plug that goes into the X base hub
RING_H = 14.0
BARB_Z = 4.5                # barb axis height
BARB_LEN = 19.0
RIDGE_OFFS = (10.0, 14.5)   # hose ridges, kept clear of the root collar
ROOT_R = 6.0                # collar radius at the ring face
ROOT_LEN = 8.0              # collar tapers to the barb shaft over this length
CH_D = 3.0                  # air channel / barb bore

# X base (one piece, body plugs into its hub)
SLOT_ANGLES = (0, 90, 180, 270)
ARM_ANGLES = (45, 135, 225, 315)
HUB_FIT = 0.25
HUB_WALL = 4.0
HUB_H = 14.0
ARM_LEN = 80.0              # centre to pad centre
ARM_W, ARM_H = 18.0, 6.0
PAD_R = 13.0
BARB_SLOT_W = 14.0          # opening in the hub for the barb collar

R = BALL_D / 2
c0 = math.sqrt(R * R - (SEAT_D / 2) ** 2)
stack_top = SEAT_Z + c0 + R + (MAX_BALLS - 1) * BALL_D
CEIL_Z = stack_top + LIFT
TOP_Z = CEIL_Z + PLUG_LEN

def cyl(h, r1, r2=None, z=0.0):
    r2 = r1 if r2 is None else r2
    return m3d.Manifold.cylinder(h, r1, r2, SEG).translate((0, 0, z))

def box(x, y, z, x0=0.0, cy=0.0, z0=0.0):
    return m3d.Manifold.cube((x, y, z)).translate((x0, cy - y / 2, z0))

def xcyl(length, r1, r2, x0, z):     # cylinder along +x
    return m3d.Manifold.cylinder(length, r1, r2, SEG).rotate((0, 90, 0)).translate((x0, 0, z))

funnel_h = (TUBE_D / 2 - SEAT_D / 2) * math.tan(math.radians(FUNNEL_DEG))

# ---- body ----
body = cyl(RING_H, RING_R) + cyl(TOP_Z, BODY_R)
# side barb along +x, resting on the bed via a rib
bx = RING_R - 1.0
barb = xcyl(BARB_LEN, 2.3, 2.3, bx, BARB_Z)
for off in RIDGE_OFFS:
    barb = barb + xcyl(3.5, 2.75, 2.3, bx + off, BARB_Z)   # ridges: wide toward body
# root reinforcement: tapered collar where the barb meets the plug (no support under the barb; add it in the slicer)
barb = barb + xcyl(ROOT_LEN, ROOT_R, 2.3, bx - 1.0, BARB_Z)
barb = barb ^ box(100, 100, 50, x0=-10, z0=0.0)              # trim anything below the bed
body = body + barb
# air path: barb bore -> centre -> up to seat
body = body - xcyl(bx + BARB_LEN + 1, CH_D / 2, CH_D / 2, 0.0, BARB_Z)
body = body - cyl(SEAT_Z - BARB_Z + 0.1, BORE_D / 2, z=BARB_Z - CH_D / 2)
cham_h = (SEAT_D - BORE_D) / 2
body = body - cyl(cham_h + 0.001, BORE_D / 2, SEAT_D / 2, SEAT_Z - cham_h)
# funnel + tube
body = body - cyl(funnel_h + 0.001, SEAT_D / 2, TUBE_D / 2, SEAT_Z)
body = body - cyl(TOP_Z - SEAT_Z, TUBE_D / 2, z=SEAT_Z + funnel_h)
# vent slots by the bottom ball
for a in SLOT_ANGLES:
    s = box(BODY_R + 2, 4.0, 20.0, x0=0, z0=SEAT_Z + 1.0)
    body = body - s.rotate((0, 0, a))   # between the legs (z above the barb)

# ---- cap (prints flat, plug up) ----
plug_r = (TUBE_D - FIT) / 2
cap = cyl(2.5, BODY_R) + cyl(PLUG_LEN, plug_r, z=2.5)
cap = cap - cyl(PLUG_LEN + 1, plug_r - 1.6, z=2.5)
cap = cap - cyl(10, 3.0, z=-1)

# ---- X base (prints flat) ----
hub_in = RING_R + HUB_FIT
xb = cyl(HUB_H, hub_in + HUB_WALL)
for a in ARM_ANGLES:
    arm = box(ARM_LEN, ARM_W, ARM_H, x0=0.0)
    arm = arm + m3d.Manifold.cylinder(ARM_H, PAD_R, PAD_R, SEG).translate((ARM_LEN, 0, 0))
    xb = xb + arm.rotate((0, 0, a))
xb = xb - cyl(HUB_H + 2, hub_in, z=-1)                              # through socket
xb = xb - box(hub_in + HUB_WALL + 2, BARB_SLOT_W, HUB_H + 2, x0=0.0, z0=-1)  # barb slot at +x

def save(man, name):
    mesh = man.to_mesh()
    tm = trimesh.Trimesh(vertices=mesh.vert_properties[:, :3], faces=mesh.tri_verts)
    tm.export(f"{OUT}/{name}")
    return tm

b = save(body, "relief_base.stl")
c = save(cap, "relief_cap.stl")
x = save(xb, "relief_xbase.stl")

area = math.pi * (SEAT_D / 2) ** 2
g1 = 7.85 * math.pi / 6 * (BALL_D / 10) ** 3
print(f"body watertight={b.is_watertight} height {TOP_Z:.1f} tube ID {TUBE_D:.1f} OD {2*BODY_R:.1f} base ring {2*RING_R:.0f}")
print(f"cap watertight={c.is_watertight} xbase watertight={x.is_watertight} xbase span {x.bounds[1][0]-x.bounds[0][0]:.0f} mm")
print(f"foot reach radius {ARM_LEN + PAD_R:.0f} mm, funnel rise {funnel_h:.2f} mm")
print(f"edge seated: R*sin(funnel)={R*math.sin(math.radians(FUNNEL_DEG)):.2f} < {SEAT_D/2}")
for n in range(1, MAX_BALLS + 1):
    print(f"  {n} ball(s): {n*g1:.1f} g -> {n*g1*9.81/area:.1f} kPa")
