"""Stacked-ball dead-weight relief valve, FDM-friendly.

- 1-5 steel balls (15 mm) stack in a loose tube on a 6.5 mm seat edge:
  1 ball ~4 kPa, 2 ~8, 3 ~12, 4 ~16, 5 ~20.
- Loose tube (FDM holes print small) + a 15-degree funnel only at the
  bottom so the lowest ball always rolls back onto the seat.
- Flat bottom, side hose barb lying on the bed: prints upright, no supports.
- Four wide flat legs, printed lying flat, plug into pockets in the base ring.

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
SEAT_D = 6.5                # sharp edge; drill with a 6.5 mm bit after printing
BORE_D = 6.0
FUNNEL_DEG = 15.0           # R*sin(15) = 1.9 < 3.25 -> ball sits on the edge, not the funnel
SEAT_Z = 14.0
LIFT = 5.0
PLUG_LEN = 3.0
FIT = 0.25                  # FDM press fit

# base ring + side barb
RING_R = 18.0
RING_H = 12.0
BARB_Z = 4.5                # barb axis height
BARB_LEN = 19.0
RIDGE_OFFS = (10.0, 14.5)   # hose ridges, kept clear of the root collar
ROOT_R = 6.0                # collar radius at the ring face
ROOT_LEN = 8.0              # collar tapers to the barb shaft over this length
CH_D = 3.0                  # air channel / barb bore

# legs
LEG_ANGLES = (45, 135, 225, 315)
SLOT_ANGLES = (0, 90, 180, 270)
POCKET_W, POCKET_H, POCKET_D = 10.4, 5.4, 10.0
TAB_W, TAB_H, TAB_D = 10.0, 5.0, 10.0
LEG_W, LEG_H, LEG_LEN = 16.0, 5.0, 60.0
PAD_R = 12.0

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
barb = barb + box(BARB_LEN - 1, 4.0, BARB_Z, x0=bx, z0=0.0)   # support rib under barb
# root reinforcement: tapered collar + wide gusset block where the barb meets the ring
barb = barb + xcyl(ROOT_LEN, ROOT_R, 2.3, bx - 1.0, BARB_Z)
barb = barb + box(ROOT_LEN * 0.6, 2 * ROOT_R, BARB_Z, x0=bx - 1.0, z0=0.0)
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
# leg pockets, open at the bottom
for a in LEG_ANGLES:
    p = box(POCKET_D + 1, POCKET_W, POCKET_H + 0.5, x0=RING_R - POCKET_D, z0=-0.5)
    body = body - p.rotate((0, 0, a))

# ---- cap (prints flat, plug up) ----
plug_r = (TUBE_D - FIT) / 2
cap = cyl(2.5, BODY_R) + cyl(PLUG_LEN, plug_r, z=2.5)
cap = cap - cyl(PLUG_LEN + 1, plug_r - 1.6, z=2.5)
cap = cap - cyl(10, 3.0, z=-1)

# ---- leg (prints flat, x 4) ----  tab at x<0 goes into the pocket
leg = box(TAB_D, TAB_W, TAB_H, x0=-TAB_D)
leg = leg + box(LEG_LEN - PAD_R, LEG_W, LEG_H)
leg = leg + m3d.Manifold.cylinder(LEG_H, PAD_R, PAD_R, SEG).translate((LEG_LEN - PAD_R, 0, 0))

def save(man, name):
    mesh = man.to_mesh()
    tm = trimesh.Trimesh(vertices=mesh.vert_properties[:, :3], faces=mesh.tri_verts)
    tm.export(f"{OUT}/{name}")
    return tm

b = save(body, "relief_base.stl")
c = save(cap, "relief_cap.stl")
l = save(leg, "relief_leg.stl")

area = math.pi * (SEAT_D / 2) ** 2
g1 = 7.85 * math.pi / 6 * (BALL_D / 10) ** 3
print(f"body watertight={b.is_watertight} height {TOP_Z:.1f} tube ID {TUBE_D:.1f} OD {2*BODY_R:.1f} base ring {2*RING_R:.0f}")
print(f"cap watertight={c.is_watertight} leg watertight={l.is_watertight} leg {LEG_LEN:.0f}x{LEG_W:.0f}x{LEG_H:.0f}")
print(f"footprint radius {RING_R + LEG_LEN:.0f} mm, funnel rise {funnel_h:.2f} mm")
print(f"edge seated: R*sin(funnel)={R*math.sin(math.radians(FUNNEL_DEG)):.2f} < {SEAT_D/2}")
for n in range(1, MAX_BALLS + 1):
    print(f"  {n} ball(s): {n*g1:.1f} g -> {n*g1*9.81/area:.1f} kPa")
