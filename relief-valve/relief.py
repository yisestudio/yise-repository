"""Universal dead-weight ball relief valve for 4mm-ID silicone airline.

Swap the steel ball (10-25 mm) to change the opening pressure.
The ball always seats on the same sharp 5.0 mm edge, so the effective
area is fixed and pressure scales with ball weight. A shallow 8-degree
funnel floor rolls a bounced ball back onto the seat.

Usage: python3 relief.py <out_dir>   (units: mm, z up)
"""
import math, sys
import manifold3d as m3d
import trimesh

OUT = sys.argv[1] if len(sys.argv) > 1 else "."
SEG = 128

BALL_MAX = 25.0
SEAT_D = 5.0            # sharp seat edge -> effective area
BORE_D = 4.4            # hole below the 45-degree chamfer
BARB_BORE_D = 3.0
CHAMBER_D = BALL_MAX + 1.8
WALL = 2.6
BODY_R = CHAMBER_D / 2 + WALL
BARB_TOP = 13.0
SEAT_Z = 18.0           # seat plane
FUNNEL_DEG = 8.0        # must satisfy R*sin(slope) < seat radius for the largest ball
CEIL_ABOVE_SEAT = 30.0  # largest ball top at rest ~24.75 -> ~5 mm lift
PLUG_LEN = 3.0
TOP_Z = SEAT_Z + CEIL_ABOVE_SEAT + PLUG_LEN
FIT = 0.10

def cyl(h, r1, r2=None, z=0.0):
    r2 = r1 if r2 is None else r2
    return m3d.Manifold.cylinder(h, r1, r2, SEG).translate((0, 0, z))

funnel_h = (CHAMBER_D / 2 - SEAT_D / 2) * math.tan(math.radians(FUNNEL_DEG))

# ---- base ----
base = cyl(BARB_TOP, 2.3)                                   # barb shaft OD 4.6
for z in (2.0, 6.5):
    base = base + cyl(3.5, 2.3, 2.75, z)                    # barb ridges OD 5.5
base = base + cyl(TOP_Z - BARB_TOP, BODY_R, z=BARB_TOP)     # body
base = base - cyl(TOP_Z - SEAT_Z, CHAMBER_D / 2, z=SEAT_Z + funnel_h)       # chamber
base = base - cyl(funnel_h + 0.001, SEAT_D / 2, CHAMBER_D / 2, SEAT_Z)     # 8-degree funnel floor
base = base - cyl(BARB_TOP + 1, BARB_BORE_D / 2, z=-0.5)    # barb bore
base = base - cyl(SEAT_Z - BARB_TOP + 0.2, BORE_D / 2, z=BARB_TOP - 0.1)   # seat bore
cham_h = (SEAT_D - BORE_D) / 2
base = base - cyl(cham_h + 0.001, BORE_D / 2, SEAT_D / 2, SEAT_Z - cham_h)  # 45-degree chamfer -> sharp 5.0 edge
for a in range(4):                                          # vent slots
    slot = m3d.Manifold.cube((BODY_R + 2, 4.0, 22.0)).translate((0, -2.0, SEAT_Z + 4.0))
    base = base - slot.rotate((0, 0, a * 90 + 45))

# ---- cap ----
plug_r = (CHAMBER_D - FIT) / 2
cap = cyl(2.5, BODY_R) + cyl(PLUG_LEN, plug_r, z=2.5)
cap = cap - cyl(PLUG_LEN + 1, plug_r - 1.6, z=2.5)
cap = cap - cyl(10, 3.0, z=-1)

def save(man, name):
    mesh = man.to_mesh()
    tm = trimesh.Trimesh(vertices=mesh.vert_properties[:, :3], faces=mesh.tri_verts)
    tm.export(f"{OUT}/{name}")
    return tm

b = save(base, "relief_base.stl")
c = save(cap, "relief_cap.stl")

area = math.pi * (SEAT_D / 2) ** 2
ceiling = SEAT_Z + funnel_h * 0 + CEIL_ABOVE_SEAT
print(f"base watertight={b.is_watertight} size={2*BODY_R:.1f} x {TOP_Z:.1f} mm, chamber ID {CHAMBER_D:.1f}")
print(f"cap watertight={c.is_watertight}, funnel rise {funnel_h:.2f} mm")
print("ball_mm  g(carbon)  kPa   edge_seated  lift_mm")
for d in (10, 12, 14, 16, 18, 20, 22, 24, 25):
    R = d / 2
    g = 7.85 * math.pi / 6 * (d / 10) ** 3
    kpa = g * 9.81 / area
    seated = R * math.sin(math.radians(FUNNEL_DEG)) < SEAT_D / 2
    top = SEAT_Z + math.sqrt(R * R - (SEAT_D / 2) ** 2) + R
    print(f"{d:>6}  {g:8.1f}  {kpa:5.1f}  {seated!s:>11}  {ceiling - top:6.1f}")
