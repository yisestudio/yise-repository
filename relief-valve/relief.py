"""Dead-weight ball relief valve for 4mm-ID silicone airline. Units: mm, z up."""
import math, sys
import manifold3d as m3d
import numpy as np
import trimesh

OUT = sys.argv[1]
SEG = 96

BALL_D = 18.0          # steel ball
SEAT_D = 5.0           # sharp seat edge diameter -> effective area
BORE_D = 4.4           # hole below the chamfer
BARB_BORE_D = 3.0
BODY_R = 13.0
CHAMBER_D = 19.8       # ball + 1.8 clearance
FLOOR_Z0, FLOOR_Z1 = 13.0, 18.0   # seat plane at z=18
TOP_Z = 44.0
PLUG_LEN = 3.0
FIT = 0.10             # plug OD = chamber ID - FIT

def cyl(h, r1, r2=None, z=0.0):
    r2 = r1 if r2 is None else r2
    return m3d.Manifold.cylinder(h, r1, r2, SEG).translate((0, 0, z))

# ---- base ----
base = cyl(FLOOR_Z0, 2.3)                                  # barb shaft OD 4.6
for z in (2.0, 6.5):                                       # two barb ridges
    base = base + cyl(3.5, 2.3, 2.75, z)
base = base + cyl(TOP_Z - FLOOR_Z0, BODY_R, z=FLOOR_Z0)    # body
base = base - cyl(TOP_Z - FLOOR_Z1 + 1, CHAMBER_D / 2, z=FLOOR_Z1)   # ball chamber
base = base - cyl(FLOOR_Z0 + 1, BARB_BORE_D / 2, z=-0.5)   # barb bore
base = base - cyl(FLOOR_Z1 - FLOOR_Z0 + 0.2, BORE_D / 2, z=FLOOR_Z0 - 0.1)  # seat bore
cham_h = (SEAT_D - BORE_D) / 2                             # 45 deg chamfer up to seat edge
base = base - cyl(cham_h + 0.001, BORE_D / 2, SEAT_D / 2, FLOOR_Z1 - cham_h)
for a in range(4):                                         # vent slots
    slot = m3d.Manifold.cube((BODY_R + 2, 4.0, 16.0)).translate((0, -2.0, 22.0))
    base = base - slot.rotate((0, 0, a * 90 + 45))

# ---- cap ----
cap = cyl(2.5, BODY_R)                                     # disc
cap = cap + cyl(PLUG_LEN, (CHAMBER_D - FIT) / 2, z=2.5)    # press-fit plug
cap = cap - cyl(PLUG_LEN + 1, (CHAMBER_D - FIT) / 2 - 1.6, z=2.5)  # hollow plug
cap = cap - cyl(10, 3.0, z=-1)                             # center vent

def save(man, name):
    mesh = man.to_mesh()
    tm = trimesh.Trimesh(vertices=mesh.vert_properties[:, :3], faces=mesh.tri_verts)
    tm.export(f"{OUT}/{name}")
    return tm

b = save(base, "relief_base.stl")
c = save(cap, "relief_cap.stl")

# checks
seat_z = FLOOR_Z1
r = BALL_D / 2
center = seat_z + math.sqrt(r**2 - (SEAT_D / 2) ** 2)
ceiling = TOP_Z - PLUG_LEN
area = math.pi * (SEAT_D / 2) ** 2
ball_g = 7.85 * math.pi / 6 * (BALL_D / 10) ** 3
print("base watertight", b.is_watertight, "vol cm3", round(b.volume / 1000, 2), "bbox", b.bounds.round(2).tolist())
print("cap watertight", c.is_watertight, "vol cm3", round(c.volume / 1000, 2))
print("ball top at rest z", round(center + r, 2), "ceiling z", ceiling, "lift mm", round(ceiling - center - r, 2))
print("seat area mm2", round(area, 2), "ball g", round(ball_g, 1), "open kPa", round(ball_g * 9.81 / area, 2))
