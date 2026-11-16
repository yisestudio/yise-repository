"""Stacked-ball dead-weight relief valve with plug-in legs.

1, 2 or 3 steel balls (16 mm) stack in a close-fit tube on a 5.9 mm
sharp seat edge. More balls = more weight = higher opening pressure:
1 ball ~6 kPa, 2 balls ~12 kPa, 3 balls ~18 kPa.
Four removable legs plug into sockets so the valve stands on the floor
with room under it for the hose.

Usage: python3 relief.py <out_dir>   (units: mm, z up)
"""
import math, sys
import manifold3d as m3d
import trimesh

OUT = sys.argv[1] if len(sys.argv) > 1 else "."
SEG = 128

BALL_D = 16.0
MAX_BALLS = 3
TUBE_D = BALL_D + 0.6       # close fit keeps the stack straight
WALL = 2.4
BODY_R = TUBE_D / 2 + WALL
SEAT_D = 5.9                # sharp seat edge -> effective area
BORE_D = 5.3                # hole below the 45-degree chamfer
BARB_BORE_D = 3.0
BARB_TOP = 13.0
SEAT_Z = 18.0
LIFT = 5.0
PLUG_LEN = 3.0
FIT = 0.10

R = BALL_D / 2
c0 = math.sqrt(R * R - (SEAT_D / 2) ** 2)              # bottom ball centre above seat
stack_top = SEAT_Z + c0 + R + (MAX_BALLS - 1) * BALL_D
CEIL_Z = stack_top + LIFT
TOP_Z = CEIL_Z + PLUG_LEN

# legs
LEG_ANGLES = (45, 135, 225, 315)
SLOT_ANGLES = (0, 90, 180, 270)
SOCKET_R = 18.0             # socket centre radius
PIN = 5.0                   # leg pin cross-section
SOCKET = PIN + 0.3          # clearance for SLA
SOCKET_DEPTH = 8.0
BOSS_H = 11.0
BOSS_W = 10.0
LEG_W = 7.0                 # leg body is thicker than the pin -> shoulder stops it
LEG_BELOW = 45.0            # leg length under the body (barb is 13 mm, leaves ~32 mm for the hose bend)
FOOT = (20.0, 12.0, 4.0)    # foot pad, extends outward

def cyl(h, r1, r2=None, z=0.0):
    r2 = r1 if r2 is None else r2
    return m3d.Manifold.cylinder(h, r1, r2, SEG).translate((0, 0, z))

def box(x, y, z, cx=0.0, cy=0.0, z0=0.0):
    return m3d.Manifold.cube((x, y, z)).translate((cx - x / 2, cy - y / 2, z0))

# ---- body ----
base = cyl(BARB_TOP, 2.3)
for z in (2.0, 6.5):
    base = base + cyl(3.5, 2.3, 2.75, z)
base = base + cyl(TOP_Z - BARB_TOP, BODY_R, z=BARB_TOP)
for a in LEG_ANGLES:                                       # leg bosses
    boss = box(SOCKET_R + BOSS_W / 2, BOSS_W, BOSS_H, cx=(SOCKET_R + BOSS_W / 2) / 2, z0=BARB_TOP)
    base = base + boss.rotate((0, 0, a))
base = base - cyl(TOP_Z - SEAT_Z + 1, TUBE_D / 2, z=SEAT_Z)            # ball tube
base = base - cyl(BARB_TOP + 1, BARB_BORE_D / 2, z=-0.5)               # barb bore
base = base - cyl(SEAT_Z - BARB_TOP + 0.2, BORE_D / 2, z=BARB_TOP - 0.1)
cham_h = (SEAT_D - BORE_D) / 2
base = base - cyl(cham_h + 0.001, BORE_D / 2, SEAT_D / 2, SEAT_Z - cham_h)  # sharp 5.9 edge
for a in SLOT_ANGLES:                                      # vent slots at the bottom ball
    slot = box(BODY_R + 2, 4.0, 20.0, cx=(BODY_R + 2) / 2, z0=SEAT_Z + 0.5)
    base = base - slot.rotate((0, 0, a))
for a in LEG_ANGLES:                                       # leg sockets (open downward)
    sock = box(SOCKET, SOCKET, SOCKET_DEPTH + 0.5, cx=SOCKET_R, z0=BARB_TOP - 0.5)
    base = base - sock.rotate((0, 0, a))

# ---- cap ----
plug_r = (TUBE_D - FIT) / 2
cap = cyl(2.5, BODY_R) + cyl(PLUG_LEN, plug_r, z=2.5)
cap = cap - cyl(PLUG_LEN + 1, plug_r - 1.5, z=2.5)
cap = cap - cyl(10, 2.5, z=-1)

# ---- leg (print 3) ----  pin up, foot pad points +x (outward)
leg = box(PIN, PIN, SOCKET_DEPTH, z0=LEG_BELOW)                       # pin into socket
leg = leg + box(LEG_W, LEG_W, LEG_BELOW)                               # leg body
leg = leg + box(FOOT[0], FOOT[1], FOOT[2], cx=FOOT[0] / 2 - LEG_W / 2)  # foot pad

def save(man, name):
    mesh = man.to_mesh()
    tm = trimesh.Trimesh(vertices=mesh.vert_properties[:, :3], faces=mesh.tri_verts)
    tm.export(f"{OUT}/{name}")
    return tm

b = save(base, "relief_base.stl")
c = save(cap, "relief_cap.stl")
l = save(leg, "relief_leg.stl")

area = math.pi * (SEAT_D / 2) ** 2
g1 = 7.85 * math.pi / 6 * (BALL_D / 10) ** 3
print(f"body watertight={b.is_watertight}  OD {2*BODY_R:.1f}  height {TOP_Z:.1f}  tube ID {TUBE_D:.1f}")
print(f"cap watertight={c.is_watertight}  leg watertight={l.is_watertight}")
print(f"standing height ~{TOP_Z + LEG_BELOW + 0 - BARB_TOP + BARB_TOP:.0f} mm above floor (legs {LEG_BELOW} below body bottom)")
print(f"foot reach radius ~{SOCKET_R + FOOT[0] - LEG_W/2:.0f} mm")
print(f"seat area {area:.2f} mm2, one ball {g1:.1f} g")
for n in range(1, MAX_BALLS + 1):
    print(f"  {n} ball(s): {n*g1:.1f} g -> {n*g1*9.81/area:.1f} kPa")
print(f"lift with {MAX_BALLS} balls: {CEIL_Z - stack_top:.1f} mm")
