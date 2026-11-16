"""Dead-weight ball relief valve with swappable 12 mm ball cartridges (FDM).

Body (fixed): base plug + side hose barb + 16.2 mm tube + tapered socket.
Cartridge (swap): sleeve for 12 mm balls with its own seat hole and a
15-degree funnel. A tapered spigot under the cartridge wedges into the
body socket, so it seals without a washer. A rib on the cartridge rides
in a groove in the tube so its vent slots line up with the body's.

  cartridge 8.0 mm hole : ~1.4 kPa per 12 mm steel ball (big modules)
  cartridge 5.5 mm hole : ~2.9 kPa per ball           (small modules)

Usage: python3 relief.py <out_dir>   (units: mm, z up)
"""
import math, os, sys
import manifold3d as m3d
import trimesh

OUT = sys.argv[1] if len(sys.argv) > 1 else "."
SEG = 128

# ---------- body ----------
TUBE_D = 16.2
WALL = 2.4
BODY_R = TUBE_D / 2 + WALL
FLOOR_Z = 14.0              # top of body floor
TOP_Z = 96.5
PLUG_LEN = 3.0
CAP_FIT = 0.25
RING_R, RING_H = 14.0, 14.0
BARB_Z, BARB_LEN, CH_D = 4.5, 19.0, 3.0
RIDGE_OFFS = (10.0, 14.5)
ROOT_R, ROOT_LEN = 6.0, 8.0
SOCK_TOP_D, SOCK_BOT_D, SOCK_DEPTH = 12.5, 11.6, 9.0   # tapered socket in the floor
GROOVE_W, GROOVE_DEPTH, GROOVE_ANG = 3.0, 1.0, 45.0
SLOT_ANGLES = (0, 90, 180, 270)
SLOT_W, SLOT_H = 4.0, 28.0
SLOT_Z0 = FLOOR_Z + 1.0

# ---------- cartridge ----------
BALL_D = 12.0
CART_OD = TUBE_D - 0.5
CART_ID = BALL_D + 1.0
CART_FLOOR = 3.0
CART_GAP = 0.8              # underside hovers above the body floor; the taper does the sealing
SPIG_TOP_D, SPIG_BOT_D, SPIG_LEN = 12.6, 11.8, 8.0
FUNNEL_DEG = 15.0
RIB_W, RIB_H = GROOVE_W - 0.5, GROOVE_DEPTH - 0.2
CART_SLOT_W = 3.5
CART_TOP_Z = TOP_Z - PLUG_LEN - 1.0     # stays below the cap plug
SEATS = (8.0, 5.5)

# ---------- X base ----------
ARM_ANGLES = (45, 135, 225, 315)
HUB_FIT, HUB_WALL, HUB_H = 0.25, 4.0, 14.0
ARM_LEN, ARM_W, ARM_H = 90.0, 18.0, 6.0
BARB_SLOT_W = 14.0

def cyl(h, r1, r2=None, z=0.0):
    r2 = r1 if r2 is None else r2
    return m3d.Manifold.cylinder(h, r1, r2, SEG).translate((0, 0, z))

def box(x, y, z, x0=0.0, cy=0.0, z0=0.0):
    return m3d.Manifold.cube((x, y, z)).translate((x0, cy - y / 2, z0))

def xcyl(length, r1, r2, x0, z):
    return m3d.Manifold.cylinder(length, r1, r2, SEG).rotate((0, 90, 0)).translate((x0, 0, z))

def to_tm(man):
    mesh = man.to_mesh()
    return trimesh.Trimesh(vertices=mesh.vert_properties[:, :3], faces=mesh.tri_verts)

# ---- body ----
body = cyl(RING_H, RING_R) + cyl(TOP_Z, BODY_R)
bx = RING_R - 1.0
barb = xcyl(BARB_LEN, 2.3, 2.3, bx, BARB_Z)
for off in RIDGE_OFFS:
    barb = barb + xcyl(3.5, 2.75, 2.3, bx + off, BARB_Z)
barb = barb + xcyl(ROOT_LEN, ROOT_R, 2.3, bx - 1.0, BARB_Z)       # root collar
barb = barb ^ box(100, 100, 50, x0=-10, z0=0.0)                  # nothing below the bed
body = body + barb
body = body - xcyl(bx + BARB_LEN + 1, CH_D / 2, CH_D / 2, 0.0, BARB_Z)          # side channel
riser_z0 = BARB_Z - CH_D / 2
body = body - cyl(FLOOR_Z - SOCK_DEPTH - riser_z0 + 0.2, 2.0, z=riser_z0)      # riser to socket
body = body - cyl(SOCK_DEPTH + 0.01, SOCK_BOT_D / 2, SOCK_TOP_D / 2, FLOOR_Z - SOCK_DEPTH)  # tapered socket
body = body - cyl(TOP_Z - FLOOR_Z + 1, TUBE_D / 2, z=FLOOR_Z)                  # tube
groove = box(TUBE_D / 2 + GROOVE_DEPTH, GROOVE_W, TOP_Z - FLOOR_Z + 1, x0=0, z0=FLOOR_Z)
body = body - groove.rotate((0, 0, GROOVE_ANG))
for a in SLOT_ANGLES:
    body = body - box(BODY_R + 2, SLOT_W, SLOT_H, x0=0, z0=SLOT_Z0).rotate((0, 0, a))

# ---- cap ----
plug_r = (TUBE_D - CAP_FIT) / 2
cap = cyl(2.5, BODY_R) + cyl(PLUG_LEN, plug_r, z=2.5)
cap = cap - cyl(PLUG_LEN + 1, plug_r - 1.6, z=2.5)
cap = cap - cyl(10, 3.0, z=-1)

# ---- cartridge (modelled in place) ----
def cartridge(seat_d):
    z0 = FLOOR_Z + CART_GAP
    seat_z = z0 + CART_FLOOR
    funnel_h = (CART_ID / 2 - seat_d / 2) * math.tan(math.radians(FUNNEL_DEG))
    c = cyl(CART_TOP_Z - z0, CART_OD / 2, z=z0)
    c = c + cyl(SPIG_LEN, SPIG_BOT_D / 2, SPIG_TOP_D / 2, z0 - SPIG_LEN)
    rib = box(CART_OD / 2 + RIB_H, RIB_W, CART_TOP_Z - z0, x0=0, z0=z0)
    c = c + rib.rotate((0, 0, GROOVE_ANG))
    c = c - cyl(funnel_h + 0.001, seat_d / 2, CART_ID / 2, seat_z)
    c = c - cyl(CART_TOP_Z - seat_z, CART_ID / 2, z=seat_z + funnel_h)
    c = c - cyl(SPIG_LEN + CART_FLOOR + 0.2, seat_d / 2, z=z0 - SPIG_LEN - 0.1)
    for a in SLOT_ANGLES:
        c = c - box(CART_OD / 2 + 2, CART_SLOT_W, SLOT_H - 2, x0=0, z0=seat_z + 0.5).rotate((0, 0, a))
    return c, seat_z

# ---- X base (no end pads) ----
hub_in = RING_R + HUB_FIT
xb = cyl(HUB_H, hub_in + HUB_WALL)
for a in ARM_ANGLES:
    xb = xb + box(ARM_LEN, ARM_W, ARM_H, x0=0.0).rotate((0, 0, a))
xb = xb - cyl(HUB_H + 2, hub_in, z=-1)
xb = xb - box(hub_in + HUB_WALL + 2, BARB_SLOT_W, HUB_H + 2, x0=0.0, z0=-1)

parts = {"relief_base.stl": to_tm(body), "relief_cap.stl": to_tm(cap), "relief_xbase.stl": to_tm(xb)}
g1 = 7.85 * math.pi / 6 * (BALL_D / 10) ** 3
info = []
for sd in SEATS:
    cm, seat_z = cartridge(sd)
    tm = to_tm(cm)
    tm.apply_translation((0, 0, -tm.bounds[0][2]))        # print upright, spigot on the bed
    name = f"relief_cart_{str(sd).replace('.', 'p')}mm.stl"
    parts[name] = tm
    area = math.pi * (sd / 2) ** 2
    c0 = seat_z + math.sqrt((BALL_D / 2) ** 2 - (sd / 2) ** 2)
    nmax = int((CART_TOP_Z - 3.0 - (c0 + BALL_D / 2)) // BALL_D) + 1
    seated = BALL_D / 2 * math.sin(math.radians(FUNNEL_DEG)) < sd / 2
    info.append((name, tm, seated, nmax, area))

for name, tm in parts.items():
    tm.export(f"{OUT}/{name}")
    print(f"{name}: watertight={tm.is_watertight} size={[round(v,1) for v in (tm.bounds[1]-tm.bounds[0])]}")
for name, tm, seated, nmax, area in info:
    print(f"{name}: edge_seated={seated} max balls {nmax}: " + "  ".join(f"{n}:{n*g1*9.81/area:.1f}" for n in range(1, nmax + 1)) + " kPa")
print(f"ball {g1:.2f} g, cartridge wall {(CART_OD-CART_ID)/2:.2f} mm")

# ---- one print plate ----
plate = [parts["relief_xbase.stl"]]
for name, pos in (("relief_base.stl", (52, 0)), ("relief_cap.stl", (-52, 0)),
                  ("relief_cart_8p0mm.stl", (0, 52)), ("relief_cart_5p5mm.stl", (0, -52))):
    t = parts[name].copy(); t.apply_translation((pos[0], pos[1], 0)); plate.append(t)
P = trimesh.util.concatenate(plate)
P.export(f"{OUT}/relief_plate.stl")
print("plate size", [round(v, 1) for v in (P.bounds[1] - P.bounds[0])])
