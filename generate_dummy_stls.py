import struct, math, os

OUT_DIR = os.path.join(os.path.dirname(__file__), "meshes")
os.makedirs(OUT_DIR, exist_ok=True)


def write_stl(path, triangles):
    n = len(triangles)
    with open(path, "wb") as f:
        f.write(b"\x00" * 80)
        f.write(struct.pack("<I", n))
        for tri in triangles:
            v1, v2, v3 = tri[0], tri[1], tri[2]
            e1 = (v2[0]-v1[0], v2[1]-v1[1], v2[2]-v1[2])
            e2 = (v3[0]-v1[0], v3[1]-v1[1], v3[2]-v1[2])
            nx = e1[1]*e2[2] - e1[2]*e2[1]
            ny = e1[2]*e2[0] - e1[0]*e2[2]
            nz = e1[0]*e2[1] - e1[1]*e2[0]
            ln = math.hypot(nx, ny, nz)
            if ln > 0:
                nx /= ln; ny /= ln; nz /= ln
            else:
                nx = ny = 0; nz = 1
            f.write(struct.pack("<ffffffffffffH",
                nx, ny, nz,
                v1[0], v1[1], v1[2],
                v2[0], v2[1], v2[2],
                v3[0], v3[1], v3[2], 0))


def cylinder(r, h, seg=24):
    out = []
    dz = h/2
    for i in range(seg):
        a1 = 2*math.pi*i/seg
        a2 = 2*math.pi*(i+1)/seg
        x1, y1 = r*math.cos(a1), r*math.sin(a1)
        x2, y2 = r*math.cos(a2), r*math.sin(a2)
        out.append([(x1,y1,dz),(x2,y2,dz),(x1,y1,-dz)])
        out.append([(x2,y2,dz),(x2,y2,-dz),(x1,y1,-dz)])
    for side, z in [(1,dz), (-1,-dz)]:
        for i in range(seg):
            a1 = 2*math.pi*i/seg
            a2 = 2*math.pi*(i+1)/seg
            x1, y1 = r*math.cos(a1), r*math.sin(a1)
            x2, y2 = r*math.cos(a2), r*math.sin(a2)
            out.append([(0,0,z), (x1*side,y1*side,z), (x2*side,y2*side,z)])
    return out


def box(sx, sy, sz):
    hx, hy, hz = sx/2, sy/2, sz/2
    c = [(-hx,-hy,-hz),(hx,-hy,-hz),(hx,hy,-hz),(-hx,hy,-hz),
         (-hx,-hy,hz),(hx,-hy,hz),(hx,hy,hz),(-hx,hy,hz)]
    faces = [(0,1,3),(1,2,3),(5,4,7),(5,7,6),(0,4,1),(1,4,5),
             (2,6,3),(3,6,7),(0,3,4),(4,3,7),(1,5,2),(2,5,6)]
    return [[c[a],c[b],c[cc]] for a,b,cc in faces]


def sphere(r, seg=16):
    out = []
    for i in range(seg):
        lat1 = math.pi*i/seg
        lat2 = math.pi*(i+1)/seg
        for j in range(seg):
            lon1 = 2*math.pi*j/seg
            lon2 = 2*math.pi*(j+1)/seg
            c00 = (r*math.sin(lat1)*math.cos(lon1), r*math.sin(lat1)*math.sin(lon1), r*math.cos(lat1))
            c01 = (r*math.sin(lat1)*math.cos(lon2), r*math.sin(lat1)*math.sin(lon2), r*math.cos(lat1))
            c10 = (r*math.sin(lat2)*math.cos(lon1), r*math.sin(lat2)*math.sin(lon1), r*math.cos(lat2))
            c11 = (r*math.sin(lat2)*math.cos(lon2), r*math.sin(lat2)*math.sin(lon2), r*math.cos(lat2))
            out.append([c00,c10,c01])
            out.append([c01,c10,c11])
    return out


# Generate actuator dummies (cylinders, METERS)
write_stl(os.path.join(OUT_DIR, "J1_baseyaw_Z.stl"), cylinder(r=0.049, h=0.0385))
write_stl(os.path.join(OUT_DIR, "J2_shoulderpitch_X.stl"), cylinder(r=0.049, h=0.0617))
write_stl(os.path.join(OUT_DIR, "J3_shoulderroll_Z.stl"), cylinder(r=0.049, h=0.0385))
write_stl(os.path.join(OUT_DIR, "J4_elbowpitch_X.stl"), cylinder(r=0.0445, h=0.05025))
write_stl(os.path.join(OUT_DIR, "J5_wristroll_Z.stl"), cylinder(r=0.0395, h=0.043))
write_stl(os.path.join(OUT_DIR, "J6_wristpitch_X.stl"), cylinder(r=0.0395, h=0.043))
write_stl(os.path.join(OUT_DIR, "J7_toolroll_Z.stl"), cylinder(r=0.0395, h=0.043))

# Tool grip (sphere + cross bars)
grip = sphere(r=0.022, seg=12)
grip += box(0.037, 0.005, 0.005)
grip += box(0.005, 0.037, 0.005)
grip += box(0.005, 0.005, 0.037)
write_stl(os.path.join(OUT_DIR, "tool_grip.stl"), grip)

print("All 8 actuator dummies regenerated.")
for name, d_mm, l_mm, zc_mm in [
    ("J1_baseyaw_Z", 98, 38.5, 0),
    ("J2_shoulderpitch_X", 98, 61.7, 180),
    ("J3_shoulderroll_Z", 98, 38.5, 230.1),
    ("J4_elbowpitch_X", 89, 50.25, 395),
    ("J5_wristroll_Z", 79, 43, 567),
    ("J6_wristpitch_X", 79, 43, 610),
    ("J7_toolroll_Z", 79, 43, 653),
]:
    print(f"  {name}: D{d_mm}xL{l_mm}mm @ Z{'' if zc_mm==0 else '+'}{zc_mm}mm")
