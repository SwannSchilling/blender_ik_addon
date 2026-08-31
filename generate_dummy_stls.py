"""Generate dummy STL meshes for Design B arm visual segments."""
import struct, math, os

OUT_DIR = os.path.join(os.path.dirname(__file__), "meshes")
os.makedirs(OUT_DIR, exist_ok=True)


def write_stl(path: str, triangles: list) -> None:
    """Write triangles as binary STL."""
    n = len(triangles)
    with open(path, "wb") as f:
        f.write(b"\x00" * 80)
        f.write(struct.pack("<I", n))
        for tri in triangles:
            v1 = (tri[3], tri[4], tri[5])
            v2 = (tri[6], tri[7], tri[8])
            v3 = (tri[9], tri[10], tri[11])
            e1 = (v2[0]-v1[0], v2[1]-v1[1], v2[2]-v1[2])
            e2 = (v3[0]-v1[0], v3[1]-v1[1], v3[2]-v1[2])
            nx = e1[1]*e2[2] - e1[2]*e2[1]
            ny = e1[2]*e2[0] - e1[0]*e2[2]
            nz = e1[0]*e2[1] - e1[1]*e2[0]
            ln = math.sqrt(nx*nx + ny*ny + nz*nz)
            if ln > 0:
                nx /= ln; ny /= ln; nz /= ln
            else:
                nx = ny = 0.0; nz = 1.0
            f.write(struct.pack("<ffffffffffffH",
                nx, ny, nz,
                tri[3], tri[4], tri[5],
                tri[6], tri[7], tri[8],
                tri[9], tri[10], tri[11],
                tri[0] & 0xFFFF))
    print(f"  wrote {n} tris -> {path}")


def box(sx: float, sy: float, sz: float) -> list:
    hx, hy, hz = sx/2, sy/2, sz/2
    c = [
        (-hx, -hy, -hz), ( hx, -hy, -hz), ( hx,  hy, -hz), (-hx,  hy, -hz),
        (-hx, -hy,  hz), ( hx, -hy,  hz), ( hx,  hy,  hz), (-hx,  hy,  hz),
    ]
    faces = [
        (0,1,3), (1,2,3),  # -z
        (5,4,7), (5,7,6),  # +z
        (0,4,1), (1,4,5),  # -y
        (2,6,3), (3,6,7),  # +y
        (0,3,4), (4,3,7),  # -x
        (1,5,2), (2,5,6),  # +x
    ]
    out = []
    for a, b, c2 in faces:
        out.append([0, 0, 0,
            c[a][0], c[a][1], c[a][2],
            c[b][0], c[b][1], c[b][2],
            c[c2][0], c[c2][1], c[c2][2],
            0])
    return out


def cylinder(r: float, h: float, seg: int = 24) -> list:
    out = []
    dz = h/2
    for i in range(seg):
        a1 = 2*math.pi * i / seg
        a2 = 2*math.pi * (i+1) / seg
        x1, y1 = r*math.cos(a1), r*math.sin(a1)
        x2, y2 = r*math.cos(a2), r*math.sin(a2)
        out.append([0, 0, 0,
            x1, y1, dz,
            x2, y2, dz,
            x1, y1, -dz,
            0])
        out.append([0, 0, 0,
            x2, y2, dz,
            x2, y2, -dz,
            x1, y1, -dz,
            0])
    for side, z in [(1, dz), (-1, -dz)]:
        for i in range(seg):
            a1 = 2*math.pi * i / seg
            a2 = 2*math.pi * (i+1) / seg
            x1, y1 = r*math.cos(a1), r*math.sin(a1)
            x2, y2 = r*math.cos(a2), r*math.sin(a2)
            out.append([0, 0, 0,
                0.0, 0.0, z,
                x1*side, y1*side, z,
                x2*side, y2*side, z,
                0])
    return out


def sphere(r: float, seg: int = 16) -> list:
    out = []
    for i in range(seg):
        lat1 = math.pi * i / seg
        lat2 = math.pi * (i+1) / seg
        for j in range(seg):
            lon1 = 2*math.pi * j / seg
            lon2 = 2*math.pi * (j+1) / seg
            def p(lat, lon):
                return (r*math.sin(lat)*math.cos(lon),
                        r*math.sin(lat)*math.sin(lon),
                        r*math.cos(lat))
            c00 = p(lat1, lon1)
            c01 = p(lat1, lon2)
            c10 = p(lat2, lon1)
            c11 = p(lat2, lon2)
            out.append([0, 0, 0,
                c00[0], c00[1], c00[2],
                c10[0], c10[1], c10[2],
                c01[0], c01[1], c01[2],
                0])
            out.append([0, 0, 0,
                c01[0], c01[1], c01[2],
                c10[0], c10[1], c10[2],
                c11[0], c11[1], c11[2],
                0])
    return out


# Generate all Design B visual segments
write_stl(os.path.join(OUT_DIR, "base.stl"), cylinder(r=0.10, h=0.18))
write_stl(os.path.join(OUT_DIR, "column.stl"), box(0.08, 0.08, 0.18))
write_stl(os.path.join(OUT_DIR, "upper_arm.stl"), box(0.06, 0.215, 0.06))
write_stl(os.path.join(OUT_DIR, "upper_arm_pivot.stl"), sphere(r=0.055, seg=16))
write_stl(os.path.join(OUT_DIR, "forearm.stl"), box(0.06, 0.215, 0.06))
write_stl(os.path.join(OUT_DIR, "forearm_pivot.stl"), sphere(r=0.048, seg=16))
write_stl(os.path.join(OUT_DIR, "wrist.stl"), box(0.06, 0.065, 0.06))
write_stl(os.path.join(OUT_DIR, "wrist_pivot.stl"), sphere(r=0.038, seg=16))

# tool grip: sphere + cross bars combined
grip = sphere(r=0.022, seg=12)
grip += box(0.037, 0.005, 0.005)
grip += box(0.005, 0.037, 0.005)
grip += box(0.005, 0.005, 0.037)
write_stl(os.path.join(OUT_DIR, "tool_grip.stl"), grip)

print("All dummy STLs written to", OUT_DIR)