# Copyright 2026 Intelligent Robotics Lab (URJC)
# Licensed under the Apache License, Version 2.0 (see LICENSE)
"""Colours of the R1 (MuJoCo), after the photos of the stock robot (unitree.com/R1): glossy white
shells, slate-blue joints and side panels, black visor and knee pads, red accents, dark metal.

The meshes of unitree_mujoco are one STL per link with one colour. apply_skin() cuts each visual
mesh into one mesh per material: the CAD parts (connected components, ranked by size, which match
between the left and right meshes) and, inside a part, regions given by the position and normal of
each triangle in the mesh frame (left frame; right meshes are mirrored in y before the rules).
Only the visual geoms (group 1) change; the collision geoms keep their meshes and go to group 3,
hidden by default, so they no longer show over the visual ones. Physics is unchanged.
"""
import os
import xml.etree.ElementTree as ET

import numpy as np

# name: rgba, specular, shininess, reflectance
MATERIALS = {
    'shell': ('0.92 0.92 0.93 1', 0.6, 0.6, 0.0),       # satin white plastic
    'slate': ('0.36 0.43 0.60 1', 0.9, 0.8, 0.0),       # metallic blue-steel
    'dark': ('0.11 0.115 0.125 1', 0.35, 0.45, 0.0),    # dark grey: motors, joints, soles
    'black': ('0.02 0.02 0.025 1', 1.0, 1.0, 0.15),     # glossy black: visor, knee pads
    'red': ('0.80 0.07 0.09 1', 0.6, 0.7, 0.0),
    'metal': ('0.55 0.56 0.58 1', 1.0, 0.9, 0.0),       # rods, handle
    'hand': ('0.24 0.24 0.26 1', 0.45, 0.55, 0.0),        # Dex3-1: graphite
}


def components(tris):
    """Connected component of each triangle (shared vertices), ranked by size (0 = largest)."""
    v = np.round(tris.reshape(-1, 3) / 1e-6).astype(np.int64)
    _, idx = np.unique(v, axis=0, return_inverse=True)
    faces = idx.reshape(-1, 3)
    label = np.arange(idx.max() + 1)
    while True:   # min-label propagation with pointer jumping
        m = label[faces].min(axis=1)
        new = label.copy()
        np.minimum.at(new, faces.ravel(), np.repeat(m, 3))
        new = new[new]
        if np.array_equal(new, label):
            break
        label = new
    face_label = label[faces[:, 0]]
    uniq, inv, count = np.unique(face_label, return_inverse=True, return_counts=True)
    rank = np.empty(len(uniq), dtype=np.int64)
    rank[np.lexsort((uniq, -count))] = np.arange(len(uniq))
    return rank[inv]


class Faces:
    """Sample points of triangles in the left frame: k (component rank), x, y, z, nx, ny, nz."""

    def __init__(self, points, normals, k):
        self.k = k
        self.x, self.y, self.z = points.T
        self.nx, self.ny, self.nz = normals.T
        self.n = len(k)


def classify(tris, mirror, rule, min_edge=0.0015, rounds=6):
    """Material of each triangle, splitting (4 ways) the triangles whose corners and centroid
    fall in different materials until their edges are min_edge long: clean region borders."""
    t = tris.copy()
    if mirror:
        t[:, :, 1] *= -1
    k = components(tris)
    for r in range(rounds + 1):
        n = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
        n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
        if mirror:
            n = -n   # mirroring flips the winding
        pts = np.concatenate([t.mean(axis=1), t[:, 0], t[:, 1], t[:, 2]])
        m = rule(Faces(pts, np.tile(n, (4, 1)), np.tile(k, 4))).reshape(4, -1)
        mixed = (m != m[0]).any(axis=0)
        edge = np.linalg.norm(t - np.roll(t, 1, axis=1), axis=2).max(axis=1)
        split = mixed & (edge > min_edge)
        if r == rounds or not split.any():
            break
        a, b, c = t[split, 0], t[split, 1], t[split, 2]
        ab, bc, ca = (a + b) / 2, (b + c) / 2, (c + a) / 2
        sub = np.concatenate([np.stack(q, axis=1) for q in
                              ((a, ab, ca), (ab, b, bc), (ca, bc, c), (ab, bc, ca))])
        t = np.concatenate([t[~split], sub])
        k = np.concatenate([k[~split], np.tile(k[split], 4)])
    if mirror:
        t[:, :, 1] *= -1
    return t, m[0]


def v_mark(f, cx, half_width, depth, bar, closed=False):
    """A V seen from the front (+x): apex at z = cx[1], arms up to depth above it; closed: a
    triangle outline (a bar across the top)."""
    u = f.z - cx[1]
    w = half_width * u / depth
    ay = np.abs(f.y - cx[0])
    mark = (u >= 0) & (u <= depth) & (np.abs(ay - w) < bar)
    if closed:
        mark |= (u <= depth) & (u >= depth - 2 * bar) & (ay <= half_width + bar)
    return mark


# Rules per mesh (left names; 'right_' meshes use the 'left_' rules mirrored). Each returns the
# material of every triangle; meshes not listed are one material (DEFAULT_MATERIAL or 'shell').
def head(f):
    m = np.full(f.n, 'dark', dtype=object)
    m[f.k == 0] = 'shell'
    m[f.k == 1] = 'shell'
    m[(f.k == 1) & (np.abs(f.y) > 0.05) & (np.abs(f.ny) > 0.6)] = 'slate'   # ear discs
    m[f.k == 2] = 'black'                                                   # visor
    forehead = (f.k == 0) & (f.x > 0.02) & (f.nx > 0.2) & (f.nz > -0.2)
    m[forehead & v_mark(f, (0.0, 0.005), 0.026, 0.045, 0.0035)] = 'red'
    return m


def torso(f):
    m = np.full(f.n, 'dark', dtype=object)
    shell = f.k == 0
    m[shell] = 'shell'
    ay = np.abs(f.y)
    # white chest plate shaped as a shield: slate back, sides widening downwards, belly
    front = f.x > -0.035
    m[shell & front & ((ay > 0.088) | (ay > 0.10 - 0.3 * (0.25 - f.z))
                       | (f.z < 0.10 - 0.25 * ay))] = 'slate'
    m[shell & ~front & (ay < 0.03)] = 'slate'        # back: white, blue-steel spine
    m[shell & (f.nx > 0.5) & v_mark(f, (0.0, 0.205), 0.022, 0.03, 0.003)] = 'red'   # chest mark
    m[f.k == 1] = 'slate'
    m[(f.k == 7) | (f.k == 8)] = 'metal'       # handle on the back
    m[(f.k == 3) | (f.k == 4) | (f.k == 10)] = 'slate'   # neck and shoulder mounts
    return m


def pelvis(f):
    m = np.full(f.n, 'dark', dtype=object)
    m[(f.k == 0) | (f.k == 1)] = 'shell'
    m[f.k == 2] = 'shell'
    m[(f.k == 5) | (f.k == 6)] = 'slate'
    return m


def waist_roll(f):
    m = np.full(f.n, 'slate', dtype=object)
    m[f.k == 2] = 'dark'
    return m


def hip_pitch(f):
    m = np.full(f.n, 'slate', dtype=object)
    m[f.k == 3] = 'dark'
    return m


def hip_roll(f):
    m = np.full(f.n, 'shell', dtype=object)
    m[f.k == 1] = 'shell'          # motor cover: white like the thigh
    return m


def thigh(f):
    m = np.full(f.n, 'dark', dtype=object)
    m[f.k == 2] = 'shell'          # back
    m[f.k == 3] = 'shell'          # front
    stripes = (f.k == 3) & (f.nx > 0.3) & (f.y > 0.0) & (f.z > -0.14) & (f.z < -0.07)
    m[stripes & (np.mod(f.z + 0.14, 0.014) < 0.006)] = 'red'
    m[f.k == 0] = 'black'          # knee housing
    return m


def shin(f):
    m = np.full(f.n, 'dark', dtype=object)
    m[f.k == 0] = 'slate'          # calf
    m[f.k == 1] = 'slate'          # side
    m[f.k == 2] = 'shell'          # front
    m[f.k == 3] = 'black'          # knee pad
    return m


def foot(f):
    m = np.full(f.n, 'shell', dtype=object)
    m[f.k == 0] = 'dark'           # sole
    return m


def shoulder_pitch(f):
    m = np.full(f.n, 'shell', dtype=object)
    m[f.k == 1] = 'slate'
    m[(f.k == 0) & (f.ny > 0.6) & (f.x ** 2 + (f.z + 0.015) ** 2 < 0.026 ** 2)] = 'slate'  # cap disc
    return m


def shoulder_roll(f):
    m = np.full(f.n, 'shell', dtype=object)
    m[f.k >= 1] = 'dark'
    return m


def shoulder_yaw(f):
    m = np.full(f.n, 'shell', dtype=object)
    m[(f.k == 0) & (np.abs(f.ny) > 0.85) & (f.z < -0.02)] = 'slate'   # elbow motor caps
    m[f.k >= 1] = 'dark'
    return m


def elbow(f):
    m = np.full(f.n, 'dark', dtype=object)
    m[f.k == 0] = 'slate'
    m[f.k == 1] = 'shell'
    return m


def wrist(f):
    m = np.full(f.n, 'shell', dtype=object)
    m[f.k >= 3] = 'dark'
    return m


RULES = {
    'head_yaw_link': head, 'waist_yaw_link': torso, 'pelvis_link': pelvis,
    'waist_roll_link': waist_roll, 'left_hip_pitch_link': hip_pitch,
    'left_hip_roll_link': hip_roll, 'left_hip_yaw_link': thigh, 'left_knee_link': shin,
    'left_ankle_roll_link': foot, 'left_shoulder_pitch_link': shoulder_pitch,
    'left_shoulder_roll_link': shoulder_roll, 'left_shoulder_yaw_link': shoulder_yaw,
    'left_elbow_link': elbow, 'left_wrist_roll_dex3_link': wrist,
}
SINGLE = {   # meshes of one material
    'head_pitch_link': 'slate', 'imu_in_pelvis_link': 'dark', 'left_ankle_pitch_link': 'dark',
    'left_ankle_A_link': 'dark', 'left_ankle_B_link': 'dark', 'left_ankle_A_rod_link': 'metal',
    'left_ankle_B_rod_link': 'metal', 'left_ankle_constraint_A_link': 'metal',
    'left_ankle_constraint_B_link': 'metal',
}


def rule_of(mesh):
    """(rule or material, mirrored) for a mesh name"""
    mirror = mesh.startswith('right_')
    key = 'left_' + mesh[len('right_'):] if mirror else mesh
    if key in RULES:
        return RULES[key], mirror
    if '_hand_' in key:
        return 'hand', mirror
    return SINGLE.get(key, 'shell'), mirror


def apply_skin(root, mesh_file, read_stl, write_stl, out_meshes):
    """Materials and per-material visual meshes for the robot of the MJCF root (in place).

    mesh_file(name) -> path of the STL of an asset mesh; new meshes are written to out_meshes."""
    asset = root.find('asset')
    for name, (rgba, spec, shin, refl) in MATERIALS.items():
        ET.SubElement(asset, 'material', name=f'r1_{name}', rgba=rgba, specular=f'{spec:g}',
                      shininess=f'{shin:g}', reflectance=f'{refl:g}')
    split = {}   # mesh -> [(material, sub mesh name)]
    for body in root.find('worldbody').iter('body'):
        for index, geom in list(enumerate(body.findall('geom'))):
            if geom.get('group') != '1':
                if geom.get('contype') != '0' or geom.get('conaffinity') != '0':
                    geom.set('group', '3')            # collision: hidden by default
                    geom.attrib.pop('rgba', None)
                continue
            mesh = geom.get('mesh')
            if mesh is None:
                continue
            rule, mirror = rule_of(mesh)
            if isinstance(rule, str):
                geom.attrib.pop('rgba', None)
                geom.set('material', f'r1_{rule}')
                continue
            if mesh not in split:
                tris, mats = classify(read_stl(mesh_file(mesh)), mirror, rule)
                split[mesh] = []
                for mat in sorted(set(mats)):
                    sub = f'{mesh}_{mat}'
                    sel = tris[mats == mat]
                    extent = sel.reshape(-1, 3).max(axis=0) - sel.reshape(-1, 3).min(axis=0)
                    if len(sel) < 4 or extent.min() < 1e-4:   # flat: MuJoCo cannot hull it
                        continue
                    write_stl(os.path.join(out_meshes, sub + '.STL'), sel)
                    ET.SubElement(asset, 'mesh', name=sub, file=sub + '.STL')
                    split[mesh].append((mat, sub))
            # one visual geom per material, same frame as the original
            position = list(body).index(geom)
            body.remove(geom)
            for i, (mat, sub) in enumerate(split[mesh]):
                g = ET.Element('geom', {k: v for k, v in geom.attrib.items() if k not in ('rgba', 'mesh')})
                g.set('mesh', sub)
                g.set('material', f'r1_{mat}')
                body.insert(position + i, g)
