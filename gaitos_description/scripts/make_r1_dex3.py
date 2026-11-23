#!/usr/bin/env python3
# Copyright 2026 Intelligent Robotics Lab (URJC)
# Licensed under the Apache License, Version 2.0 (see LICENSE)
"""The R1 with Dex3-1 hands, from the unitree_mujoco R1 and the Dex3-1 of the unitree_ros G1 URDF:
the MuJoCo model (mjcf/r1_dex3.xml) and the URDF of the gaitos model (urdf/r1_dex3.urdf).

    make_r1_dex3.py <r1.xml> <r1.urdf> <g1 URDF with hands> <R1 mesh folder> <output folder>

Writes <output>/r1_dex3.xml, <output>/meshes/{left,right}_wrist_roll_dex3_link.STL and
<output>/r1_dex3.urdf: r1.urdf with each hand as one rigid body fixed to the wrist (the mass,
centre of mass and inertia of its 8 links with the fingers at zero), so the controllers of gaitos
know its weight (1.4 kg in all) without the finger joints, which the simulation keeps.
The hand meshes are those of the G1 (installed next to the R1 ones). For each side:
 - the wrist roll link mesh of the R1 ends in the stock fist: it is clipped at x = PALM_X
   (where the wrist housing ends), visual and collision;
 - <side>_hand_palm_link is fixed to <side>_wrist_roll_link at (PALM_X, +-PALM_Y, 0), rpy 0
   (at zero pose the R1 wrist frame has the orientation of the G1 one: forearm along +x);
 - the 7 joints and 7 links of the hand with the inertias, limits and meshes of the URDF.
   The fingers do not collide with each other (excluded pairs), only with the world.
The colours are those of the stock robot (r1_skin.py): the visual meshes are cut per material.
"""
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from r1_skin import apply_skin  # noqa: E402

PALM_X = 0.075      # [m] palm origin along the wrist roll axis (end of the wrist housing)
PALM_Y = 0.003      # [m] the housing is 3 mm off the roll axis, outwards (+y left, -y right)
HAND_RGBA = '0.15 0.15 0.16 1'


def read_stl(path):
    data = open(path, 'rb').read()
    if data[:5] == b'solid' and b'facet' in data[:512]:
        v = [[float(x) for x in line.split()[1:4]] for line in data.decode().splitlines()
             if line.strip().startswith('vertex')]
        return np.array(v, dtype=np.float64).reshape(-1, 3, 3)
    n = int(np.frombuffer(data[80:84], '<u4')[0])
    rec = np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])
    return np.frombuffer(data[84:84 + 50 * n], dtype=rec)['v'].astype(np.float64)


def write_stl(path, tris):
    tris = np.asarray(tris, dtype=np.float32)
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    rec = np.zeros(len(tris), dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]))
    rec['n'], rec['v'] = n, tris
    with open(path, 'wb') as f:
        f.write(b'gaitos: clipped mesh'.ljust(80, b' '))
        f.write(np.uint32(len(tris)).tobytes())
        f.write(rec.tobytes())


def clip_x(tris, x_max):
    """Triangles clipped to x <= x_max (Sutherland-Hodgman, polygons fanned back to triangles)."""
    out = []
    for tri in tris:
        inside = tri[:, 0] <= x_max
        if inside.all():
            out.append(tri)
            continue
        if not inside.any():
            continue
        poly = []
        for i in range(3):
            a, b = tri[i], tri[(i + 1) % 3]
            ia, ib = a[0] <= x_max, b[0] <= x_max
            if ia:
                poly.append(a)
            if ia != ib:
                t = (x_max - a[0]) / (b[0] - a[0])
                poly.append(a + t * (b - a))
        for i in range(1, len(poly) - 1):
            out.append(np.array([poly[0], poly[i], poly[i + 1]]))
    return np.array(out)


def quat_rpy(r, p, y):
    cr, sr, cp, sp = math.cos(r / 2), math.sin(r / 2), math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)


def fmt(values):
    return ' '.join(f'{v:.9g}' for v in values)


def origin(elem):
    o = elem.find('origin') if elem is not None else None
    xyz = [float(v) for v in (o.get('xyz', '0 0 0') if o is not None else '0 0 0').split()]
    rpy = [float(v) for v in (o.get('rpy', '0 0 0') if o is not None else '0 0 0').split()]
    return xyz, rpy


def place(elem, xyz, rpy):
    elem.set('pos', fmt(xyz))
    if any(abs(a) > 1e-12 for a in rpy):
        elem.set('quat', fmt(quat_rpy(*rpy)))


def mesh_name(link):
    f = link.find('visual/geometry/mesh').get('filename')
    return os.path.splitext(os.path.basename(f))[0]


def add_link(parent_body, link, joint, assets):
    """Body of a URDF link (with the joint that moves it, None if fixed) under parent_body."""
    body = ET.SubElement(parent_body, 'body', name=link.get('name'))
    xyz, rpy = origin(joint) if joint is not None else ([0, 0, 0], [0, 0, 0])
    place(body, xyz, rpy)
    inertial = link.find('inertial')
    ixyz, irpy = origin(inertial)
    if any(abs(a) > 1e-12 for a in irpy):
        raise SystemExit(f'{link.get("name")}: rotated inertial frames are not supported')
    i = inertial.find('inertia')
    ET.SubElement(body, 'inertial', pos=fmt(ixyz), mass=inertial.find('mass').get('value'),
                  fullinertia=fmt(float(i.get(k)) for k in ('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz')))
    if joint is not None and joint.get('type') == 'revolute':
        lim = joint.find('limit')
        ET.SubElement(body, 'joint', name=joint.get('name'), pos='0 0 0',
                      axis=joint.find('axis').get('xyz'),
                      range=f'{lim.get("lower")} {lim.get("upper")}',
                      actuatorfrcrange=f'-{lim.get("effort")} {lim.get("effort")}')
    mesh = mesh_name(link)
    assets.add(mesh)
    ET.SubElement(body, 'geom', type='mesh', contype='0', conaffinity='0', group='1', density='0',
                  rgba=HAND_RGBA, mesh=mesh)
    ET.SubElement(body, 'geom', type='mesh', rgba=HAND_RGBA, mesh=mesh)
    return body


def rot_rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y),
                              math.sin(y))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def lumped_hand(links, joints, children, palm):
    """Mass, centre of mass and inertia about it (palm frame) of the hand tree, fingers at zero."""
    parts = []
    stack = [(palm, np.zeros(3), np.eye(3))]
    while stack:
        name, p, R = stack.pop()
        inertial = links[name].find('inertial')
        c, _ = origin(inertial)
        i = inertial.find('inertia')
        g = {k: float(i.get(k)) for k in ('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz')}
        inertia = np.array([[g['ixx'], g['ixy'], g['ixz']], [g['ixy'], g['iyy'], g['iyz']],
                            [g['ixz'], g['iyz'], g['izz']]])
        parts.append((float(inertial.find('mass').get('value')), p + R @ np.array(c),
                      R @ inertia @ R.T))
        for child in children.get(name, []):
            xyz, rpy = origin(joints[child])
            stack.append((child, p + R @ np.array(xyz), R @ rot_rpy(*rpy)))
    mass = sum(m for m, _, _ in parts)
    com = sum(m * c for m, c, _ in parts) / mass
    inertia = sum(I + m * ((c - com) @ (c - com) * np.eye(3) - np.outer(c - com, c - com))
                  for m, c, I in parts)
    return mass, com, inertia


def write_urdf(r1_urdf, out, hands):
    """r1.urdf with the lumped hands fixed to the wrists"""
    tree = ET.parse(r1_urdf)
    robot = tree.getroot()
    for side, (mass, com, inertia), mount in hands:
        link = ET.SubElement(robot, 'link', name=f'{side}_hand_palm_link')
        inertial = ET.SubElement(link, 'inertial')
        ET.SubElement(inertial, 'origin', xyz=fmt(com), rpy='0 0 0')
        ET.SubElement(inertial, 'mass', value=f'{mass:.9g}')
        ET.SubElement(inertial, 'inertia', ixx=f'{inertia[0, 0]:.9g}', ixy=f'{inertia[0, 1]:.9g}',
                      ixz=f'{inertia[0, 2]:.9g}', iyy=f'{inertia[1, 1]:.9g}',
                      iyz=f'{inertia[1, 2]:.9g}', izz=f'{inertia[2, 2]:.9g}')
        joint = ET.SubElement(robot, 'joint', name=f'{side}_hand_palm_joint', type='fixed')
        ET.SubElement(joint, 'origin', xyz=fmt(mount), rpy='0 0 0')
        ET.SubElement(joint, 'parent', link=f'{side}_wrist_roll_link')
        ET.SubElement(joint, 'child', link=f'{side}_hand_palm_link')
    ET.indent(tree, space='  ')
    with open(os.path.join(out, 'r1_dex3.urdf'), 'w') as f:
        f.write('<?xml version="1.0"?>\n<!-- Generated by gaitos_description make_r1_dex3.py: '
                'r1.urdf with the Dex3-1 hands as rigid bodies. Do not edit. -->\n')
        tree.write(f, encoding='unicode')
        f.write('\n')


def main():
    if len(sys.argv) != 6:
        raise SystemExit(__doc__)
    r1_xml, r1_urdf, g1_urdf, r1_meshes, out = sys.argv[1:]
    os.makedirs(os.path.join(out, 'meshes'), exist_ok=True)
    tree = ET.parse(r1_xml)
    root = tree.getroot()
    root.set('model', 'R1 Dex3')
    urdf = ET.parse(g1_urdf).getroot()
    links = {link.get('name'): link for link in urdf.findall('link')}
    joints = {j.find('child').get('link'): j for j in urdf.findall('joint')}
    children = {}
    for j in urdf.findall('joint'):
        children.setdefault(j.find('parent').get('link'), []).append(j.find('child').get('link'))

    asset = root.find('asset')
    world = root.find('worldbody')
    hand_meshes = set()
    excludes = []
    lumped = []
    for side, sign in (('left', 1.0), ('right', -1.0)):
        # Wrist: the stock mesh clipped where the hand starts
        clipped = f'{side}_wrist_roll_dex3_link'
        tris = clip_x(read_stl(os.path.join(r1_meshes, f'{side}_wrist_roll_link.STL')), PALM_X)
        write_stl(os.path.join(out, 'meshes', clipped + '.STL'), tris)
        ET.SubElement(asset, 'mesh', name=clipped, file=clipped + '.STL')
        wrist = next(b for b in world.iter('body') if b.get('name') == f'{side}_wrist_roll_link')
        for g in wrist.findall('geom'):
            if g.get('mesh') == f'{side}_wrist_roll_link':
                g.set('mesh', clipped)
        # Hand tree from the URDF
        palm_name = f'{side}_hand_palm_link'
        palm = add_link(wrist, links[palm_name], None, hand_meshes)
        palm.set('pos', fmt([PALM_X, sign * PALM_Y, 0.0]))
        bodies = [palm_name]
        stack = [(palm, palm_name)]
        while stack:
            body, name = stack.pop()
            for child in children.get(name, []):
                stack.append((add_link(body, links[child], joints[child], hand_meshes), child))
                bodies.append(child)
        if len(bodies) != 8:
            raise SystemExit(f'{side} hand: expected 8 links, found {bodies}')
        lumped.append((side, lumped_hand(links, joints, children, palm_name),
                       [PALM_X, sign * PALM_Y, 0.0]))
        excludes += [(a, b) for k, a in enumerate(bodies) for b in bodies[k + 1:]]
    for mesh in sorted(hand_meshes):
        ET.SubElement(asset, 'mesh', name=mesh, file=mesh + '.STL')
    contact = root.find('contact')
    if contact is None:
        contact = ET.Element('contact')
        root.insert(list(root).index(world) + 1, contact)
    for a, b in excludes:
        ET.SubElement(contact, 'exclude', body1=a, body2=b)

    files = {m.get('name'): m.get('file') for m in asset.findall('mesh')}

    def mesh_file(name):
        for folder in (os.path.join(out, 'meshes'), r1_meshes):
            if os.path.exists(os.path.join(folder, files[name])):
                return os.path.join(folder, files[name])
        raise SystemExit(f'mesh {name}: {files[name]} not found')
    apply_skin(root, mesh_file, read_stl, write_stl, os.path.join(out, 'meshes'))

    write_urdf(r1_urdf, out, lumped)
    ET.indent(tree, space='  ')
    with open(os.path.join(out, 'r1_dex3.xml'), 'w') as f:
        f.write('<!-- Generated by gaitos_description make_r1_dex3.py from the unitree_mujoco R1 and\n'
                '     the Dex3-1 of the unitree_ros G1 URDF (BSD-3-Clause, Unitree Robotics).\n'
                '     Do not edit. -->\n')
        tree.write(f, encoding='unicode')
        f.write('\n')


if __name__ == '__main__':
    main()
