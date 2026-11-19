#!/usr/bin/env python3
# Copyright 2026 Juan S. Cely G.
# Licensed under the Apache License, Version 2.0 (see LICENSE)
"""Convert COLLADA meshes (.dae) to Wavefront OBJ, the format MuJoCo loads.

    dae_to_obj.py <dae folder> <obj folder>

Every <name>.dae of the folder becomes <name>.obj: positions with the transform of the node
that instances the geometry (the units of the .dae are kept, so the mesh scale of the URDF
still applies), normals and texture coordinates. Only triangles (<triangles>, or <polylist>
of triangles); the materials are left out (one texture per robot, set in the MJCF).
"""
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

NS = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}


def floats(src):
    arr = src.find('c:float_array', NS)
    stride = int(src.find('c:technique_common/c:accessor', NS).get('stride', '1'))
    return np.array(arr.text.split(), dtype=float).reshape(-1, stride)


def node_transforms(root):
    """Geometry id -> 4x4 transform of the node (and its parents) that instances it."""
    out = {}

    def walk(node, parent):
        t = parent
        for e in node:
            tag = e.tag.split('}')[1]
            if tag == 'matrix':
                t = t @ np.array(e.text.split(), dtype=float).reshape(4, 4)
            elif tag == 'translate':
                m = np.eye(4)
                m[:3, 3] = [float(x) for x in e.text.split()]
                t = t @ m
            elif tag == 'scale':
                t = t @ np.diag([float(x) for x in e.text.split()] + [1.0])
            elif tag in ('rotate',):
                x, y, z, a = (float(v) for v in e.text.split())
                axis = np.array([x, y, z]) / np.linalg.norm([x, y, z])
                a = np.radians(a)
                k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]],
                              [-axis[1], axis[0], 0]])
                m = np.eye(4)
                m[:3, :3] = np.eye(3) + np.sin(a) * k + (1 - np.cos(a)) * k @ k
                t = t @ m
        for g in node.findall('c:instance_geometry', NS):
            out[g.get('url').lstrip('#')] = t
        for child in node.findall('c:node', NS):
            walk(child, t)

    for scene in root.findall('.//c:visual_scene', NS):
        for node in scene.findall('c:node', NS):
            walk(node, np.eye(4))
    return out


def convert(dae, obj):
    root = ET.parse(dae).getroot()
    up = root.find('c:asset/c:up_axis', NS)
    if up is not None and up.text.strip() != 'Z_UP':
        sys.exit(f'{dae}: up axis {up.text.strip()} (only Z_UP)')
    unit = root.find('c:asset/c:unit', NS)
    meter = float(unit.get('meter', '1')) if unit is not None else 1.0
    transforms = node_transforms(root)
    v_all, vt_all, vn_all, faces = [], [], [], []
    for geom in root.findall('.//c:library_geometries/c:geometry', NS):
        mesh = geom.find('c:mesh', NS)
        if mesh is None:
            continue
        t = transforms.get(geom.get('id'), np.eye(4))
        t = np.diag([meter, meter, meter, 1.0]) @ t
        sources = {s.get('id'): s for s in mesh.findall('c:source', NS)}
        vertices = mesh.find('c:vertices', NS)
        vert_src = {i.get('semantic'): i.get('source').lstrip('#')
                    for i in vertices.findall('c:input', NS)}
        for prim in list(mesh.findall('c:triangles', NS)) + list(mesh.findall('c:polylist', NS)):
            vcount = prim.find('c:vcount', NS)
            if vcount is not None and set(vcount.text.split()) != {'3'}:
                sys.exit(f'{dae}: polygons that are not triangles')
            inputs = prim.findall('c:input', NS)
            stride = max(int(i.get('offset')) for i in inputs) + 1
            idx = np.array(prim.find('c:p', NS).text.split(), dtype=int).reshape(-1, stride)
            base = {'v': len(v_all), 'vt': len(vt_all), 'vn': len(vn_all)}
            cols = {}
            for i in inputs:
                sem, off = i.get('semantic'), int(i.get('offset'))
                src = i.get('source').lstrip('#')
                if sem == 'VERTEX':
                    p = floats(sources[vert_src['POSITION']])[:, :3]
                    p = (np.c_[p, np.ones(len(p))] @ t.T)[:, :3]
                    v_all.extend(p)
                    cols['v'] = idx[:, off] + base['v']
                    if 'NORMAL' in vert_src:   # normals given per vertex
                        n = floats(sources[vert_src['NORMAL']])[:, :3]
                        vn_all.extend(n @ np.linalg.inv(t[:3, :3]))
                        cols['vn'] = idx[:, off] + base['vn']
                elif sem == 'NORMAL':
                    n = floats(sources[src])[:, :3]
                    vn_all.extend(n @ np.linalg.inv(t[:3, :3]))
                    cols['vn'] = idx[:, off] + base['vn']
                elif sem == 'TEXCOORD' and 'vt' not in cols:
                    vt_all.extend(floats(sources[src])[:, :2])
                    cols['vt'] = idx[:, off] + base['vt']
            faces.append(cols)

    with open(obj, 'w') as f:
        f.write(f'# {os.path.basename(obj)}: converted from {os.path.basename(dae)} by dae_to_obj.py\n')
        for p in v_all:
            f.write('v %.7g %.7g %.7g\n' % tuple(p))
        for uv in vt_all:
            f.write('vt %.7g %.7g\n' % tuple(uv))
        for n in vn_all:
            n = n / (np.linalg.norm(n) or 1.0)
            f.write('vn %.6g %.6g %.6g\n' % tuple(n))
        for cols in faces:
            v = cols['v'].reshape(-1, 3) + 1
            vt = cols['vt'].reshape(-1, 3) + 1 if 'vt' in cols else None
            vn = cols['vn'].reshape(-1, 3) + 1 if 'vn' in cols else None
            for k in range(len(v)):
                corners = []
                for c in range(3):
                    s = str(v[k, c])
                    if vt is not None or vn is not None:
                        s += '/' + (str(vt[k, c]) if vt is not None else '')
                    if vn is not None:
                        s += '/' + str(vn[k, c])
                    corners.append(s)
                f.write('f ' + ' '.join(corners) + '\n')


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    src, dst = sys.argv[1:]
    os.makedirs(dst, exist_ok=True)
    names = sorted(n for n in os.listdir(src) if n.lower().endswith('.dae'))
    if not names:
        sys.exit(f'no .dae in {src}')
    for n in names:
        convert(os.path.join(src, n), os.path.join(dst, os.path.splitext(n)[0] + '.obj'))
    print(f'{len(names)} meshes converted to {dst}')


if __name__ == '__main__':
    main()
