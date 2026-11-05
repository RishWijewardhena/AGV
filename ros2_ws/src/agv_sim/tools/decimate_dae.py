"""Shrink a Fusion-exported COLLADA mesh for simulation by decimating every part separately.

The Fusion URDF exporter writes one <geometry> with a shared position array and one
<triangles> block per CAD body and material. Each block is simplified on its own, so
parts keep their materials; everything outside <library_geometries> is copied unchanged.
The file is read through mmap because the source can be hundreds of MB.

    venv/bin/python ros2_ws/src/agv_sim/tools/decimate_dae.py base_link.dae base_link_light.dae \
        --faces 150000
"""

import argparse
import mmap
import re

import fast_simplification
import numpy as np


def floats(data, start, end):
    return np.fromstring(data[start:end].decode(), sep=' ')


def array_body(data, array_id):
    """(start, end) of the text inside the float_array with this id."""
    tag = data.find(f'<float_array id="{array_id}"'.encode())
    start = data.find(b'>', tag) + 1
    return start, data.find(b'</float_array>', start)


def blocks(data, start, end):
    """Yield (material, triangles, index stride, vertex offset, start, end of <p> text) per block."""
    for match in re.finditer(rb'<triangles material="([^"]+)" count="(\d+)">', data[start:end]):
        head = start + match.end()
        p = data.find(b'<p>', head)
        offsets = [int(o) for o in re.findall(rb'offset="(\d+)"', data[head:p])]
        vertex = int(re.search(rb'semantic="VERTEX"[^>]*offset="(\d+)"', data[head:p]).group(1))
        yield (match.group(1).decode(), int(match.group(2)), max(offsets) + 1, vertex,
               p + 3, data.find(b'</p>', p))


def proper(faces):
    """Faces whose three corners are different points."""
    return faces[(faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2]) & (faces[:, 0] != faces[:, 2])]


def simplify(points, faces, keep):
    """Weld duplicate points, decimate to about `keep` faces, keep the part inside its own box.

    Quadric decimation may move a point off thin parts; clipping stops it leaving the part.
    """
    points, inverse = np.unique(np.round(points, 6), axis=0, return_inverse=True)
    faces = proper(inverse.reshape(-1)[faces])
    if len(faces) > keep:
        low, high = points.min(axis=0), points.max(axis=0)
        points, faces = fast_simplification.simplify(points, faces, target_reduction=1 - keep / len(faces))
        points = np.clip(points, low, high)
    return points, faces


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('source')
    parser.add_argument('output')
    parser.add_argument('--faces', type=int, default=150000, help='target triangles in total')
    parser.add_argument('--min-faces', type=int, default=400, help='blocks this small are kept as they are')
    args = parser.parse_args()

    with open(args.source, 'rb') as f:
        data = mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ)
    lib_start = data.find(b'<library_geometries>')
    lib_end = data.find(b'</library_geometries>') + len(b'</library_geometries>')
    geometry = re.search(rb'<geometry id="([^"]+)" name="([^"]+)">', data[lib_start:lib_start + 500])
    geo_id, geo_name = geometry.group(1).decode(), geometry.group(2).decode()
    positions = floats(data, *array_body(data, f'{geo_id}-positions-array')).reshape(-1, 3)

    found = list(blocks(data, lib_start, lib_end))
    weight = sum(count ** 0.5 for _, count, *_ in found)    # sqrt: small parts keep relatively more detail
    parts, out_points, n_points, n_in, n_out = [], [], 0, 0, 0
    for material, count, stride, vertex, start, end in found:
        index = np.fromstring(data[start:end].decode(), dtype=np.int64, sep=' ')
        faces = index.reshape(-1, stride)[:, vertex].reshape(-1, 3)
        used, local = np.unique(faces, return_inverse=True)
        keep = max(args.min_faces, int(args.faces * count ** 0.5 / weight))
        points, faces = simplify(positions[used], local.reshape(-1, 3), keep)
        parts.append((material, faces + n_points))
        out_points.append(points)
        n_points += len(points)
        n_in += count
        n_out += len(faces)
    points = np.concatenate(out_points)

    a, b, c = (points[np.concatenate([f for _, f in parts])[:, k]] for k in range(3))
    normals = np.cross(b - a, c - a)
    normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)

    with open(args.output, 'w', encoding='utf-8') as out:
        out.write(data[:lib_start].decode())
        out.write(f'<library_geometries>\n<geometry id="{geo_id}" name="{geo_name}">\n<mesh>\n')
        for kind, values in (('positions', points), ('normals', normals)):
            out.write(f'<source id="{geo_id}-{kind}">\n'
                      f'<float_array id="{geo_id}-{kind}-array" count="{values.size}">')
            np.savetxt(out, values.reshape(1, -1), fmt='%.5f', newline=' ')
            out.write(f'</float_array>\n<technique_common><accessor source="#{geo_id}-{kind}-array" '
                      f'count="{len(values)}" stride="3"><param name="X" type="float"/>'
                      '<param name="Y" type="float"/><param name="Z" type="float"/>'
                      '</accessor></technique_common>\n</source>\n')
        out.write(f'<vertices id="{geo_id}-vertices"><input semantic="POSITION" '
                  f'source="#{geo_id}-positions"/></vertices>\n')
        face = 0
        for material, faces in parts:
            normal = np.repeat(np.arange(face, face + len(faces)), 3).reshape(-1, 3)
            out.write(f'<triangles material="{material}" count="{len(faces)}">'
                      f'<input semantic="VERTEX" source="#{geo_id}-vertices" offset="0"/>'
                      f'<input semantic="NORMAL" source="#{geo_id}-normals" offset="1"/><p>')
            np.savetxt(out, np.stack([faces, normal], axis=2).reshape(1, -1), fmt='%d', newline=' ')
            out.write('</p></triangles>\n')
            face += len(faces)
        out.write('</mesh>\n</geometry>\n</library_geometries>')
        out.write(data[lib_end:].decode())
    print(f'{len(parts)} parts, {n_in} -> {n_out} triangles, {len(points)} points')


if __name__ == '__main__':
    main()
