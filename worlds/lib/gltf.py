"""
Minimal glTF reading and writing for the garden meshes.

Gazebo loads glTF coordinates as they are (it does not turn Y-up to Z-up), so
the meshes written here are in the Gazebo Z-up frame. Texture coordinates
are written with v pointing down, as glTF has it.
"""

import json
import os

import numpy as np

COMPONENT = {5121: np.uint8, 5123: np.uint16, 5125: np.uint32, 5126: np.float32}
COUNT = {'SCALAR': 1, 'VEC2': 2, 'VEC3': 3, 'VEC4': 4}


def read(path):
    """Read a glTF: return (json, {buffer index: bytes})."""
    with open(path) as f:
        gltf = json.load(f)
    buffers = {}
    for i, buffer in enumerate(gltf['buffers']):
        with open(os.path.join(os.path.dirname(path), buffer['uri']), 'rb') as f:
            buffers[i] = f.read()
    return gltf, buffers


def accessor(gltf, buffers, index):
    """Data of a glTF accessor as a numpy array."""
    a = gltf['accessors'][index]
    view = gltf['bufferViews'][a['bufferView']]
    dtype = COMPONENT[a['componentType']]
    n = COUNT[a['type']]
    offset = view.get('byteOffset', 0) + a.get('byteOffset', 0)
    stride = view.get('byteStride', 0)
    data = buffers[view['buffer']]
    itemsize = np.dtype(dtype).itemsize * n
    if stride and stride != itemsize:
        raw = np.frombuffer(data, np.uint8, a['count'] * stride, offset).reshape(-1, stride)[:, :itemsize]
        return np.frombuffer(raw.tobytes(), dtype).reshape(a['count'], n)
    return np.frombuffer(data, dtype, a['count'] * n, offset).reshape(a['count'], n)


def mesh_data(gltf, buffers, mesh_index):
    """Positions, normals, uvs and triangle indices of the first primitive of a mesh."""
    p = gltf['meshes'][mesh_index]['primitives'][0]
    attrs = p['attributes']
    return (accessor(gltf, buffers, attrs['POSITION']).astype(np.float32),
            accessor(gltf, buffers, attrs['NORMAL']).astype(np.float32),
            accessor(gltf, buffers, attrs['TEXCOORD_0']).astype(np.float32),
            accessor(gltf, buffers, p['indices']).astype(np.uint32).reshape(-1, 3))


def write(path, positions, normals, uvs, faces, material=None, images=()):
    """
    Write a one-mesh glTF with a .bin next to it. 'material' is an optional
    glTF material dict and 'images' a list of image URIs it refers to by index.
    """
    positions = np.ascontiguousarray(positions, np.float32)
    normals = np.ascontiguousarray(normals, np.float32)
    uvs = np.ascontiguousarray(uvs, np.float32)
    faces = np.ascontiguousarray(faces, np.uint32).reshape(-1)
    blobs = [positions.tobytes(), normals.tobytes(), uvs.tobytes(), faces.tobytes()]
    views, offset = [], 0
    for i, blob in enumerate(blobs):
        views.append(dict(buffer=0, byteOffset=offset, byteLength=len(blob), target=34963 if i == 3 else 34962))
        offset += len(blob)
    accessors = [
        dict(bufferView=0, componentType=5126, count=len(positions), type='VEC3',
             min=positions.min(0).tolist(), max=positions.max(0).tolist()),
        dict(bufferView=1, componentType=5126, count=len(normals), type='VEC3'),
        dict(bufferView=2, componentType=5126, count=len(uvs), type='VEC2'),
        dict(bufferView=3, componentType=5125, count=len(faces), type='SCALAR'),
    ]
    primitive = dict(attributes=dict(POSITION=0, NORMAL=1, TEXCOORD_0=2), indices=3)
    gltf = dict(asset=dict(version='2.0', generator='kpi_rover garden'),
                scene=0, scenes=[dict(nodes=[0])], nodes=[dict(mesh=0, name='mesh')],
                meshes=[dict(name='mesh', primitives=[primitive])],
                accessors=accessors, bufferViews=views,
                buffers=[dict(uri=os.path.basename(path)[:-5] + '.bin', byteLength=offset)])
    if material:
        primitive['material'] = 0
        gltf['materials'] = [material]
        if images:
            gltf['images'] = [dict(uri=uri) for uri in images]
            gltf['textures'] = [dict(source=i) for i in range(len(images))]
    with open(path[:-5] + '.bin', 'wb') as f:
        for blob in blobs:
            f.write(blob)
    with open(path, 'w') as f:
        json.dump(gltf, f)
