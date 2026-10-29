#!/usr/bin/env python3
"""Mueve un nodo de un .glb para que sea hijo de otro sin cambiar su posicion en el mundo.

Uso: python3 glb_reparent.py entrada.glb salida.glb "NodoHijo" "NodoPadre"

Solo reescribe el chunk JSON (jerarquia y transformaciones); la geometria
(incluida la comprimida con Draco) se copia tal cual.
"""
import json
import struct
import sys

import numpy as np


def read_glb(path):
    data = open(path, "rb").read()
    magic, version, length = struct.unpack("<III", data[:12])
    assert magic == 0x46546C67, "no es un GLB"
    chunks, off = [], 12
    while off < length:
        clen, ctype = struct.unpack("<II", data[off:off + 8])
        chunks.append([ctype, data[off + 8:off + 8 + clen]])
        off += 8 + clen
    return version, chunks


def write_glb(path, version, chunks):
    body = b""
    for ctype, payload in chunks:
        pad = (4 - len(payload) % 4) % 4
        payload += (b" " if ctype == 0x4E4F534A else b"\0") * pad
        body += struct.pack("<II", len(payload), ctype) + payload
    open(path, "wb").write(struct.pack("<III", 0x46546C67, version, 12 + len(body)) + body)


def quat_to_mat(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def mat_to_quat(m):
    t = np.trace(m)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        return [(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s]
    i = int(np.argmax(np.diag(m)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = np.sqrt(1.0 + m[i, i] - m[j, j] - m[k, k]) * 2
    q = [0.0, 0.0, 0.0, 0.0]
    q[i] = 0.25 * s
    q[j] = (m[j, i] + m[i, j]) / s
    q[k] = (m[k, i] + m[i, k]) / s
    q[3] = (m[k, j] - m[j, k]) / s
    return q


def local(n):
    if "matrix" in n:
        return np.array(n["matrix"]).reshape(4, 4).T
    m = np.eye(4)
    m[:3, :3] = quat_to_mat(n.get("rotation", [0, 0, 0, 1])) @ np.diag(n.get("scale", [1, 1, 1]))
    m[:3, 3] = n.get("translation", [0, 0, 0])
    return m


def main():
    src, dst, child_name, parent_name = sys.argv[1:5]
    version, chunks = read_glb(src)
    js = json.loads(chunks[0][1])
    nodes = js["nodes"]
    by = {n.get("name"): i for i, n in enumerate(nodes)}
    ci, pi = by[child_name], by[parent_name]
    parent_of = {c: i for i, n in enumerate(nodes) for c in n.get("children", [])}

    def world(i):
        m = local(nodes[i])
        while i in parent_of:
            i = parent_of[i]
            m = local(nodes[i]) @ m
        return m

    original_world = world(ci)
    new_local = np.linalg.inv(world(pi)) @ original_world
    lin = new_local[:3, :3]
    det = np.linalg.det(lin)
    scale = np.linalg.norm(lin, axis=0)
    if det < 0:
        scale = -scale  # espejo: escala negativa (solo valida si es uniforme)
    rot = lin / scale
    if not np.allclose(rot @ rot.T, np.eye(3), atol=1e-4):
        sys.exit("la transformacion resultante tiene cizalla; no se puede expresar como TRS")
    n = nodes[ci]
    for k in ("matrix", "translation", "rotation", "scale"):
        n.pop(k, None)
    n["translation"] = [float(v) for v in new_local[:3, 3]]
    n["rotation"] = [float(v) for v in mat_to_quat(rot)]
    n["scale"] = [float(v) for v in scale]
    # quitar del padre/escena anterior y colgar del nuevo padre
    if ci in parent_of:
        nodes[parent_of[ci]]["children"].remove(ci)
    for sc in js.get("scenes", []):
        if ci in sc.get("nodes", []):
            sc["nodes"].remove(ci)
    nodes[pi].setdefault("children", []).append(ci)
    chunks[0][1] = json.dumps(js, separators=(",", ":")).encode()
    write_glb(dst, version, chunks)
    parent_of.clear()
    parent_of.update({c: i for i, nd in enumerate(nodes) for c in nd.get("children", [])})
    err = np.abs(world(ci) - original_world).max()
    print(f"{child_name} -> hijo de {parent_name}; error de transformacion {err:.2e}")


if __name__ == "__main__":
    main()
