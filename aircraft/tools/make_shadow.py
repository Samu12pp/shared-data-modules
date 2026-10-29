#!/usr/bin/env python3
"""Builds shadow.glb: unit quad (glTF X = GeoFS forward, glTF Z = GeoFS right) with a blurred silhouette texture."""
import io, json, struct, sys
from PIL import Image, ImageFilter

src, dst = sys.argv[1], sys.argv[2]
im = Image.open(src).convert("RGBA")
alpha = im.split()[3].filter(ImageFilter.GaussianBlur(3))
alpha = alpha.point(lambda a: int(a * 0.55))
tex = Image.new("RGBA", im.size, (0, 0, 0, 0)); tex.putalpha(alpha)
buf = io.BytesIO(); tex.save(buf, "PNG", optimize=True); png = buf.getvalue()

# quad: x (forward) and z (right) in [-0.5, 0.5], y = 0, normal +Y
pos = [(0.5, 0, -0.5), (0.5, 0, 0.5), (-0.5, 0, 0.5), (-0.5, 0, -0.5)]   # nose-left, nose-right, tail-right, tail-left
uv = [(0, 0), (1, 0), (1, 1), (0, 1)]                                      # image top = nose, image right = right wing
nrm = [(0, 1, 0)] * 4
idx = [0, 1, 2, 0, 2, 3]
def pack(fmt, rows): return b"".join(struct.pack(fmt, *r) for r in rows)
b_pos, b_nrm, b_uv = pack("<3f", pos), pack("<3f", nrm), pack("<2f", uv)
b_idx = struct.pack("<6H", *idx) + b"\0\0"
blob = b""; views = []
for data, target in ((b_pos, 34962), (b_nrm, 34962), (b_uv, 34962), (b_idx, 34963), (png, None)):
    v = {"buffer": 0, "byteOffset": len(blob), "byteLength": len(data)}
    if target: v["target"] = target
    views.append(v); blob += data + b"\0" * ((4 - len(data) % 4) % 4)
gltf = {
    "asset": {"version": "2.0", "generator": "make_shadow.py"},
    "extensionsUsed": ["KHR_materials_unlit"],
    "scenes": [{"nodes": [0]}], "scene": 0,
    "nodes": [{"name": "shadow", "mesh": 0}],
    "meshes": [{"primitives": [{"attributes": {"POSITION": 0, "NORMAL": 1, "TEXCOORD_0": 2}, "indices": 3, "material": 0}]}],
    "materials": [{"name": "shadow", "alphaMode": "BLEND", "doubleSided": True, "extensions": {"KHR_materials_unlit": {}},
                   "pbrMetallicRoughness": {"baseColorTexture": {"index": 0}, "metallicFactor": 0, "roughnessFactor": 1}}],
    "textures": [{"source": 0, "sampler": 0}], "samplers": [{"magFilter": 9729, "minFilter": 9729, "wrapS": 33071, "wrapT": 33071}],
    "images": [{"bufferView": 4, "mimeType": "image/png"}],
    "accessors": [
        {"bufferView": 0, "componentType": 5126, "count": 4, "type": "VEC3", "min": [-0.5, 0, -0.5], "max": [0.5, 0, 0.5]},
        {"bufferView": 1, "componentType": 5126, "count": 4, "type": "VEC3"},
        {"bufferView": 2, "componentType": 5126, "count": 4, "type": "VEC2"},
        {"bufferView": 3, "componentType": 5123, "count": 6, "type": "SCALAR"}],
    "bufferViews": views, "buffers": [{"byteLength": len(blob)}],
}
js = json.dumps(gltf, separators=(",", ":")).encode(); js += b" " * ((4 - len(js) % 4) % 4)
out = struct.pack("<III", 0x46546C67, 2, 12 + 8 + len(js) + 8 + len(blob))
out += struct.pack("<II", len(js), 0x4E4F534A) + js + struct.pack("<II", len(blob), 0x004E4942) + blob
open(dst, "wb").write(out)
print("shadow.glb", len(out), "bytes")
