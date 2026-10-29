#!/usr/bin/env python3
"""Comprueba un aircraft.json (y cockpit/cockpit.json) de GeoFS contra sus modelos .glb.

Uso:
    python3 validate_geofs.py ruta/al/aircraft.json [--model-dir DIR] [--cockpit ruta/cockpit.json]

Detecta los errores que hacen que en GeoFS una pieza no se mueva, el avion no
cargue o la fisica de NaN:
  - nodos ("node") que no existen en el .glb que carga esa pieza
  - nombres de pieza duplicados o "parent" inexistentes
  - caracteres no ASCII (GeoFS pasa el archivo por btoa() y falla)
  - valores de animacion desconocidos
  - airfoils sin "area" ni "liftFactor" (lift = NaN)
  - piezas con colisiones sin contactProperties
  - piezas fisicas ligadas a un nodo sin "position" (GeoFS usaria
    coordenadas glTF, que tienen otros ejes)
"""
import argparse
import json
import os
import struct
import sys

KNOWN_VALUES = {
    "view", "cameraMode", "gearPosition", "invGearPosition", "gearTarget", "flapsValue", "flapsPosition",
    "flapsTarget", "flapsPositionTarget", "flapsMaxPosition", "airbrakesPosition", "airbrakesTarget",
    "optionalAnimatedPartPosition", "accessoriesPosition", "parkingBrake", "groundContact", "throttle",
    "pitch", "rawPitch", "roll", "yaw", "rawYaw", "trim", "brakes", "enginesOn", "engineVibration", "prop",
    "thrust", "rpm", "kias", "ktas", "kiasChangeRate", "groundSpeed", "groundSpeedKnt", "altitude",
    "altitudeMeters", "haglMeters", "haglFeet", "groundElevationFeet", "verticalSpeed", "climbrate", "aoa",
    "turnrate", "heading", "heading360", "atilt", "aroll", "acceleration", "accX", "accY", "accZ",
    "loadFactor", "slipball", "mach", "airTemp", "relativeWind", "windSpeed", "rollingSpeed", "night",
    "hours", "minutes", "season", "overlaysVisibility", "geofsTime", "strobe", "strobe2", "strobe3",
    "random", "envelopeTemp", "arrestingHookTension", "aircraft.maxAngularVRatio", "maxAngularVRatio",
    "NAV1OBS", "NAV1Course", "NAV1BearingToStation", "ADFBearingToStation", "navHDG", "ADFManCourse",
    "cameraAircraftSpeed", "cameraAircraftDistance",
}
KNOWN_LIGHTS = {"white", "red", "green", "1", "2", "3", "whitepapi", "redpapi"}
KNOWN_INCLUDES = {
    "3d-AirbusPFD", "3d-BoeingPFD", "3d-G1000", "3d-HUD", "3d-altimeter", "3d-attitude", "3d-attitude-jet",
    "3d-cdi", "3d-compass", "3d-hsi", "3d-ias", "3d-ias-high", "3d-vario", "3d-vario-high", "3d-jet-rpm",
    "3d-rpm", "3d-rmi", "3d-turn-coordinator", "3d-gmeter", "3d-manifold", "3d-oil", "3d-compassball",
}


def glb_node_names(path):
    with open(path, "rb") as f:
        data = f.read()
    magic, _, length = struct.unpack("<III", data[:12])
    if magic != 0x46546C67:
        raise ValueError(path + " no es un GLB")
    off = 12
    while off < length:
        clen, ctype = struct.unpack("<II", data[off:off + 8])
        if ctype == 0x4E4F534A:
            js = json.loads(data[off + 8:off + 8 + clen])
            return {n.get("name") for n in js.get("nodes", [])}
        off += 8 + clen
    raise ValueError(path + " sin chunk JSON")


def load_definition(path, problems):
    raw = open(path, "rb").read()
    try:
        raw.decode("ascii")
    except UnicodeDecodeError as e:
        problems.append(f"ERROR {os.path.basename(path)}: caracter no ASCII en el byte {e.start} (GeoFS usa btoa() y no carga el avion)")
    data = json.loads(raw.decode("utf-8"))
    if not isinstance(data, list):
        problems.append(f"ERROR {os.path.basename(path)}: el archivo debe ser una lista [ {{...}} ]")
        data = [data]
    return data[0]


def check(defn, model_dir, label, problems, extra_parts=(), suspension_parts=(), wheel_parts=()):
    parts = defn.get("parts", [])
    names = {}
    for p in parts:
        n = p.get("name")
        if n in names:
            problems.append(f"ERROR {label}: pieza duplicada '{n}' (la segunda sustituye a la primera)")
        names[n] = p
    all_names = set(names) | set(extra_parts)
    model_of = {}

    def owner_model(p, depth=0):
        if depth > 50:
            return None
        if p.get("model"):
            return os.path.join(model_dir, p["model"])
        par = p.get("parent") or ("root" if p.get("name") != "root" else None)
        if par and par in names:
            return owner_model(names[par], depth + 1)
        return None

    cache = {}
    susp = set(suspension_parts) | {p["name"] + "Suspension" for p in parts if p.get("suspension")}
    rot = set(wheel_parts) | {p["name"] + "Rotation" for p in parts if p.get("type") == "wheel"}
    for p in parts:
        n = p.get("name")
        par = p.get("parent")
        if par and par not in all_names:
            problems.append(f"ERROR {label}: '{n}' tiene parent '{par}' que no existe")
        if p.get("include") and p["include"] not in KNOWN_INCLUDES:
            problems.append(f"AVISO {label}: include desconocido '{p['include']}' en '{n}'")
        if p.get("light") and str(p["light"]) not in KNOWN_LIGHTS:
            problems.append(f"ERROR {label}: luz desconocida '{p['light']}' en '{n}'")
        if p.get("node"):
            mp = owner_model(p)
            if mp is None:
                problems.append(f"AVISO {label}: no se puede saber que modelo carga el nodo de '{n}'")
            elif not os.path.exists(mp):
                problems.append(f"ERROR {label}: falta el modelo {mp}")
            else:
                if mp not in cache:
                    cache[mp] = glb_node_names(mp)
                if p["node"] not in cache[mp]:
                    problems.append(f"ERROR {label}: el nodo '{p['node']}' (pieza '{n}') no existe en {os.path.basename(mp)}")
            physical = p.get("type") in ("airfoil", "wheel", "engine") or p.get("collisionPoints") or p.get("points") or p.get("light")
            if physical and not p.get("position"):
                problems.append(f"ERROR {label}: '{n}' es fisica y usa node sin 'position': GeoFS leeria la posicion en ejes glTF")
        if p.get("model") and not p.get("include"):
            mp = os.path.join(model_dir, p["model"])
            if not os.path.exists(mp):
                problems.append(f"ERROR {label}: falta el modelo {p['model']} (pieza '{n}')")
        if p.get("type") == "airfoil" and not p.get("area") and p.get("liftFactor") is None and not p.get("span"):
            problems.append(f"ERROR {label}: airfoil '{n}' sin area ni liftFactor -> fuerzas NaN")
        if p.get("type") in ("airfoil", "engine") and not (p.get("points") or {}).get("forceSourcePoint"):
            problems.append(f"ERROR {label}: '{n}' necesita points.forceSourcePoint")
        if p.get("collisionPoints"):
            ct = p.get("contactType") or p.get("type")
            if ct not in (defn.get("contactProperties") or {}) and label == "aircraft.json":
                problems.append(f"ERROR {label}: '{n}' tiene colisiones pero no hay contactProperties['{ct}']")
        for a in p.get("animations", []):
            if a.get("function"):
                problems.append(f"AVISO {label}: '{n}' usa 'function' (GeoFS lo ignora en aviones de la comunidad)")
                continue
            v = a.get("value")
            if v is None and a.get("type") not in ("render",):
                problems.append(f"ERROR {label}: animacion sin 'value' en '{n}'")
            elif v is not None and v not in KNOWN_VALUES and v not in susp and v not in rot:
                problems.append(f"ERROR {label}: valor de animacion desconocido '{v}' en '{n}' (siempre vale 0)")
            if a.get("type") == "property" and a.get("name") == "area" and p.get("liftFactor") is None:
                problems.append(f"AVISO {label}: '{n}' anima 'area' sin liftFactor de respaldo (si llega a 0 -> NaN)")
            if a.get("type") == "rotate" and a.get("axis") not in ("X", "Y", "Z"):
                problems.append(f"ERROR {label}: eje de rotacion invalido en '{n}'")
    return names


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("aircraft")
    ap.add_argument("--model-dir")
    ap.add_argument("--cockpit")
    args = ap.parse_args()
    problems = []
    model_dir = args.model_dir or os.path.dirname(os.path.abspath(args.aircraft))
    defn = load_definition(args.aircraft, problems)
    names = check(defn, model_dir, "aircraft.json", problems)
    ck = args.cockpit
    if ck is None:
        guess = os.path.join(os.path.dirname(os.path.abspath(args.aircraft)), "cockpit", "cockpit.json")
        ck = guess if os.path.exists(guess) else None
    if defn.get("cockpitModel") and not ck:
        problems.append("AVISO aircraft.json: cockpitModel=true pero no hay cockpit/cockpit.json")
    if ck:
        cdef = load_definition(ck, problems)
        check(cdef, os.path.dirname(os.path.abspath(ck)), "cockpit.json", problems, extra_parts=names.keys())
    errors = [p for p in problems if p.startswith("ERROR")]
    for p in problems:
        print(p)
    print(f"\n{len(errors)} errores, {len(problems) - len(errors)} avisos")
    sys.exit(1 if errors else 0)


if __name__ == "__main__":
    main()
