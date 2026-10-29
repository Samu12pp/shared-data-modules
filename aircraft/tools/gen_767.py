#!/usr/bin/env python3
"""Generates aircraft.json + cockpit/cockpit.json for the 767-300ER(W).
All coordinates are GeoFS model frame: x right, y forward, z up (glTF (x,y,z) -> (z,x,y))."""
import json, sys, os, math

P = dict(
    MASS=145000, TENSOR=0.01,
    CG=[0.0, 1.0, -0.5],
    # wing (per side)
    W_AREA=97.6, W_X=12.0, W_Y=2.25, W_Z=-0.5, W_INC=2.7, DIHEDRAL=6.0,
    FLAP_DI=[0, 1.5, 4.0, 6.0, 6.8, 7.5, 8.0],          # incidence added per detent (UP,1,5,15,20,25,30)
    STALL=[16.2, 18.4, 20.4, 22.0, 22.8, 25.0, 26.0],   # wing stall incidence per detent
    ZEROLIFT=12.0, ROLL_IN=1.15, SPOILER_DUMP=0.35,
    PD_AREA=19.4, PD_ARM=20.0,                            # pitch damper pair (static moments cancel)
    FLAP_CP=[0, 0.05, 0.15, 0.3, 0.4, 0.5, 0.6],          # aft shift of wing centre of lift with Fowler flaps (m)
    AIL_OUT_AREA=8.0, AIL_OUT_X=21.0, AIL_OUT_Y=-6.6, AIL_OUT_Z=0.7, ROLL_OUT=18.0,
    # tail
    H_AREA=13.0, H_X=2.2, H_Y=-21.5, H_Z=1.1, H_INC=-0.6, ELEV=18.0, TRIM=14.0, ELEV_V0=170.0, ELEV_MIN=0.33,
    YD_AREA=28.0, YD_ARM=20.0,
    V_AREA=24.0, V_Y=-21.6, V_Z=5.7, RUDDER=12.0,
    # drag
    DRAG=2.48, INDUCED_RATIO=0.2406, WAVE_MCRIT=0.732, WAVE_RATIO=7.29, GEAR_DRAG=2.83,
    FLAPS_DRAG=[0, 0.14, 0.85, 1.7, 2.4, 4.25, 5.95], SPEEDBRAKE_DRAG=3.5,
    # engines
    THRUST=267000, REVERSE=90000, ZERO_THRUST_ALT=52960, MACH_LAPSE=0.45,
    ENG_X=8.10, ENG_Y=4.5, ENG_Z=-2.27,
    # gear (physics)
    NOSE_Y=22.58, NOSE_Z=-4.622, NOSE_R=0.475,
    MAIN_X=4.737, MAIN_Y=-0.667, MAIN_DY=0.70, MAIN_Z=-5.095, MAIN_R=0.585,
    SUSP_MAIN_LEN=0.5, SUSP_NOSE_LEN=0.45, K_MAIN=15.2, D_MAIN=2.4, K_NOSE=4.7, D_NOSE=0.6,
    STEER=35.0,
)
for a in sys.argv[2:]:
    k, v = a.split('=', 1); P[k] = json.loads(v)

def r(v, n=4):
    if isinstance(v, list): return [r(x, n) for x in v]
    return round(v, n) if isinstance(v, float) else v

def A(**kw): return kw

parts = []
add = parts.append
CG = P['CG']
add({"name": "root", "type": "root", "model": "767-300ER.glb", "position": r([-CG[0], -CG[1], -CG[2]]),
     "textures": [{"index": 0, "filename": "texture_"}],
     "animations": [A(type="justhide", value="view", eq="cockpit")]})

# ---------------------------------------------------------------- physics: airframe contact points
add({"name": "airframe", "comment": "Puntos de contacto del fuselaje/alas/motores (solo colisiones)", "contactType": "frame",
     "position": [0, 0, 0],
     "collisionPoints": r([
        [0, 27.36, -0.2], [0, 25.9, -1.55], [0, 21.0, -2.7], [0, 8.0, -2.76], [0, -6.0, -2.74], [0, -15.0, -2.5],
        [0, -19.5, -1.66], [0, -22.5, -1.05], [0, -25.5, -0.35], [0, -27.95, 0.8], [0, 20.0, 2.72], [0, -14.0, 2.72],
        [0, -26.7, 11.3], [0, -28.45, 11.49],
        [-24.83, -10.1, 3.96], [24.83, -10.1, 3.96], [-24.13, -7.44, 0.95], [24.13, -7.44, 0.95],
        [-24.13, -9.27, 0.9], [24.13, -9.27, 0.9], [-16.0, -3.0, 0.2], [16.0, -3.0, 0.2],
        [-8.1, 7.6, -3.68], [8.1, 7.6, -3.68], [-8.1, 3.4, -3.35], [8.1, 3.4, -3.35],
        [-9.12, -27.7, 1.6], [9.12, -27.7, 1.6]])})

# ---------------------------------------------------------------- physics: landing gear (no nodes)
def wheel(name, pos, cpz, radius, length, k, d, anims=()):
    # GeoFS resets the suspension origin to local z=-compression, so the part origin must sit at the
    # top of the strut travel: position z = wheel bottom + travel, collision point at z = -travel.
    p = [pos[0], pos[1], cpz + length]
    return {"name": name, "type": "wheel", "position": r(p), "radius": radius,
            "collisionPoints": [[0, 0, r(-length)]],
            "suspension": {"stiffness": k, "damping": d},
            "animations": list(anims)}
add(wheel("nosewheel", [0, P['NOSE_Y'], 0], P['NOSE_Z'], P['NOSE_R'], P['SUSP_NOSE_LEN'], P['K_NOSE'], P['D_NOSE'],
          [A(type="rotate", axis="Z", value="yaw", ratio=P['STEER'])]))
for side, sx in (("left", -1), ("right", 1)):
    for axle, dy in (("fwd", P['MAIN_DY']), ("aft", -P['MAIN_DY'])):
        add(wheel(f"main_{side}_{axle}", [sx * P['MAIN_X'], P['MAIN_Y'] + dy, 0], P['MAIN_Z'], P['MAIN_R'],
                  P['SUSP_MAIN_LEN'], P['K_MAIN'], P['D_MAIN']))

# ---------------------------------------------------------------- physics: engines
for i, sx in ((1, -1), (2, 1)):
    add({"name": f"engine{i}", "type": "engine", "thrust": P['THRUST'], "reverseThrust": P['REVERSE'],
         "forceDirection": "Y", "contrail": True, "position": r([sx * P['ENG_X'], P['ENG_Y'], P['ENG_Z']]),
         "points": {"forceSourcePoint": [0, 0, 0], "contrailAnchor": [0, -4.2, 0]},
         "animations": [A(type="pitch", value="mach", ratio=-P['MACH_LAPSE'], offset=1)]})

# ---------------------------------------------------------------- physics: drag elements (lift 0)
def drag(name, comment, anim, pos=(0, 0, 0)):
    return {"name": name, "comment": comment, "type": "airfoil", "liftFactor": 0, "dragFactor": 0,
            "forceDirection": "Y", "position": list(pos), "points": {"forceSourcePoint": [0, 0, 0]},
            "animations": [anim]}
CGp = r(CG)
add(drag("drag_induced", "Resistencia inducida ~ CL^2 (usa aoa del ala)",
         A(type="property", name="dragFactor", value="aoa", ratio=P['INDUCED_RATIO'], power=2, fmax=60), CGp))
add(drag("drag_wave", "Resistencia de onda por encima de Mach critico",
         A(type="property", name="dragFactor", value="mach", threshold=P['WAVE_MCRIT'], ratio=P['WAVE_RATIO'], power=4, fmax=40), CGp))
add(drag("drag_gear", "Tren fuera",
         A(type="property", name="dragFactor", value="invGearPosition", ratio=P['GEAR_DRAG']), r([0, 0, -3.5])))
add(drag("drag_flaps", "Flaps/slats",
         A(type="property", name="dragFactor", value="flapsValue", valueRamp=P['FLAPS_DRAG'], ratio=1), r([0, -1.5, -1.0])))
add(drag("drag_speedbrake", "Spoilers / aerofrenos",
         A(type="property", name="dragFactor", value="airbrakesPosition", ratio=P['SPEEDBRAKE_DRAG']), r([0, -2.0, 0.0])))

# ---------------------------------------------------------------- physics: tail
V0 = P['ELEV_V0']; AE = P['H_AREA']; FMIN = P['ELEV_MIN']
for side, sx in (("left", -1), ("right", 1)):
    pos = r([sx * P['H_X'], P['H_Y'], P['H_Z']])
    add({"name": f"stab_{side}", "comment": "Estabilizador horizontal (parte fija) + compensador; su area complementa a la del elevador para que la estabilidad no cambie con la velocidad",
         "type": "airfoil", "area": 0.0001, "liftFactor": 0, "dragFactor": 0, "forceDirection": "Z", "position": pos,
         "rotation": [r(-P['H_INC']), 0, 0], "points": {"forceSourcePoint": [0, 0, 0]},
         "animations": [A(type="rotate", axis="X", value="trim", ratio=P['TRIM']),
                        A(type="property", name="area", value="kias", ratio=r(-1.0 / (V0 * AE), 8), power=-1, offset=AE, fmin=0.0001, fmax=r(AE * (1 - FMIN)))]})
    add({"name": f"elevator_{side}", "comment": "Timon de profundidad: autoridad reducida con la velocidad (sensacion artificial tipo feel computer)",
         "type": "airfoil", "area": AE, "liftFactor": 0, "dragFactor": 0, "forceDirection": "Z", "position": pos,
         "rotation": [r(-P['H_INC']), 0, 0], "points": {"forceSourcePoint": [0, 0, 0]},
         "animations": [A(type="rotate", axis="X", value="trim", ratio=P['TRIM']),
                        A(type="rotate", axis="X", value="rawPitch", ratio=P['ELEV']),
                        A(type="property", name="area", value="kias", ratio=r(1.0 / (V0 * AE), 8), power=-1, fmin=r(AE * FMIN), fmax=AE)]})
add({"name": "fin", "comment": "Deriva + timon de direccion", "type": "airfoil", "area": P['V_AREA'], "forceDirection": "X",
     "position": r([0, P['V_Y'], P['V_Z']]), "points": {"forceSourcePoint": [0, 0, 0]},
     "animations": [A(type="rotate", axis="Z", value="yaw", ratio=-P['RUDDER'])]})

for nm, dy in (("yaw_damper_fwd", P['YD_ARM']), ("yaw_damper_aft", -P['YD_ARM'])):
    add({"name": nm, "comment": "Amortiguador de guinada (par delante/detras del CG, momentos estaticos anulados): amortigua el balanceo holandes",
         "type": "airfoil", "area": P['YD_AREA'], "liftFactor": 0, "dragFactor": 0, "forceDirection": "X",
         "position": r([CG[0], CG[1] + dy, CG[2]]), "points": {"forceSourcePoint": [0, 0, 0]}})

# ---------------------------------------------------------------- physics: outboard ailerons (only with flaps out)
for side, sx in (("left", -1), ("right", 1)):
    add({"name": f"aileron_out_{side}", "comment": "Aleron exterior: bloqueado con flaps arriba (como el 767 real)",
         "type": "airfoil", "area": 0.0001, "liftFactor": 0, "dragFactor": 0, "forceDirection": "Z",
         "position": r([sx * P['AIL_OUT_X'], P['AIL_OUT_Y'], P['AIL_OUT_Z']]), "rotation": [r(-P['W_INC']), 0, 0],
         "points": {"forceSourcePoint": [0, 0, 0]},
         "animations": [A(type="rotate", axis="X", value="roll", ratio=sx * P['ROLL_OUT']),
                        A(type="property", name="area", value="flapsValue", valueRamp=[0, 1, 1, 1, 1, 1, 1], ratio=P['AIL_OUT_AREA'], offset=0.0001)]})

# ---------------------------------------------------------------- physics: wings (last airfoils with stalls)
flap_inc = A(type="rotate", axis="X", value="flapsValue", valueRamp=P['FLAP_DI'], ratio=-1)
for side, sx in (("left", -1), ("right", 1)):
    add({"name": f"wing_{side}", "comment": "Semiala: incidencia + flaps + aleron interior + spoilers (lift dump)",
         "type": "airfoil", "area": P['W_AREA'], "liftFactor": 0, "dragFactor": 0, "forceDirection": "Z",
         "position": r([sx * P['W_X'], P['W_Y'], P['W_Z']]), "rotation": [r(-P['W_INC']), r(sx * P['DIHEDRAL']), 0],
         "points": {"forceSourcePoint": [0, 0, 0]}, "stalls": True, "stallIncidence": P['STALL'][0], "zeroLiftIncidence": P['ZEROLIFT'],
         "animations": [flap_inc,
                        A(type="translate", axis="Y", value="flapsValue", valueRamp=P['FLAP_CP'], ratio=-1),
                        A(type="rotate", axis="X", value="roll", ratio=sx * P['ROLL_IN']),
                        A(type="property", name="stallIncidence", value="flapsValue", valueRamp=P['STALL'], ratio=1),
                        A(type="property", name="area", value="airbrakesPosition", ratio=r(-P['SPOILER_DUMP'] * P['W_AREA']), offset=P['W_AREA'])]})
for nm, dy in (("pitch_damper_fwd", P['PD_ARM']), ("pitch_damper_aft", -P['PD_ARM'])):
    add({"name": nm, "comment": "Amortiguador de cabeceo: par delante/detras del CG con momentos estaticos que se anulan; solo actua al cabecear (efecto downwash/fuselaje)",
         "type": "airfoil", "area": P['PD_AREA'], "liftFactor": 0, "dragFactor": 0, "forceDirection": "Z",
         "position": r([CG[0], CG[1] + dy, CG[2]]), "rotation": [r(-P['W_INC']), 0, 0], "points": {"forceSourcePoint": [0, 0, 0]},
         "stalls": True, "stallIncidence": P['STALL'][0], "zeroLiftIncidence": P['ZEROLIFT'],
         "animations": [flap_inc, A(type="property", name="stallIncidence", value="flapsValue", valueRamp=P['STALL'], ratio=1)]})
add({"name": "aoa_probe", "comment": "Sonda: fija 'aoa' (incidencia del ala) para la resistencia inducida; no genera fuerza apreciable",
     "type": "airfoil", "area": 0.0001, "forceDirection": "Z", "position": CGp, "rotation": [r(-P['W_INC']), 0, 0],
     "points": {"forceSourcePoint": [0, 0, 0]}, "stalls": True, "stallIncidence": 89, "zeroLiftIncidence": 12,
     "animations": [flap_inc]})

# ---------------------------------------------------------------- visuals (exterior nodes)
def vis(name, node, anims, comment=None):
    d = {"name": name, "node": node, "animations": anims}
    if comment: d["comment"] = comment
    return d
FV = "flapsValue"
FLAP_ANG = [0, 1, 5, 15, 20, 25, 30]
# trailing-edge flaps: rotate (TE down) + Fowler travel aft (local +Z)
for nm, node, fowler in (("vis_flap_in_left", "Inboard Flap L", 0.9), ("vis_flap_in_right", "Inboard Flap R", 0.9),
                         ("vis_flap_out_left", "Outboard Flap Left", 1.1), ("vis_flap_out_right", "Outboard Flap Right", 1.1)):
    add(vis(nm, node, [A(type="rotate", axis="X", value=FV, valueRamp=FLAP_ANG, ratio=-1),
                       A(type="translate", axis="Z", value=FV, valueRamp=[0, 0.35, 0.6, 0.8, 0.9, 0.95, 1.0], ratio=fowler),
                       A(type="translate", axis="Y", value=FV, valueRamp=[0, 0, -0.03, -0.06, -0.08, -0.09, -0.1], ratio=1)]))
SLAT_ANG = [0, 14, 14, 14, 14, 22, 22]
for nm, node in (("vis_slat_in_left", "Slat Left 1"), ("vis_slat_in_right", "Slat Right 1"),
                 ("vis_slat_out_left", "Slat Left 2.001"), ("vis_slat_out_right", "Slat Right 2.001")):
    add(vis(nm, node, [A(type="rotate", axis="Z", value=FV, valueRamp=SLAT_ANG, ratio=1),
                       A(type="translate", axis="X", value=FV, valueRamp=[0, 0.30, 0.30, 0.30, 0.30, 0.42, 0.42], ratio=1),
                       A(type="translate", axis="Y", value=FV, valueRamp=[0, -0.08, -0.08, -0.08, -0.08, -0.14, -0.14], ratio=1)]))
add(vis("vis_aileron_out_left", "Aileron L.001", [A(type="rotate", axis="X", value="roll", ratio=-18)]))
add(vis("vis_aileron_out_right", "Aileron R.001", [A(type="rotate", axis="X", value="roll", ratio=18)]))
add(vis("vis_aileron_in_left", "Aileron iL", [A(type="rotate", axis="X", value="roll", ratio=-12)]))
add(vis("vis_aileron_in_right", "Aileron iR", [A(type="rotate", axis="X", value="roll", ratio=-12)]))
add(vis("vis_elevator_left", "Elevator L", [A(type="rotate", axis="X", value="rawPitch", ratio=25)]))
add(vis("vis_elevator_right", "Elevator R", [A(type="rotate", axis="X", value="rawPitch", ratio=25)]))
add(vis("vis_rudder", "Rudder", [A(type="rotate", axis="Y", value="yaw", ratio=-25)]))
for s in ("L", "R"):
    for i in range(1, 8):
        add(vis(f"vis_spoiler_{s}{i}", f"Spoiler {s}{i}", [A(type="rotate", axis="Z", value="airbrakesPosition", ratio=45 if i > 2 else 50)]))
# engines
add(vis("vis_fan_left", "Fanblades L", [A(type="rotate", axis="X", value="prop", ratio=-1)]))
add(vis("vis_fan_right", "Fanblades R", [A(type="rotate", axis="X", value="prop", ratio=-1)]))
add(vis("vis_reverser_left", "Reversers L", [A(type="translate", axis=[0, -1, 0], value="thrust", min=-60000, max=1, ratio=1e-05)]))
add(vis("vis_reverser_right", "Reversers R", [A(type="translate", axis=[0, -1, 0], value="thrust", min=-60000, max=1, ratio=6.93e-06)],
        "nodo escalado x1.4436 en el modelo: ratio corregido"))
# main gear
for s, sgn, wheels in (("left", 1, ("main_left_fwd",)), ("right", -1, ("main_right_fwd",))):
    S = "L" if s == "left" else "R"
    add(vis(f"vis_gear_strut_{s}", f"Gear Strut {S}", [A(type="rotate", axis="Z", value="gearPosition", ratio=sgn * 89.5)]))
    add(vis(f"vis_gear_bogie_{s}", f"Gear Suspension {S}",
            [A(type="translate", axis=[0, -1, 0], value=f"main_{s}_aftSuspension", ratio=1),
             A(type="rotate", axis="X", value="groundContact", ratio=-8, offset=8)],
            "sube con la compresion y se inclina (morro abajo) en el aire, como el 767"))
    add(vis(f"vis_gear_strutdoor_{s}", "Wings.013" if s == "left" else "Wings.014",
            [A(type="rotate", axis="X", value="gearPosition", ratio=sgn * 89.5)]))
    add(vis(f"vis_gear_door_{s}", f"Main Gear Door {S}",
            [A(type="rotate", axis="X", value="gearPosition", valueRamp=[0, 80, 80, 80, 80, 80, 80, 80, 80, 0], ratio=sgn)]))
# nose gear
add(vis("vis_nose_strut", "Cylinder.042", [A(type="rotate", axis="X", value="gearPosition", ratio=-118)]))
add(vis("vis_nose_wheels", "Nose Gear",
        [A(type="translate", axis="Z", value="nosewheelSuspension", ratio=1),
         A(type="rotate", axis="Z", value="yaw", ratio=P['STEER']),
         A(type="rotate", axis="Y", value="nosewheelRotation", ratio=-1)]))
add(vis("vis_nose_door_fwd_left", "BFGDL", [A(type="rotate", axis="X", value="gearPosition", valueRamp=[0, 80, 80, 80, 80, 80, 80, 80, 80, 0], ratio=-1)]))
add(vis("vis_nose_door_fwd_right", "BFGDL.001", [A(type="rotate", axis="X", value="gearPosition", valueRamp=[0, 80, 80, 80, 80, 80, 80, 80, 80, 0], ratio=1)]))
add(vis("vis_nose_door_aft_left", "SBGDL", [A(type="rotate", axis="X", value="gearPosition", ratio=75)]))
add(vis("vis_nose_door_aft_right", "SBGDR", [A(type="rotate", axis="X", value="gearPosition", ratio=-75)]))

# ---------------------------------------------------------------- lights
def light(name, color, pos, anims): return {"name": name, "light": color, "position": r(pos), "animations": anims}
ENG_OFF = A(type="justhide", value="enginesOn", notEq=1)
add(light("nav_left", "red", [-24.10, -7.30, 0.98], [A(type="show", value="enginesOn")]))
add(light("nav_right", "green", [24.10, -7.30, 0.98], [A(type="show", value="enginesOn")]))
add(light("nav_tail_left", "white", [-24.10, -9.20, 0.92], [A(type="show", value="enginesOn")]))
add(light("nav_tail_right", "white", [24.10, -9.20, 0.92], [A(type="show", value="enginesOn")]))
add(light("strobe_left", "white", [-24.13, -7.55, 0.98], [A(type="show", value="strobe"), ENG_OFF]))
add(light("strobe_right", "white", [24.13, -7.55, 0.98], [A(type="show", value="strobe"), ENG_OFF]))
add(light("strobe_tail", "white", [0, -27.95, 0.85], [A(type="show", value="strobe"), ENG_OFF]))
add(light("beacon_top", "red", [0, 3.0, 2.80], [A(type="show", value="strobe2"), ENG_OFF]))
add(light("beacon_bottom", "red", [0, 1.0, -2.80], [A(type="show", value="strobe2"), ENG_OFF]))
add(light("landing_left", "white", [-3.3, 6.9, -1.45], [A(type="show", value="invGearPosition", gt=0.95), ENG_OFF]))
add(light("landing_right", "white", [3.3, 6.9, -1.45], [A(type="show", value="invGearPosition", gt=0.95), ENG_OFF]))
add(light("taxi_light", "white", [0, 22.75, -3.1], [A(type="show", value="invGearPosition", gt=0.95), ENG_OFF]))

defn = {
    "comment": "// This aircraft is made by GT. This code MAY NOT be redistributed or used in any part of any code without written permission from GT and GT ONLY. If ANY part of this code is to be found in ANY other aircraft, Immediate action will be taken, including the contact of Xavier.",
    "comment2": "Boeing 767-300ER (winglets) - definicion rehecha para el modelo 767-300ER.glb. Ver README.md.",
    "mass": P['MASS'], "tensorFactor": P['TENSOR'], "dragFactor": P['DRAG'],
    "zeroThrustAltitude": P['ZERO_THRUST_ALT'],
    "gearTravelTime": 10, "flapsTravelTime": 4, "flapsSteps": 6, "airbrakesTravelTime": 1.5,
    "reverse": True, "brakeDamping": 3,
    "minRPM": 1000, "maxRPM": 10000, "driveRatio": 0.25, "engineInertia": 0.45, "startupTime": 20, "shutdownTime": 30,
    "maxLimits": [0.86, 43100],
    "Vspeeds": {"VS0": 111, "VS": 146, "VFE": 170, "VNO": 340, "VNE": 360},
    "startAltitude": 4.45, "startTilt": 1.1, "minimumSpeed": 250,
    "autopilot": {"maxBankAngle": 22, "maxClimbrate": 4000, "maxDescentrate": -4000, "baseClimbrate": 1800, "baseDescentrate": -1800},
    "cockpitModel": True, "cockpitShadowMapMaxDistance": 50,
    "instruments": {
        "airspeedJet": {"animations": [{"value": "view", "type": "show", "notEq": "cockpit"}]},
        "attitudeJet": {"animations": [{"value": "view", "type": "show", "notEq": "cockpit"}]},
        "altitude": {"animations": [{"value": "view", "type": "show", "notEq": "cockpit"}]},
        "varioJet": {"animations": [{"value": "view", "type": "show", "notEq": "cockpit"}]},
        "compass": {"animations": [{"value": "view", "type": "show", "notEq": "cockpit"}]},
        "rpmJet": {"animations": [{"value": "view", "type": "show", "notEq": "cockpit"}]},
        "spoilers": "", "brakes": "", "gear": "", "flaps": ""},
    "parts": parts,
    "contactProperties": {
        "wheel": {"frictionCoef": 15, "dynamicFriction": 0.01, "rollingFriction": 0.00001, "lockSpeed": 0.008, "damping": 1},
        "frame": {"frictionCoef": 2, "dynamicFriction": 0.01, "damping": 1},
        "airfoil": {"frictionCoef": 2, "dynamicFriction": 0.01, "damping": 1}},
    "soundSet": "player",
    "sounds": [
        {"id": "rpm1", "file": "sounds/777/rpm1.mp3", "effects": {"volume": {"value": "rpm", "ramp": [500, 2000, 4000, 6000]}, "pitch": {"value": "rpm", "ramp": [1000, 20000, 20000, 20000], "ratio": 1, "offset": 1}}},
        {"id": "rpm2", "file": "sounds/777/rpm2.mp3", "effects": {"volume": {"value": "rpm", "ramp": [1500, 3000, 6000, 8000]}, "pitch": {"value": "rpm", "ramp": [1000, 20000, 20000, 20000], "ratio": 1, "offset": 1}}},
        {"id": "rpm3", "file": "sounds/777/rpm3.mp3", "effects": {"volume": {"value": "rpm", "ramp": [3000, 4500, 8000, 15000]}, "pitch": {"value": "rpm", "ramp": [2500, 20000, 20000, 20000], "ratio": 1, "offset": 1}}},
        {"id": "rpm4", "file": "sounds/777/rpm4.mp3", "effects": {"volume": {"value": "rpm", "ramp": [6000, 8000, 10000, 10000]}, "pitch": {"value": "rpm", "ramp": [6000, 20000, 20000, 20000], "ratio": 1, "offset": 1}}},
        {"id": "rolling", "file": "sounds/tyres/rolling1.mp3", "fadeDuration": 500, "effects": {"volume": {"value": "rollingSpeed", "ramp": [0, 50, 1000, 1000]}}},
        {"id": "touch1", "file": "sounds/tyres/touch1.mp3", "effects": {"start": {"value": "maxAngularVRatio", "gt": 300}}},
        {"id": "startup", "file": "sounds/777/startup.mp3"},
        {"id": "shutdown", "file": "sounds/777/shutdown.mp3"}],
    "cameras": {
        "follow": {"distance": 80},
        "cockpit": {"position": [-0.80, 24.32, 1.01], "orientation": [0, -12, 0], "FOV": 1.35},
        "Ala izquierda": {"position": [-2.8, 12.0, 0.9], "orientation": [-125, -12, 0]},
        "Ala derecha": {"position": [2.8, 12.0, 0.9], "orientation": [125, -12, 0]},
        "Cola": {"position": [0, -31.5, 13.5], "orientation": [0, -10, 0]},
        "Tren": {"position": [1.2, 21.0, -3.3], "orientation": [180, -15, 0]}},
    "shadowBox": [25.0, 30.5],
}

cockpit_parts = [
    {"name": "cockpit", "model": "cockpit.glb", "animations": [A(type="hide", value="view", notEq="cockpit")]},
    {"name": "PFD", "include": "3d-BoeingPFD", "parent": "cockpit", "position": [-0.7595, 25.3909, 0.5562], "scale": [0.25, 0.25, 0.25], "rotation": [0, 0, 0]},
    {"name": "ck_yoke_wheel_L", "parent": "cockpit", "node": "Cylinder.005", "animations": [A(type="rotate", axis="X", value="roll", ratio=-70)]},
    {"name": "ck_yoke_wheel_R", "parent": "cockpit", "node": "Cylinder.008", "animations": [A(type="rotate", axis="X", value="roll", ratio=-70)]},
    {"name": "ck_yoke_column_L", "parent": "cockpit", "node": "Cylinder.006", "animations": [A(type="translate", axis="X", value="rawPitch", ratio=-2.37)]},
    {"name": "ck_yoke_column_R", "parent": "cockpit", "node": "Cylinder.007", "animations": [A(type="translate", axis="X", value="rawPitch", ratio=-2.37)]},
    {"name": "ck_thrust_levers", "parent": "cockpit", "node": "Cube", "animations": [A(type="rotate", axis="Z", value="throttle", min=0.0001, ratio=42)]},
    {"name": "ck_reverse_levers", "parent": "cockpit", "node": "Cube.004", "comment": "palancas de reversa (hijas de las de gases en cockpit.glb)", "animations": [A(type="rotate", axis="Z", value="throttle", max=-0.0001, ratio=55)]},
    {"name": "ck_speedbrake_lever", "parent": "cockpit", "node": "Cube.013", "animations": [A(type="rotate", axis="Z", value="airbrakesPosition", ratio=-40)]},
    {"name": "ck_flap_lever", "parent": "cockpit", "node": "Cube.014", "animations": [A(type="rotate", axis="Z", value=FV, valueRamp=[0, 7, 14, 21, 28, 35, 42], ratio=-1)]},
    {"name": "ck_gear_lever", "parent": "cockpit", "node": "Cylinder.009", "animations": [A(type="translate", axis=[6.8, -0.63, 0], value="gearTarget", ratio=1)]},
]
cockpit = {"comment": "Cabina del 767 (modelo de GT). Se carga al entrar en la vista de cabina.", "parts": cockpit_parts}

out = sys.argv[1]
os.makedirs(os.path.join(out, "cockpit"), exist_ok=True)
def dump(obj, path):
    txt = json.dumps([obj], indent=1, ensure_ascii=True)
    open(path, "w").write(txt + "\n")
dump(defn, os.path.join(out, "aircraft.json"))
dump(cockpit, os.path.join(out, "cockpit", "cockpit.json"))
print("written", out, len(parts), "parts")
