// Minimal re-implementation of the GeoFS 3.x flight model (ported from GeoFS.js:
// Object3D, geofs.animation.filter, placePart, flight.tick, rigidBody) so an
// aircraft.json can be tested offline. Flat earth, no wind/turbulence.
'use strict';
const fs = require('fs');

const GRAVITY = 9.81, DEG = Math.PI / 180, RAD2DEG = 180 / Math.PI, TWO_PI = 2 * Math.PI, HALF_PI = Math.PI / 2;
const DRAG_CONSTANT = 0.07, MIN_DRAG_COEF = 0.02, METERS_TO_FEET = 3.28084, MS_TO_KNOTS = 1.94384;
const KELVIN_OFFSET = 273.15, TEMPERATURE_LAPSE_RATE = 0.0065, IDEAL_GAS_CONSTANT = 8.31447, MOLAR_MASS_DRY_AIR = 0.0289644;
const GM_RL = (GRAVITY * MOLAR_MASS_DRY_AIR) / (IDEAL_GAS_CONSTANT * TEMPERATURE_LAPSE_RATE);
const AXIS_TO_INDEX = { X: 0, Y: 1, Z: 2 };
const AXIS_TO_VECTOR = { X: [1, 0, 0], Y: [0, 1, 0], Z: [0, 0, 1] };
const clamp = (v, a, b) => Math.min(Math.max(v, a), b);

const V3 = {
  add: (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]],
  sub: (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]],
  scale: (a, s) => [a[0] * s, a[1] * s, a[2] * s],
  mult: (a, b) => [a[0] * b[0], a[1] * b[1], a[2] * b[2]],
  dot: (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2],
  cross: (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]],
  length: (a) => Math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]),
  normalize: (a) => { const l = V3.length(a); return l ? V3.scale(a, 1 / l) : [0, 0, 0]; },
  dup: (a) => [a[0], a[1], a[2]],
  toRadians: (a) => [a[0] * DEG, a[1] * DEG, a[2] * DEG],
  rotate: (a, b, c) => { const d = V3.dot(a, b); a = V3.sub(a, V3.scale(b, d)); const e = V3.cross(b, a); return V3.add(V3.scale(b, d), V3.add(V3.scale(a, Math.cos(c)), V3.scale(e, Math.sin(c)))); },
};
// M33 stored as array of columns (as GeoFS)
const M33 = {
  identity: () => [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
  dup: (a) => [V3.dup(a[0]), V3.dup(a[1]), V3.dup(a[2])],
  multiply: (a, b) => { const r = []; for (let j = 0; j < 3; j++) r.push(M33.transform(a, b[j])); return r; },
  transform: (a, b) => [a[0][0] * b[0] + a[1][0] * b[1] + a[2][0] * b[2], a[0][1] * b[0] + a[1][1] * b[1] + a[2][1] * b[2], a[0][2] * b[0] + a[1][2] * b[1] + a[2][2] * b[2]],
  transformByTranspose: (a, b) => [V3.dot(a[0], b), V3.dot(a[1], b), V3.dot(a[2], b)],
  transpose: (a) => [[a[0][0], a[1][0], a[2][0]], [a[0][1], a[1][1], a[2][1]], [a[0][2], a[1][2], a[2][2]]],
  setFromEuler: (a) => {
    const b = Math.cos(a[0]), c = Math.sin(a[0]), d = Math.cos(a[1]), e = Math.sin(a[1]), g = Math.cos(a[2]), s = Math.sin(a[2]);
    return [[g * d + s * c * e, -s * d + g * c * e, b * e], [s * b, g * b, -c], [g * -e + s * c * d, -s * -e + g * c * d, b * d]];
  },
  rotationXYZ: (a, b) => M33.multiply(a, M33.setFromEuler(b)),
  rotationX: (a, b) => { const c = Math.cos(b), s = Math.sin(b); return M33.multiply(a, [[1, 0, 0], [0, c, -s], [0, s, c]]); },
  rotationY: (a, b) => { const c = Math.cos(b), s = Math.sin(b); return M33.multiply(a, [[c, 0, s], [0, 1, 0], [-s, 0, c]]); },
  rotationZ: (a, b) => { const c = Math.cos(b), s = Math.sin(b); return M33.multiply(a, [[c, -s, 0], [s, c, 0], [0, 0, 1]]); },
  rotationParentFrameX: (a, b) => { const c = Math.cos(b), s = Math.sin(b); return M33.multiply([[1, 0, 0], [0, c, -s], [0, s, c]], a); },
  rotationParentFrameY: (a, b) => { const c = Math.cos(b), s = Math.sin(b); return M33.multiply([[c, 0, s], [0, 1, 0], [-s, 0, c]], a); },
  rotationParentFrameZ: (a, b) => { const c = Math.cos(b), s = Math.sin(b); return M33.multiply([[c, -s, 0], [s, c, 0], [0, 0, 1]], a); },
  getOrientation: (a) => {
    let h, p, r;
    if (a[1][2] > 0.998) { h = Math.atan2(-a[2][0], -a[2][1]); p = -HALF_PI; r = 0; }
    else if (a[1][2] < -0.998) { h = Math.atan2(a[2][0], a[2][1]); p = HALF_PI; r = 0; }
    else { h = Math.atan2(a[1][0], a[1][1]); p = Math.asin(-a[1][2]); r = Math.atan2(a[0][2], a[2][2]); }
    return [h * RAD2DEG, p * RAD2DEG, r * RAD2DEG];
  },
};

function getRampRatio(a, b) {
  if (b < 0 || b > 1) return 0;
  let c = a.length - 1; const d = 1 / c; let e = Math.ceil(b / d); const g = clamp(e - 1, 0, c - 1); e = clamp(e, 1, c);
  c = a[g]; a = a[e];
  return a > c ? c + ((b - g * d) / d) * (a - c) : a + ((d - (b - g * d)) / d) * (c - a);
}
function getRampValue(a, b) { let c = 0; if (b > a[0] && b < a[3]) c = b < a[1] ? (b - a[0]) / (a[1] - a[0]) : b > a[2] ? 1 - (b - a[2]) / (a[3] - a[2]) : 1; return c; }
function filter(a, values, now) {
  let b;
  if (a.value === 'random') b = Math.random();
  else if (a['function']) b = 0; // community aircraft cannot run functions
  else b = a.text ? a.text : values[a.value] || 0;
  if (a.ramp) b = getRampValue(a.ramp, b);
  else if (a.valueRamp) b = getRampRatio(a.valueRamp, b);
  else if (a.ratioRamp) b *= getRampRatio(a.ratioRamp, b);
  if (a.value === 'strobe') b = (now % 1500 > 1400) ? 1 : 0;
  else if (a.value === 'strobe2') b = (now % 1800 > 1700) ? 1 : 0;
  if (a.floor) b = Math.floor(b);
  if (a.abs) b = Math.abs(b);
  if (a.between) b = b > a.between[0] && b < a.between[1] ? 1 : 0;
  if (a.delay) { b -= a.delay; b = clamp(b, 0, 1); if (a.delay < 0) b += a.delay; }
  if (a.threshold) b = b < a.threshold ? 0 : b - a.threshold;
  if (a.negthreshold) b = b > a.negthreshold ? 0 : b - a.negthreshold;
  if (a.eq) b = b == a.eq ? 1 : 0;
  if (a.notEq) b = b != a.notEq ? 1 : 0;
  if (a.gt) b = b > a.gt ? 1 : 0;
  if (a.lt) b = b < a.lt ? 1 : 0;
  if (a.min && b < a.min) b = a.min;
  if (a.max && b > a.max) b = a.max;
  if (a.when) b = a.when.some(w => w == b); else if (a.whenNot) b = !a.whenNot.some(w => w == b);
  if (a.preoffset) b += a.preoffset;
  if (a.log) b = Math.log(b);
  if (a.ratio) b *= a.ratio;
  if (a.power) b = Math.pow(b, a.power);
  if (a.offset) b += a.offset;
  if (a.set) b = b ? a.set : a.unset || 0;
  if (a.fmin && b < a.fmin) b = a.fmin;
  if (a.fmax && b > a.fmax) b = a.fmax;
  return b;
}

class Obj3D {
  constructor(p) {
    this.name = p.name; this.nodeName = p.node; this.children = []; this.parent = null;
    this.points = p.points || {}; this.collisionPoints = p.collisionPoints || [];
    for (const k in this.points) this.points[k].worldPosition = [0, 0, 0];
    for (const c of this.collisionPoints) c.worldPosition = [0, 0, 0];
    this.initialRotation = M33.rotationXYZ(M33.identity(), p.rotation || [0, 0, 0]);
    this.rotation = M33.dup(this.initialRotation);
    let pos = p.position;
    if (this.nodeName) { this.nodeOrigin = pos; pos = [0, 0, 0]; }
    this.initialPosition = V3.dup(pos || [0, 0, 0]); this.position = V3.dup(this.initialPosition);
    this.worldScale = [1, 1, 1];
  }
  addChild(c) { c.parent = this; this.children.push(c); }
  resetAnimatedTransform() { this.rotation = M33.dup(this.initialRotation); this.position = V3.dup(this.initialPosition); }
  setVectorWorldPosition(a) { a.worldPosition = V3.add(this.worldPosition, M33.transform(this.worldRotation, a)); return a.worldPosition; }
  compute() {
    if (this.parent) {
      this.worldRotation = M33.multiply(this.parent.worldRotation, this.rotation);
      let b = this.position;
      if (this.nodeName && this.nodeOrigin) b = V3.add(b, this.nodeOrigin);
      this.worldPosition = V3.add(this.parent.worldPosition, M33.transform(this.parent.worldRotation, b));
    } else { this.worldRotation = this.initialRotation; this.worldPosition = M33.transform(this.initialRotation, this.position); }
    for (const k in this.points) this.setVectorWorldPosition(this.points[k]);
    for (const c of this.collisionPoints) this.setVectorWorldPosition(c);
    for (const c of this.children) c.compute();
  }
}
for (const ax of ['X', 'Y', 'Z']) {
  Obj3D.prototype['rotate' + ax] = function (a) { this.rotation = M33['rotation' + ax](this.rotation, a); };
  Obj3D.prototype['rotateParentFrame' + ax] = function (a) { this.rotation = M33['rotationParentFrame' + ax](this.rotation, a); };
}

class Sim {
  constructor(defPath, opts = {}) {
    const def = JSON.parse(fs.readFileSync(defPath, 'utf8'))[0];
    this.def = def;
    this.parts = {}; this.airfoils = []; this.engines = []; this.wheels = []; this.collisionPoints = []; this.suspensions = [];
    const parts = def.parts.filter(p => !p.disabled);
    for (const p of parts) this.parts[p.name] = p;
    if (!this.parts.root) { const r = { name: 'root', position: [0, 0, 0] }; parts.push(r); this.parts.root = r; }
    const root = this.parts.root; root.position = root.position || [0, 0, 0]; root.points = root.points || {};
    root.points.centerOfMass = root.points.centerOfMass || V3.scale(root.position, -1);
    for (const e of parts) {
      if (e.include) continue; // visual includes only
      e.points = e.points || {}; e.type = e.type || false; e.animations = (e.animations || []).map(a => Object.assign({}, a));
      if (e.forceDirection) e.forceDirection = AXIS_TO_INDEX[e.forceDirection];
      if (e.rotation) e.rotation = V3.toRadians(e.rotation);
      e.object3d = new Obj3D(e);
      if (e.suspension) {
        const cp = e.collisionPoints[0]; let g;
        if (e.suspension.length) { e.suspension.origin = [cp[0], cp[1], cp[2] + e.suspension.length]; g = e.suspension.length; }
        else { e.suspension.origin = [cp[0], cp[1], 0]; g = -cp[2]; }
        e.suspension.restLength = g;
        e.animations.push({ type: 'translate', axis: e.suspension.axis || 'Z', value: e.name + 'Suspension', ratio: e.suspension.ratio || 1 });
        e.suspension.hardPoint = e.suspension.hardPoint || 0.5; e.suspension.rest = true;
        e.points.suspensionOrigin = V3.dup(e.suspension.origin); e.points.suspensionOrigin.worldPosition = [0, 0, 0];
        e.object3d.points = e.points;
        this.suspensions.push(e);
      }
      for (const g of e.animations) {
        g.ratio = g.ratio || 1; g.offset = g.offset || 0;
        if (g.delay) g.ratio /= 1 - Math.abs(g.delay);
        if (g.type === 'rotate') { let k = g.method || 'rotate'; if (g.frame === 'parent') k = 'rotateParentFrame'; g.rotationMethod = k + g.axis; }
        if (g.type === 'translate' && !Array.isArray(g.axis)) g.axis = AXIS_TO_VECTOR[g.axis];
      }
      if (e.type === 'wheel') { e.radius = e.radius || 1; e.arcDegree = e.radius * TWO_PI / 360; e.angularVelocity = 0; this.wheels.push(e); }
      if (e.type === 'airfoil') { e.lift = 0; this.airfoils.push(e); e.stalls = e.stalls || false; e.stallIncidence = e.stallIncidence || 12; e.zeroLiftIncidence = e.zeroLiftIncidence || 16; }
      if (e.type === 'engine') { e.rpm = 0; this.engines.push(e); }
      if (e.collisionPoints) {
        const cprops = def.contactProperties[e.contactType || e.type];
        if (!cprops) throw new Error('no contactProperties for part ' + e.name + ' (' + (e.contactType || e.type) + ')');
        for (const c of e.collisionPoints) { c.part = e; c.contactProperties = cprops; this.collisionPoints.push(c); }
      }
    }
    for (const e of parts) {
      if (e.include || e.name === 'root') continue;
      const parent = this.parts[e.parent || 'root'];
      if (!parent || !parent.object3d) throw new Error('bad parent for ' + e.name);
      parent.object3d.addChild(e.object3d);
    }
    this.mass = def.mass;
    let t = def.tensorFactor || 0.1; if (!Array.isArray(t)) t = [t, t, t];
    this.invInertia = [t[0] / this.mass, t[1] / this.mass, t[2] / this.mass];
    this.RPM2PropAS = (def.driveRatio / 60) * 360;
    this.invRPMRange = 1 / (def.maxRPM - def.minRPM);
    this.controls = { throttle: 0, rawPitch: 0, roll: 0, yaw: 0, trim: 0, brakes: 0, gear: { position: 0, target: 0 }, flaps: { position: 0, target: 0 }, airbrakes: { position: 0, target: 0 } };
    this.flapsMax = def.flapsPositions ? def.flapsPositions.length : def.flapsSteps;
    this.values = { view: 'follow' };
    this.now = 0;
    this.enginesOn = true;
    this.reset(opts);
  }
  reset({ alt = 0, speed = 0, pitch = 0, gearDown = true } = {}) {
    this.pos = [0, 0, alt];  // world ENU (aircraft origin = CG)
    this.R = M33.rotationXYZ(M33.identity(), [-pitch * DEG, 0, 0]); // body->world (columns: right, fwd, up)
    this.v = [0, speed, 0];
    this.w = [0, 0, 0];
    this.controls.gear.position = this.controls.gear.target = gearDown ? 0 : 1;
    for (const e of this.engines) e.rpm = this.enginesOn ? this.def.minRPM : 0;
    this.groundContact = false;
    this.setValues(0.01); this.placeParts(); this.compute();
  }
  atmosphere(altM) {
    const T0 = 15 + KELVIN_OFFSET;
    const T = 15 - altM * TEMPERATURE_LAPSE_RATE + KELVIN_OFFSET;
    const P = 101325 * Math.pow(clamp(1 - (altM * TEMPERATURE_LAPSE_RATE) / T0, 0, 1), GM_RL);
    return { rho: (P * MOLAR_MASS_DRY_AIR) / (IDEAL_GAS_CONSTANT * T), tempC: T - KELVIN_OFFSET };
  }
  compute() { const r = this.parts.root.object3d; r.initialRotation = this.R; r.compute(); }
  setValues(dt) {
    const c = this.controls, d = this.values;
    d.throttle = c.throttle; d.rawPitch = c.rawPitch; d.pitch = c.rawPitch + c.trim; d.roll = c.roll; d.yaw = c.yaw; d.trim = c.trim;
    d.brakes = c.brakes; d.gearPosition = c.gear.position; d.invGearPosition = 1 - c.gear.position; d.gearTarget = c.gear.target;
    d.flapsPosition = c.flaps.position; d.flapsValue = c.flaps.position / this.flapsMax; d.flapsTarget = c.flaps.target;
    d.airbrakesPosition = c.airbrakes.position; d.airbrakesTarget = c.airbrakes.target;
    d.enginesOn = this.enginesOn; d.groundContact = this.groundContact ? 1 : 0;
    d.altitude = this.pos[2] * METERS_TO_FEET; d.kias = V3.length(this.v) * MS_TO_KNOTS;
    d.thrust = this.totalThrust || 0; d.rpm = this.engines.length ? this.engines[0].rpm : 0;
    const atm = this.atmosphere(this.pos[2]);
    d.mach = V3.length(this.v) / (331.3 + 0.606 * atm.tempC);
    d.aoa = this.aoa || 0;
    const [h, p, r] = M33.getOrientation(this.R); d.heading = h; d.atilt = p; d.aroll = r;
    d.verticalSpeed = this.v[2] * 196.85;
  }
  animatePart(name, dt) {
    const c = this.controls[name], tt = this.def[name + 'TravelTime'];
    const target = c.target;
    if (c.position === target) return;
    if (!tt) { c.position = target; return; }
    const delta = c.delta != null ? c.delta : (c.delta = target - c.position);
    c.position += delta / (tt / dt);
    if ((delta < 0 && c.position <= target) || (delta > 0 && c.position >= target)) { c.position = target; c.delta = null; }
  }
  setTarget(name, t) { const c = this.controls[name]; c.target = t; c.delta = t - c.position; }
  placePart(a) {
    if (!a.object3d || !a.animations) return;
    a.object3d.resetAnimatedTransform();
    for (const d of a.animations) {
      let e = filter(d, this.values, this.now);
      if (d.type === 'rotate') a.object3d[d.rotationMethod](e * DEG);
      else if (d.type === 'translate') a.object3d.position = V3.add(a.object3d.position, V3.scale(d.axis, e));
      else if (d.type === 'property') a[d.name] = e;
    }
  }
  placeParts() { for (const k in this.parts) this.placePart(this.parts[k]); }
  // total force/torque (world frame) for current state; GeoFS torque convention: cross(F, r)
  forces() {
    const F = [0, 0, 0], T = [0, 0, 0];
    const applyForce = (f, r) => { F[0] += f[0]; F[1] += f[1]; F[2] += f[2]; const t = V3.cross(f, r); T[0] += t[0]; T[1] += t[1]; T[2] += t[2]; };
    const atm = this.atmosphere(this.pos[2]); const rho = atm.rho; this.rho = rho;
    const def = this.def;
    // engines
    let m = 1; const n = def.zeroThrustAltitude; if (n) m = clamp(n - this.pos[2] * METERS_TO_FEET, 0, n) / n;
    this.totalThrust = 0;
    for (const u of this.engines) {
      const Da = Math.abs(u.rpm); let ma = u.thrust; if (u.rpm < 0) ma = u.reverseThrust ? -u.reverseThrust : 0;
      const ea = ma * clamp(Da - def.minRPM, 0, def.maxRPM) * this.invRPMRange * m;
      u.currentThrust = ea; this.totalThrust += ea;
      applyForce(V3.scale(u.object3d.worldRotation[u.forceDirection], ea), u.points.forceSourcePoint.worldPosition);
    }
    const tas = V3.length(this.v);
    this.aoa = 0; this.stalled = false; this.airfoilReport = {};
    if (tas > 0.01) {
      const Ga = tas * tas; const vdir = V3.normalize(this.v);
      const Zc = V3.scale(vdir, -(def.dragCoefficient ? 0.5 * def.dragCoefficient * rho * Ga : def.dragFactor * Ga * rho));
      F[0] += Zc[0]; F[1] += Zc[1]; F[2] += Zc[2];
      this.airfoilReport._globalDrag = V3.length(Zc);
      for (const x of this.airfoils) {
        const fa = x.points.forceSourcePoint; const ib = x.object3d.worldRotation;
        const Fv = V3.add(this.v, V3.cross(fa.worldPosition, this.w));
        const vel = V3.length(Fv); const na = V3.normalize(Fv); const ha = vel * vel;
        const oa = ib[x.forceDirection]; const O = -V3.dot(oa, na); const lb = V3.cross(oa, na); const mb = V3.rotate(oa, lb, O);
        let Z, ja;
        if (x.area) {
          let ka = O * TWO_PI;
          if (x.stalls) { this.aoa = O * RAD2DEG; const M = Math.abs(this.aoa); if (M > x.stallIncidence) { this.stalled = true; ka *= 1 - clamp(M - x.stallIncidence, 0, 0.9 * x.zeroLiftIncidence) / x.zeroLiftIncidence; } }
          Z = 0.5 * ha * (MIN_DRAG_COEF + DRAG_CONSTANT * ka * ka) * rho;
          ja = rho * ha * 0.5 * x.area * ka;
        } else {
          let Ja = x.liftFactor; const pb = x.dragFactor;
          if (x.stalls) { this.aoa = O * RAD2DEG; const M = Math.abs(this.aoa); if (M > x.stallIncidence) { this.stalled = true; Ja *= 0.9 - clamp(M - x.stallIncidence, 0, x.zeroLiftIncidence) / x.zeroLiftIncidence; } }
          const Ka = O * ha; ja = Ja * Ka * rho; Z = pb * Math.abs(Ka) * rho;
        }
        if (!isFinite(ja) || !isFinite(Z)) throw new Error('NaN force in airfoil ' + x.name + ' (area=' + x.area + ' liftFactor=' + x.liftFactor + ')');
        x.lift = ja;
        this.airfoilReport[x.name] = { lift: ja, drag: Z, O: O * RAD2DEG };
        applyForce(V3.scale(mb, ja), fa.worldPosition);
        applyForce(V3.scale(na, -Z), fa.worldPosition);
      }
    }
    const g = [0, 0, -GRAVITY * this.mass];
    applyForce(g, this.parts.root.points.centerOfMass.worldPosition);
    return { F, T };
  }
  velAt(r) { return V3.add(this.v, V3.cross(r, this.w)); }
  applyImpulse(J, r) {
    this.v = V3.add(this.v, V3.scale(J, 1 / this.mass));
    const t = V3.cross(J, r); this.w = V3.add(this.w, [t[0] * this.invInertia[0], t[1] * this.invInertia[1], t[2] * this.invInertia[2]]);
  }
  jacobian(e, b, c, d) { // computeJacobian(0, b, c, d)
    const a = -(1 + e) * b; const inv = 1 / this.mass;
    const x = V3.cross(d, c); const y = [x[0] * this.invInertia[0], x[1] * this.invInertia[1], x[2] * this.invInertia[2]];
    return a / (inv + V3.dot(d, V3.cross(c, y)));
  }
  ground(dt) { // flat ground at z=0 (port of the suspension / wheel / standard contact code)
    const P = []; let Q = 0; this.groundContact = false;
    for (const z of this.collisionPoints) {
      const v = z.part; v.contact = null;
      const Tz = this.pos[2] + z.worldPosition[2]; const V = 0; const R = v.object3d.worldRotation; const H = [0, 0, 1];
      if (v.suspension) {
        const sa = v.points.suspensionOrigin;
        const ta = v.suspension.restLength - (sa.worldPosition[2] + this.pos[2] - V);
        const ua = clamp(ta / v.suspension.restLength, 0, 1); const qb = ua * v.suspension.restLength;
        if (ua > 0 && sa.worldPosition[2] >= z.worldPosition[2]) {
          const rb = V3.dot(H, R[2]);
          const y = { cp: z, normal: H, force: v.suspension.stiffness * qb, type: 'raycast', fwd: V3.cross(H, V3.normalize(R[0])), side: V3.cross(H, V3.normalize(R[1])) };
          if (ua >= v.suspension.hardPoint || rb < 0.4) { y.type = 'hardpoint'; y.penetration = V - Tz; Q = Math.max(Q, y.penetration); }
          v.contact = y; P.push(y);
          sa[2] = -ta; this.values[v.name + 'Suspension'] = ta; v.suspension.rest = false;
        } else if (!v.suspension.rest) { this.values[v.name + 'Suspension'] = 0; v.points.suspensionOrigin[2] = 0; v.suspension.rest = true; }
        this.placePart(v); v.object3d.compute();
      } else {
        const ba = V - Tz;
        if (ba >= 0) { Q = Math.max(Q, ba); const y = { cp: z, normal: H, penetration: ba, type: 'standard', fwd: V3.cross(H, V3.normalize(R[0])), side: V3.cross(H, V3.normalize(R[1])) }; v.contact = y; P.push(y); }
      }
    }
    this.maxImpact = 0;
    if (P.length) {
      this.groundContact = true;
      if (Q > 0.001) this.pos[2] += Q;
      for (const y of P) {
        const z = y.cp, v = z.part, N = z.contactProperties;
        const J = this.velAt(z.worldPosition); const W = V3.dot(y.normal, J); this.maxImpact = Math.max(this.maxImpact, Math.abs(W));
        let ca = 0;
        if (y.type === 'raycast' || y.type === 'hardpoint') { ca = (y.force - v.suspension.damping * W) * this.mass * dt; if (ca > 0) this.applyImpulse(V3.scale(y.normal, ca), z.worldPosition); }
        if ((y.type === 'standard' || y.type === 'hardpoint') && W < 0) { const Ta = this.jacobian(0, W, z.worldPosition, y.normal); this.applyImpulse(V3.scale(y.normal, Ta), z.worldPosition); ca = Ta; }
        let K = ca * N.frictionCoef; K = clamp(K, K, 2 * this.mass * dt * N.frictionCoef);
        if (v.type === 'wheel') {
          const wa = y.fwd, xa = y.side; const Ua = V3.dot(xa, J), ya = V3.dot(wa, J);
          const Va = this.jacobian(0, Ua, z.worldPosition, xa), Wa = this.jacobian(0, ya, z.worldPosition, wa);
          const za = Math.abs(Va), Aa = Math.abs(Wa); let Xa = 1, la = 1;
          if (Math.abs(ya) > N.lockSpeed) la = N.rollingFriction;
          const zb = this.def.brakeDamping || 3;
          if (this.controls.brakes > 0.05) la = clamp((K / (Aa * N.frictionCoef * zb)) * this.controls.brakes, 0, 1);
          if (za > K) Xa = clamp(K / (za * za), N.dynamicFriction, 1);
          this.applyImpulse(V3.scale(xa, Va * Xa), z.worldPosition);
          this.applyImpulse(V3.scale(wa, Wa * la), z.worldPosition);
        } else {
          const Za = V3.sub(J, V3.scale(y.normal, W)); const ab = V3.length(Za);
          if (ab) { const dirn = V3.normalize(Za); const bb = this.jacobian(0, ab, z.worldPosition, dirn); const Ca = Math.abs(bb); let cb = 1; if (Ca > K) cb = clamp(K / (Ca * Ca), N.dynamicFriction, 1); this.applyImpulse(V3.scale(dirn, cb * bb), z.worldPosition); }
        }
      }
    } else for (const s of this.suspensions) if (!s.suspension.rest) { this.values[s.name + 'Suspension'] = 0; s.points.suspensionOrigin[2] = 0; s.suspension.rest = true; }
  }
  step(dt) {
    const c = this.controls; const def = this.def;
    // engines rpm
    for (const u of this.engines) {
      if (this.enginesOn) {
        const da = (def.maxRPM - def.minRPM) * c.throttle + def.minRPM;
        u.rpm += (da - u.rpm) * def.engineInertia * dt;
        if (def.reverse) { if (u.rpm < def.minRPM && u.rpm > 0) u.rpm = -def.minRPM; if (u.rpm > -def.minRPM && u.rpm < 0) u.rpm = def.minRPM; }
      } else u.rpm = u.rpm < 1e-5 ? 0 : u.rpm - u.rpm * def.engineInertia * dt;
    }
    this.compute();
    const { F, T } = this.forces();
    if (this.useGround) this.ground(dt);
    // integrate (GeoFS: v += F/m dt ; w += I^-1 T dt ; rotation by rotationXYZ(R, R^T w dt))
    this.v = V3.add(this.v, V3.scale(F, dt / this.mass));
    this.w = V3.add(this.w, [T[0] * this.invInertia[0] * dt, T[1] * this.invInertia[1] * dt, T[2] * this.invInertia[2] * dt]);
    this.pos = V3.add(this.pos, V3.scale(this.v, dt));
    const a = M33.transformByTranspose(this.R, V3.scale(this.w, dt));
    this.R = M33.rotationXYZ(this.R, a);
    // re-orthonormalize
    let y = V3.normalize(this.R[1]); let x = V3.normalize(V3.cross(y, this.R[2])); let z = V3.cross(x, y); this.R = [x, y, z];
    this.compute();
    this.now += dt * 1000;
    this.animatePart('gear', dt); this.animatePart('flaps', dt); this.animatePart('airbrakes', dt);
    this.setValues(dt); this.placeParts();
  }
  htr() { return M33.getOrientation(this.R); }
  state() {
    const [h, p, r] = this.htr();
    return { altFt: this.pos[2] * METERS_TO_FEET, kts: V3.length(this.v) * MS_TO_KNOTS, gsKts: Math.hypot(this.v[0], this.v[1]) * MS_TO_KNOTS, vsFpm: this.v[2] * 196.85, pitch: -p, bank: -r, heading: h, aoa: this.aoa, thrust: this.totalThrust };
  }
}
module.exports = { Sim, V3, M33, filter, getRampRatio };
