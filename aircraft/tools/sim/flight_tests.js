'use strict';
const { Sim, V3, M33 } = require('./geofs_sim.js');
const file = process.argv[2];
const KT = 0.514444, FT = 0.3048;
const sim = new Sim(file);
const def = sim.def;
function setRpm(t) { for (const e of sim.engines) e.rpm = (def.maxRPM - def.minRPM) * t + def.minRPM; }
function setConfig({ flaps = 0, gear = 1, spoilers = 0 }) {
  sim.controls.flaps.position = sim.controls.flaps.target = flaps;
  sim.controls.gear.position = sim.controls.gear.target = gear;
  sim.controls.airbrakes.position = sim.controls.airbrakes.target = spoilers;
}
function evalState(altFt, kts, pitch, elev, thr, gamma = 0) {
  const V = kts * KT;
  sim.pos = [0, 0, altFt * FT]; sim.R = M33.rotationXYZ(M33.identity(), [-pitch * Math.PI / 180, 0, 0]);
  const g = gamma * Math.PI / 180; sim.v = [0, V * Math.cos(g), V * Math.sin(g)]; sim.w = [0, 0, 0];
  sim.controls.rawPitch = elev; sim.controls.throttle = thr; setRpm(Math.max(thr, 0));
  let f;
  for (let k = 0; k < 3; k++) { sim.setValues(0.01); sim.placeParts(); sim.compute(); f = sim.forces(); }
  return f;
}
// residuals along/perp flight path, pitch torque
function resid(altFt, kts, x, gamma) {
  const { F, T } = evalState(altFt, kts, x[0], x[1], x[2], gamma);
  const g = gamma * Math.PI / 180; const along = [0, Math.cos(g), Math.sin(g)], perp = [0, -Math.sin(g), Math.cos(g)];
  return [V3.dot(F, along) / 1e4, V3.dot(F, perp) / 1e4, T[0] / 1e5];
}
function solve(m, b) { // 3x3 gaussian
  const a = m.map((r, i) => [...r, b[i]]);
  for (let i = 0; i < 3; i++) { let p = i; for (let j = i + 1; j < 3; j++) if (Math.abs(a[j][i]) > Math.abs(a[p][i])) p = j; [a[i], a[p]] = [a[p], a[i]];
    for (let j = i + 1; j < 3; j++) { const f = a[j][i] / a[i][i]; for (let k = i; k < 4; k++) a[j][k] -= f * a[i][k]; } }
  const x = [0, 0, 0]; for (let i = 2; i >= 0; i--) { let s = a[i][3]; for (let k = i + 1; k < 3; k++) s -= a[i][k] * x[k]; x[i] = s / a[i][i]; } return x;
}
function trim(altFt, kts, cfg = {}, gamma = 0, fixThr = null) {
  setConfig(cfg);
  let x = [3, 0, 0.5];
  for (let it = 0; it < 60; it++) {
    const r0 = resid(altFt, kts, x, gamma);
    if (Math.hypot(...r0) < 1e-4) break;
    const J = [[], [], []];
    for (let j = 0; j < 3; j++) { const h = [0.01, 0.002, 0.002][j]; const xp = x.slice(); xp[j] += h; const r1 = resid(altFt, kts, xp, gamma); for (let i = 0; i < 3; i++) J[i][j] = (r1[i] - r0[i]) / h; }
    const dx = solve(J, r0.map(v => -v));
    for (let j = 0; j < 3; j++) x[j] += clampStep(dx[j], [2, 0.2, 0.2][j]);
    if (!x.every(isFinite)) { x = [NaN, NaN, NaN]; break; }
    x[2] = Math.max(0, Math.min(1, x[2]));
  }
  if (!x.every(isFinite)) return { ok: false, pitch: NaN, elev: NaN, thr: NaN, aoa: NaN, stalled: false, mach: 0, thrust: 0, rep: {} };
  const r = resid(altFt, kts, x, gamma);
  const ok = Math.hypot(...r) < 1e-3;
  return { ok, pitch: x[0], elev: x[1], thr: x[2], aoa: sim.values.aoa, stalled: sim.stalled, mach: sim.values.mach, thrust: sim.totalThrust, rep: sim.airfoilReport };
}
function clampStep(v, m) { return Math.max(-m, Math.min(m, v)); }
const fmt = (v, n = 2) => (v >= 0 ? ' ' : '') + v.toFixed(n);
function row(label, t, extra = '') {
  console.log(`${label.padEnd(34)} ${t.ok ? '  ' : '!!'} pitch ${fmt(t.pitch, 1)}  wingO ${fmt(t.aoa, 1)}  elev ${fmt(t.elev, 3)}  thr ${fmt(t.thr * 100, 0)}%  M ${t.mach.toFixed(3)}  T ${(t.thrust / 1000).toFixed(0)}kN ${t.stalled ? 'STALL' : ''} ${extra}`);
}
module.exports = { sim, trim, evalState, setConfig, setRpm, row, KT, FT };
if (require.main === module) {
  console.log('mass', def.mass, 'kg');
  console.log('--- level flight trims');
  const cases = [
    ['FL350 M0.80 clean', 35000, 0.80, {}], ['FL390 M0.80 clean', 39000, 0.80, {}], ['FL310 M0.80 clean', 31000, 0.80, {}],
    ['FL100 250kt clean', 10000, 250, {}], ['FL100 210kt clean', 10000, 210, {}], ['5000ft 180kt F5', 5000, 180, { flaps: 2 }],
    ['3000ft 165kt F15', 3000, 165, { flaps: 3 }], ['2000ft 150kt F20 gear', 2000, 150, { flaps: 4, gear: 0 }],
    ['2000ft 145kt F25 gear', 2000, 145, { flaps: 5, gear: 0 }], ['1500ft 140kt F30 gear', 1500, 140, { flaps: 6, gear: 0 }],
  ];
  for (const [label, alt, spd, cfg] of cases) {
    let kts = spd;
    if (spd < 1) { const T = 15 - alt * FT * 0.0065; kts = spd * (331.3 + 0.606 * T) / KT; }
    const t = trim(alt, kts, cfg); row(label + ` (${kts.toFixed(0)}KTAS)`, t, `L/D ${(def.mass * 9.81 / t.thrust).toFixed(1)}`);
  }
  console.log('--- 3 deg glideslope, 140 kt F30 gear down');
  row('glide -3deg F30', trim(1500, 140, { flaps: 6, gear: 0 }, -3));
  row('glide -3deg F25', trim(1500, 145, { flaps: 5, gear: 0 }, -3));
  console.log('--- idle descent (thr=0 search gamma) clean 280kt FL200');
  for (const g of [-2, -2.5, -3, -3.5, -4]) { const t = trim(20000, 280 * 1.35, {}, g); console.log(`  gamma ${g}: thr ${(t.thr * 100).toFixed(0)}% ${t.ok ? '' : '(no trim)'}`); }
  console.log('--- stall (1g, aero lift max, 2000 ft), CLmax on S=283.3');
  const names = ['UP', 'F1', 'F5', 'F15', 'F20', 'F25', 'F30'];
  for (let f = 0; f <= 6; f++) {
    setConfig({ flaps: f, gear: f >= 4 ? 0 : 1 });
    const V = 80 * KT; let best = { L: 0, p: 0 };
    for (let p = 0; p <= 30; p += 0.25) {
      const { F } = evalState(2000, 80, p, 0, 0);
      const L = F[2] + def.mass * 9.81; if (L > best.L) best = { L, p };
    }
    const rho = sim.rho; const CL = best.L / (0.5 * rho * V * V * 283.3);
    const Vs = Math.sqrt(2 * def.mass * 9.81 / (rho * 283.3 * CL)) / KT;
    console.log(`  ${names[f].padEnd(4)} CLmax ${CL.toFixed(2)} at pitch ${best.p.toFixed(1)}  Vs1g ${Vs.toFixed(0)} kt  Vref(1.23Vs) ${(1.23 * Vs).toFixed(0)} kt`);
  }
}
