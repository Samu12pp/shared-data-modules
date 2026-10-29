'use strict';
const { Sim, V3, M33 } = require('./geofs_sim.js');
const file = process.argv[2];
const KT = 0.514444, FT = 0.3048;
function mk() {
  const s = new Sim(file); s.useGround = true;
  s.pos = [0, 0, s.def.startAltitude]; s.R = M33.rotationXYZ(M33.identity(), [(s.def.startTilt || 0) * Math.PI / 180, 0, 0]);
  s.v = [0, 0, 0]; s.w = [0, 0, 0]; s.controls.gear.position = s.controls.gear.target = 0;
  s.setValues(0.01); s.placeParts(); s.compute(); return s;
}
const dt = 1 / 60;
// --- static settle
let s = mk();
for (let i = 0; i < 60 * 25; i++) s.step(dt);
const st = s.state();
const comp = Object.fromEntries(s.suspensions.map(p => [p.name, +(s.values[p.name + 'Suspension'] || 0).toFixed(3)]));
console.log('static: CG height', s.pos[2].toFixed(3), 'm  pitch', st.pitch.toFixed(2), 'deg  compression', JSON.stringify(comp));
const nose = (s.values['nosewheelSuspension'] || 0) * s.parts.nosewheel.suspension.stiffness;
const mains = ['main_left_fwd', 'main_left_aft', 'main_right_fwd', 'main_right_aft'].reduce((a, n) => a + (s.values[n + 'Suspension'] || 0) * s.parts[n].suspension.stiffness, 0);
console.log('  nose load share', (100 * nose / (nose + mains)).toFixed(1), '%   (spring sum/g =', ((nose + mains) / 9.81).toFixed(3), ')');
console.log('  -> recommended startAltitude', s.pos[2].toFixed(2), ' startTilt', (-st.pitch).toFixed(2));
// --- takeoff with a pitch-tracking pilot (2.5 deg/s rotation to 12.5 deg)
function takeoff(flaps, vr, thr = 1.0, label = '') {
  const s = mk(); for (let i = 0; i < 60 * 10; i++) s.step(dt);
  s.controls.flaps.position = s.controls.flaps.target = flaps;
  s.controls.throttle = thr;
  let t = 0, x0 = s.pos[1], liftoff = null, maxPitch = 0, tailStrike = false, gearUp = false, air = 0, th = null, prevP = s.state().pitch, maxElev = 0, minClear = 99;
  while (t < 120) {
    const kts = V3.length(s.v) / KT; const st = s.state(); const q = (st.pitch - prevP) / dt; prevP = st.pitch;
    if (kts > vr) { th = th === null ? st.pitch : Math.min(12.5, th + 3.0 * dt); }
    if (th !== null) { s.controls.rawPitch = Math.max(-1, Math.min(1, 0.12 * (th - st.pitch) + 0.3 * ((th < 12.5 ? 3.0 : 0) - q) + (liftoff ? 0.15 : 0.1))); maxElev = Math.max(maxElev, s.controls.rawPitch); }
    s.step(dt); t += dt; maxPitch = Math.max(maxPitch, st.pitch);
    const wheelsOn = s.suspensions.some(p => (s.values[p.name + 'Suspension'] || 0) > 0);
    air = wheelsOn ? 0 : air + dt;
    if (!liftoff && air > 0.5) liftoff = { t: t - 0.5, kts, dist: s.pos[1] - x0, pitch: st.pitch };
    if (!liftoff) { const tail = s.parts.airframe.collisionPoints.filter(c => c[1] < -15).map(c => c.worldPosition[2] + s.pos[2]); minClear = Math.min(minClear, ...tail); }
    if (s.parts.airframe.contact) tailStrike = true;
    if (liftoff && !gearUp && s.pos[2] > 15) { s.setTarget('gear', 1); gearUp = true; }
    if (s.pos[2] * 3.28 > 1500) break;
  }
  const st = s.state();
  console.log(`takeoff ${label} F${['UP', '1', '5', '15', '20'][flaps]} thr ${(thr * 100).toFixed(0)}% Vr ${vr}: liftoff ${liftoff ? liftoff.kts.toFixed(0) + 'kt ' + liftoff.dist.toFixed(0) + 'm ' + liftoff.t.toFixed(1) + 's pitch ' + liftoff.pitch.toFixed(1) : 'NONE'} | tail clearance ${minClear.toFixed(2)}m strike ${tailStrike} | max elev ${maxElev.toFixed(2)} | @1500ft ${st.kts.toFixed(0)}kt ${st.vsFpm.toFixed(0)}fpm pitch ${st.pitch.toFixed(1)} t=${t.toFixed(0)}s`);
}
takeoff(2, 150); takeoff(2, 150, 0.85, 'derate85'); takeoff(3, 145); takeoff(1, 155); takeoff(2, 140, 1.0, 'early-Vr');
// --- landing rollout: touch at 135 kt with reversers + brakes + spoilers
{
  const s = mk(); for (let i = 0; i < 60 * 5; i++) s.step(dt);
  s.v = [0, 135 * KT, 0]; s.controls.flaps.position = s.controls.flaps.target = 6;
  s.controls.throttle = 0; s.setTarget('airbrakes', 1);
  let t = 0, x0 = s.pos[1];
  s.controls.throttle = -1; s.controls.brakes = 0.6;
  while (V3.length(s.v) > 1 && t < 120) { s.step(dt); t += dt; if (V3.length(s.v) < 60 * KT) s.controls.throttle = 0; }
  console.log(`landing roll from 135 kt (spoilers, full reverse to 60kt, brakes 0.6): ${(s.pos[1] - x0).toFixed(0)} m, ${t.toFixed(1)} s`);
}
