const { Sim, V3, M33 } = require('./geofs_sim.js');
const KT = 0.514444, dt = 1 / 60;
function mk(file) { const s = new Sim(file); s.useGround = true; s.pos = [0, 0, s.def.startAltitude]; s.R = M33.rotationXYZ(M33.identity(), [(s.def.startTilt || 0) * Math.PI / 180, 0, 0]); s.v = [0, 0, 0]; s.w = [0, 0, 0]; s.setValues(0.01); s.placeParts(); s.compute(); for (let i = 0; i < 600; i++) s.step(dt); return s; }
const file = process.argv[2];
for (const flaps of [2, 3]) for (const e of [0.4, 0.55, 0.7, 0.85, 1.0]) {
  const s = mk(file); s.controls.flaps.position = s.controls.flaps.target = flaps; s.controls.throttle = 1;
  let t = 0, tr = null, noseOff = null, lof = null, maxRate = 0, prev = s.state().pitch, maxP = -9, strike = false;
  while (t < 80) {
    const kts = V3.length(s.v) / KT; const st = s.state(); const q = (st.pitch - prev) / dt; prev = st.pitch;
    if (kts > 150 && tr === null) tr = t;
    if (tr !== null) s.controls.rawPitch = st.pitch < 11 ? e : Math.max(0, e - 0.15 * (st.pitch - 11));
    s.step(dt); t += dt;
    if (tr !== null) { maxRate = Math.max(maxRate, q); maxP = Math.max(maxP, st.pitch); }
    if (tr !== null && noseOff === null && !(s.values.nosewheelSuspension > 0)) noseOff = { t: t - tr, kts };
    const on = s.suspensions.some(p => (s.values[p.name + 'Suspension'] || 0) > 0);
    if (!on && lof === null && kts > 100) lof = { t: t - tr, kts, p: st.pitch };
    if (s.parts.airframe.contact) strike = true;
    if (lof && t - tr > lof.t + 4) break;
  }
  console.log(`F${['UP','1','5','15'][flaps]} elev ${e}: nose off ${noseOff ? noseOff.t.toFixed(1) + 's ' + noseOff.kts.toFixed(0) + 'kt' : '-'} | liftoff ${lof ? lof.t.toFixed(1) + 's ' + lof.kts.toFixed(0) + 'kt pitch ' + lof.p.toFixed(1) : '-'} | max rate ${maxRate.toFixed(1)} deg/s max pitch ${maxP.toFixed(1)} strike ${strike}`);
}
