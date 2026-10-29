'use strict';
const { sim, trim, setConfig, setRpm, KT, FT } = require('./flight_tests.js');
const { V3, M33 } = require('./geofs_sim.js');
const dt = 1 / 60;
function startTrimmed(alt, kts, cfg) {
  const t = trim(alt, kts, cfg);
  sim.pos = [0, 0, alt * FT]; sim.R = M33.rotationXYZ(M33.identity(), [-t.pitch * Math.PI / 180, 0, 0]);
  sim.v = [0, kts * KT, 0]; sim.w = [0, 0, 0]; sim.controls.rawPitch = t.elev; sim.controls.throttle = t.thr; setRpm(t.thr);
  sim.controls.roll = 0; sim.controls.yaw = 0; sim.setValues(dt); sim.placeParts(); sim.compute();
  return t;
}
function pitchStep(label, alt, kts, cfg, de) {
  const t0 = startTrimmed(alt, kts, cfg);
  let prev = sim.state().pitch, t = 0, peakQ = 0, peakN = 0, qs = [];
  while (t < 12) {
    sim.controls.rawPitch = t0.elev + (t > 1 && t < 4 ? de : 0);
    const vz0 = sim.v[2]; sim.step(dt); t += dt;
    const st = sim.state(); const q = (st.pitch - prev) / dt; prev = st.pitch; qs.push(q);
    const n = 1 + ((sim.v[2] - vz0) / dt) / 9.81; // approx (level start)
    if (t < 4.5) { peakQ = Math.max(peakQ, q); peakN = Math.max(peakN, n); }
  }
  // oscillation after release: count sign changes of q after t=4
  const after = qs.slice(Math.round(4 / dt)); let osc = 0; for (let i = 1; i < after.length; i++) if (Math.sign(after[i]) !== Math.sign(after[i - 1]) && Math.abs(after[i]) > 0.2) osc++;
  console.log(`${label.padEnd(26)} elev step +${de}: peak q ${peakQ.toFixed(2)} deg/s  peak n ${peakN.toFixed(2)} g  oscill.(q sign flips >0.2) ${osc}`);
}
function rollStep(label, alt, kts, cfg, da = 1) {
  startTrimmed(alt, kts, cfg);
  let t = 0, prev = sim.state().bank, maxP = 0, bank30 = null;
  while (t < 8) { sim.controls.roll = da; sim.step(dt); t += dt; const b = sim.state().bank; const p = (b - prev) / dt; prev = b; maxP = Math.max(maxP, p); if (bank30 === null && b > 30) bank30 = t; }
  console.log(`${label.padEnd(26)} full roll: max roll rate ${maxP.toFixed(1)} deg/s  time to 30 deg ${bank30 ? bank30.toFixed(1) + 's' : '>8s'}`);
}
function dutch(label, alt, kts, cfg) {
  startTrimmed(alt, kts, cfg);
  let t = 0; const hs = [];
  while (t < 30) { sim.controls.yaw = (t < 1) ? 0.5 : 0; sim.step(dt); t += dt; hs.push(sim.state().heading); }
  // yaw rate oscillation decay
  const r = hs.map((h, i) => i ? ((h - hs[i - 1] + 540) % 360 - 180) / dt : 0);
  let peaks = []; for (let i = 2; i < r.length - 1; i++) if (Math.abs(r[i]) > Math.abs(r[i - 1]) && Math.abs(r[i]) >= Math.abs(r[i + 1]) && i * dt > 1.5) peaks.push([i * dt, r[i]]);
  peaks = peaks.filter((p, i) => i === 0 || Math.sign(p[1]) !== Math.sign(peaks[i - 1][1])).slice(0, 5);
  console.log(`${label.padEnd(26)} rudder pulse: yaw-rate peaks ${peaks.map(p => p[0].toFixed(1) + 's:' + p[1].toFixed(2)).join('  ')}  final bank ${sim.state().bank.toFixed(1)}`);
}
pitchStep('approach 150kt F20', 2000, 150, { flaps: 4, gear: 0 }, 0.2);
pitchStep('climb 250kt clean', 10000, 250 * 1.16, {}, 0.2);
pitchStep('cruise M0.80 FL350', 35000, 464, {}, 0.1);
rollStep('approach 150kt F20', 2000, 150, { flaps: 4, gear: 0 });
rollStep('250kt clean FL100', 10000, 290, {});
rollStep('cruise M0.80 FL350', 35000, 464, {});
dutch('approach 150kt F20', 2000, 150, { flaps: 4, gear: 0 });
dutch('cruise M0.80 FL350', 35000, 464, {});
