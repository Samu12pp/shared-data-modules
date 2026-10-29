const { sim, trim, setRpm, KT, FT } = require('./flight_tests.js');
const { M33 } = require('./geofs_sim.js');
function spiral(label, alt, kts, cfg = {}) {
  const t0 = trim(alt, kts, cfg);
  sim.pos = [0, 0, alt * FT]; sim.R = M33.rotationXYZ(M33.identity(), [-t0.pitch * Math.PI / 180, 0, 0]); sim.v = [0, kts * KT, 0]; sim.w = [0, 0, 0];
  sim.controls.rawPitch = t0.elev; sim.controls.throttle = t0.thr; setRpm(t0.thr); sim.setValues(0.016); sim.placeParts(); sim.compute();
  const dt = 1 / 60; let t = 0; const out = [];
  // roll to ~20 deg then release
  while (t < 60) { const st = sim.state(); sim.controls.roll = (t < 6 && st.bank < 20) ? 0.5 : 0; sim.step(dt); t += dt; if (Math.abs(t % 10) < dt) out.push(`${t.toFixed(0)}s bank ${sim.state().bank.toFixed(1)}`); }
  console.log(label.padEnd(24), out.join(' | '));
}
spiral('250kt FL100 clean', 10000, 290);
spiral('cruise M0.80 FL350', 35000, 464);
spiral('approach 150 F20', 2000, 150, { flaps: 4, gear: 0 });
