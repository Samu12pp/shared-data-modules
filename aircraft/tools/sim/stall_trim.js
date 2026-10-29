const { sim, trim, setConfig, KT } = require('./flight_tests.js');
const def = sim.def; const names = ['UP', 'F1', 'F5', 'F15', 'F20', 'F25', 'F30'];
const wingStall = (f) => { const p = def.parts.find(p => p.name === 'wing_left'); const a = p.animations.find(a => a.name === 'stallIncidence'); return require('./geofs_sim.js').getRampRatio(a.valueRamp, f / 6); };
for (let f = 0; f <= 6; f++) {
  const cfg = { flaps: f, gear: f >= 4 ? 0 : 1 };
  let lo = 80, hi = 220;
  for (let i = 0; i < 30; i++) { const mid = (lo + hi) / 2; const t = trim(2000, mid, cfg); if (!t.ok || t.aoa > wingStall(f)) lo = mid; else hi = mid; }
  const t = trim(2000, hi, cfg);
  const CL = def.mass * 9.81 / (0.5 * sim.rho * (hi * KT) ** 2 * 283.3);
  console.log(`${names[f].padEnd(4)} trimmed Vs ${hi.toFixed(0)} KTAS (2000ft)  CLmax_trim ${CL.toFixed(2)}  pitch ${t.pitch.toFixed(1)}  elev ${t.elev.toFixed(2)}  -> Vref ${(1.23 * hi).toFixed(0)}`);
}
