const { sim, trim, KT, FT } = require('./flight_tests.js');
const a = (alt) => 331.3 + 0.606 * (15 - alt * FT * 0.0065);
for (const alt of [10000, 25000, 35000, 39000]) {
  let lo = 0.5, hi = 1.1;
  for (let i = 0; i < 25; i++) { const m = (lo + hi) / 2; const t = trim(alt, m * a(alt) / KT, {}); if (t.ok && t.thr < 0.999) lo = m; else hi = m; }
  console.log(`max level Mach at ${alt} ft (full thrust): M${lo.toFixed(3)}  (${(lo * a(alt) / KT).toFixed(0)} KTAS)`);
}
for (const alt of [37000, 39000, 41000, 43000]) {
  const t = trim(alt, 0.80 * a(alt) / KT, {}, 0.5); // 0.5 deg climb
  const vs = Math.sin(0.5 * Math.PI / 180) * 0.80 * a(alt) * 196.85;
  console.log(`M0.80 at ${alt} ft: throttle for ~${vs.toFixed(0)} fpm climb = ${t.ok ? (t.thr * 100).toFixed(0) + '%' : 'n/a'} ${t.thr >= 0.999 ? '(limit)' : ''}`);
}
