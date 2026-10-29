'use strict';
const { sim, trim, setRpm, KT, FT } = require('./flight_tests.js');
const { M33 } = require('./geofs_sim.js');
const clamp = (v, a, b) => Math.min(Math.max(v, a), b);
class PID { constructor(p, i, d) { this.kp = p; this.ki = i; this.kd = d; this.reset(); } reset() { this.max = this.min = this.sp = this.int = this.pe = this.pi = 0; }
  set(a, b, c) { this.min = b; this.max = c; this.sp = a; } init(a, b) { this.int = a || 0; this.pi = b || 0; }
  compute(a, dt) { const c = this.sp - a; this.int += c * dt * this.ki; this.int = clamp(this.int, this.min, this.max); const d = -(a - this.pi); this.pi = a; return clamp(this.kp * c + this.int + this.kd * d, this.min, this.max); } }
const D = Object.assign({ maxBankAngle: 30, maxPitchAngle: 20, minPitchAngle: -20, baseClimbrate: 1e3, baseDescentrate: -1e3, maxClimbrate: 6e3, maxDescentrate: -6e3, verticalSpeedHoldMargin: 1e3, targetBankAngleRatio: 2, yawBankAngleRatio: 0.005, pitchAnglePID: [0.01, 0.002, 0.01], elevatorPitchPID: [0.01, 0.001, 1e-7], bankAnglePID: [0.15, 0.001, 0.01], aileronsRollPID: [0.1, 0.001, 1e-7], throttlePID: [0.5, 0.1, 10], effectivenessRatioMaximum: 1.5 }, sim.def.autopilot);
const fixAngle = a => ((a % 360) + 540) % 360 - 180;
function run(label, alt0, kts0, tgt, T) {
  const t0 = trim(alt0, kts0, {});
  sim.pos = [0, 0, alt0 * FT]; sim.R = M33.rotationXYZ(M33.identity(), [-t0.pitch * Math.PI / 180, 0, 0]); sim.v = [0, kts0 * KT, 0]; sim.w = [0, 0, 0];
  sim.controls.rawPitch = t0.elev; sim.controls.throttle = t0.thr; setRpm(t0.thr); sim.setValues(0.016); sim.placeParts(); sim.compute();
  const P = { pitchAngle: new PID(...D.pitchAnglePID), elevatorPitch: new PID(...D.elevatorPitchPID), bankAngle: new PID(...D.bankAnglePID), aileronsRoll: new PID(...D.aileronsRollPID), throttle: new PID(...D.throttlePID) };
  const b = sim.values; const g0 = clamp(b.kias / 100, 1, D.effectivenessRatioMaximum);
  P.pitchAngle.init(-b.atilt, b.verticalSpeed); P.elevatorPitch.init(sim.controls.rawPitch * g0, -b.atilt); P.throttle.init(sim.controls.throttle, b.kias);
  let sm = { S: sim.controls.throttle, X: sim.controls.throttle };
  const dt = 1 / 60; let t = 0; const log = []; let maxBank = 0, minPitch = 99, maxPitch = -99, rawMax = 0;
  while (t < T) {
    const e = clamp(b.kias / 150, 1, 5), g = clamp(b.kias / 100, 1, D.effectivenessRatioMaximum);
    let f = fixAngle(((b.heading % 360) + 360) % 360 - tgt.course); f = clamp(f * D.targetBankAngleRatio, -D.maxBankAngle, D.maxBankAngle);
    P.bankAngle.set(f, -D.maxBankAngle, D.maxBankAngle); let k = P.bankAngle.compute(b.aroll, dt);
    P.aileronsRoll.set(k, -1, 1); sim.controls.roll = clamp(-P.aileronsRoll.compute(sim.controls.roll, dt) / g, -1, 1); sim.controls.yaw = clamp(-f * D.yawBankAngleRatio, -1, 1);
    const m = tgt.alt - b.altitude; const fc = clamp(e * D.baseClimbrate, 0, D.maxClimbrate), kd = clamp(e * D.baseDescentrate, D.maxDescentrate, 0);
    const tvs = clamp(2 * m * e, kd, fc);
    P.pitchAngle.set(tvs, D.minPitchAngle, D.maxPitchAngle); const dd = P.pitchAngle.compute(b.verticalSpeed, dt);
    P.elevatorPitch.set(dd, -1, 1); sim.controls.rawPitch = clamp(P.elevatorPitch.compute(-b.atilt, dt) / g, -1, 1);
    P.throttle.set(tgt.speed, 0, 1); const th = P.throttle.compute(b.kias, dt); const c = sm.X * 0.9 + 0.1 * sm.S; sm.S = c; sm.X = th; sim.controls.throttle = clamp(c, 0, 1);
    sim.step(dt); t += dt;
    const st = sim.state(); maxBank = Math.max(maxBank, Math.abs(st.bank)); minPitch = Math.min(minPitch, st.pitch); maxPitch = Math.max(maxPitch, st.pitch); rawMax = Math.max(rawMax, Math.abs(sim.controls.rawPitch));
    if (Math.abs(t % 20) < dt) log.push(`t${t.toFixed(0)}: ${st.altFt.toFixed(0)}ft ${st.kts.toFixed(0)}kt hdg ${((st.heading + 360) % 360).toFixed(0)} bank ${st.bank.toFixed(1)} pitch ${st.pitch.toFixed(1)} vs ${st.vsFpm.toFixed(0)} thr ${(sim.controls.throttle * 100).toFixed(0)}%`);
  }
  console.log(`== ${label}`); log.forEach(l => console.log('  ' + l));
  console.log(`  max bank ${maxBank.toFixed(1)}  pitch range ${minPitch.toFixed(1)}..${maxPitch.toFixed(1)}  max |elev| ${rawMax.toFixed(2)}`);
}
run('FL100 250kt -> 13000ft, hdg 090, 280kt', 10000, 250 * 1.16, { alt: 13000, course: 90, speed: 280 * 1.2 }, 200);
run('FL350 M0.80 -> FL310, hdg 330, hold speed', 35000, 464, { alt: 31000, course: 330, speed: 464 }, 220);
