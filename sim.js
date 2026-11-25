// Physics trial runner on Rapier. Torque limits: inside the limit an implicit PD motor holds the joint;
// when the PD demand exceeds `effort`, the motor is released and exactly ±effort is applied instead.
import * as THREE from 'three';

const G_GROUND = (0x0001 << 16) | 0x0002, G_ROBOT = (0x0002 << 16) | 0x0001;
const tq1 = new THREE.Quaternion(), tq2 = new THREE.Quaternion(), tv = new THREE.Vector3(), tv2 = new THREE.Vector3();

export const DEFAULTS = { mode: 'stand', freq: 1.5, amp: 0.3, off: Math.PI / 2, gain: 1, friction: 0.8, effortScale: 1, T: 4, dt: 1 / 500, settle: 0.6, iters: 4, pgs: null, internal: 1, contactHz: 300 };

export function createSim(RAPIER, model, opt) {
  const p = { ...DEFAULTS, ...opt };
  const world = new RAPIER.World({ x: 0, y: 0, z: -9.81 });
  world.timestep = p.dt; world.numSolverIterations = p.iters;
  if (p.pgs === 'small') world.switchToSmallStepsPgsSolver(); else if (p.pgs === 'standard') world.switchToStandardPgsSolver();
  world.numInternalPgsIterations = p.internal;
  // Default contact softness (30 Hz) lets a heavy body tip a light foot over its edge; stiffer contacts fix it.
  world.integrationParameters.contact_natural_frequency = p.contactHz;
  const gb = world.createRigidBody(RAPIER.RigidBodyDesc.fixed().setTranslation(0, 0, -0.5));
  world.createCollider(RAPIER.ColliderDesc.cuboid(200, 200, 0.5).setFriction(p.friction).setCollisionGroups(G_GROUND), gb);

  const rbs = model.bodies.map(B => {
    const d = RAPIER.RigidBodyDesc.dynamic().setTranslation(B.pos.x, B.pos.y, B.pos.z).setCanSleep(false)
      .setAdditionalMassProperties(B.m, xyz(B.com), { x: B.inertia[0], y: B.inertia[1], z: B.inertia[2] }, xyzw(B.iframe));
    const rb = world.createRigidBody(d);
    for (const s of B.shapes) {
      const c = s.type === 'box' ? RAPIER.ColliderDesc.cuboid(...s.half) : s.type === 'sphere' ? RAPIER.ColliderDesc.ball(s.r)
        : s.type === 'cylinder' ? RAPIER.ColliderDesc.cylinder(s.hh, s.r)
        : s.type === 'hull' ? (RAPIER.ColliderDesc.convexHull(s.pts) || RAPIER.ColliderDesc.ball(0.005))
        : RAPIER.ColliderDesc.capsule(s.hh, s.r);
      c.setTranslation(s.p.x, s.p.y, s.p.z).setRotation(xyzw(s.q)).setDensity(0).setFriction(p.friction).setCollisionGroups(G_ROBOT);
      world.createCollider(c, rb);
    }
    return rb;
  });

  const J = model.joints.map(j => {
    const data = j.type === 'prismatic' ? RAPIER.JointData.prismatic(xyz(j.a1), { x: 0, y: 0, z: 0 }, xyz(j.axisW))
      : RAPIER.JointData.revolute(xyz(j.a1), { x: 0, y: 0, z: 0 }, xyz(j.axisW));
    const h = world.createImpulseJoint(data, rbs[j.pb], rbs[j.cb], true);
    h.setContactsEnabled(false);
    if (j.lower !== null) h.setLimits(j.lower - j.q0, j.upper - j.q0); // Rapier measures from the rest pose
    h.configureMotorModel(RAPIER.MotorModel.ForceBased);
    const kp = p.gain * j.kp0, kd = Math.min(2 * 0.7 * Math.sqrt(kp * j.Iload), 0.1 * kp);
    return { j, h, kp, kd, eff: j.effort * p.effortScale, sat: false,
      st: { util: 0, sat: 0, peak: 0, vpeak: 0, lim: 0, E: 0 } };
  });

  const root = rbs[model.rootIdx], z0 = model.bodies[model.rootIdx].pos.z, x0 = model.bodies[model.rootIdx].pos.x, y0 = model.bodies[model.rootIdx].pos.y;
  const S = { t: 0, n: 0, nUp: 0, fallen: false, tFall: null, maxTilt: 0, series: [], E: 0, startX: null, startY: null };

  function target(c, t) {
    const j = c.j;
    let center = j.q0, a = 0;
    if (p.mode === 'gait' && j.osc && (j.swing || j.lift)) {
      // Sprawled leg: swing = A·sin φ (forward while sin rises); lift raises the foot only during that half,
      // lift = L·max(0, sin(φ + off)) with off = +90° → lift while swinging forward (off = −90° walks backward).
      const ramp = Math.min(1, Math.max(0, (t - p.settle) / 0.5)), ph = 2 * Math.PI * p.freq * (t - p.settle) + j.limb.phase;
      const v = j.swing ? p.amp * Math.sin(ph) : j.liftAmp * Math.max(0, Math.sin(ph + p.off));
      const q = j.q0 + ramp * j.gsign * v;
      return j.lower !== null ? Math.min(Math.max(q, j.lower), j.upper) : q;
    }
    if (p.mode === 'gait' && j.osc) {
      a = p.amp * (j.isLeg ? 1 : 0.5) * Math.min(1, Math.max(0, (t - p.settle) / 0.5));
      if (j.lower !== null) center = Math.min(Math.max(j.q0, j.lower + p.amp + 0.05), j.upper - p.amp - 0.05);
      if (j.lower !== null && j.upper - j.lower < 2 * p.amp) center = (j.lower + j.upper) / 2;
      const ramp = Math.min(1, Math.max(0, (t - p.settle) / 0.5));
      return j.q0 + (center - j.q0) * ramp + j.gsign * (j.lift ? 0.7 : 1) * a * Math.sin(2 * Math.PI * p.freq * (t - p.settle) + j.limb.phase + j.chainIdx * p.off);
    }
    return j.q0;
  }

  function step() {
    const t = S.t;
    if (!S.fallen) S.nUp++;
    for (const c of J) {
      const j = c.j, P = rbs[j.pb], C = rbs[j.cb];
      const pq = P.rotation(); tq1.set(pq.x, pq.y, pq.z, pq.w);
      const axisNow = tv.copy(j.axisW).applyQuaternion(tq1);
      let q, qd;
      if (j.type === 'revolute') {
        const cq = C.rotation(); tq2.set(cq.x, cq.y, cq.z, cq.w);
        const rel = tq1.clone().invert().multiply(tq2);
        q = 2 * Math.atan2(rel.x * j.axisW.x + rel.y * j.axisW.y + rel.z * j.axisW.z, rel.w);
        if (q > Math.PI) q -= 2 * Math.PI; if (q < -Math.PI) q += 2 * Math.PI;
        q += j.q0;
        const wc = C.angvel(), wp = P.angvel();
        qd = (wc.x - wp.x) * axisNow.x + (wc.y - wp.y) * axisNow.y + (wc.z - wp.z) * axisNow.z;
      } else {
        const pp = P.translation(), cp = C.translation();
        tv2.copy(j.a1).applyQuaternion(tq1).add(pp);
        q = j.q0 + (cp.x - tv2.x) * axisNow.x + (cp.y - tv2.y) * axisNow.y + (cp.z - tv2.z) * axisNow.z;
        const vc = C.linvel(), vp = P.linvel();
        qd = (vc.x - vp.x) * axisNow.x + (vc.y - vp.y) * axisNow.y + (vc.z - vp.z) * axisNow.z;
      }
      const tgt = target(c, t), demand = c.kp * (tgt - q) - c.kd * qd;
      const sat = Math.abs(demand) > c.eff, tau = sat ? Math.sign(demand) * c.eff : demand;
      // Rapier treats stiffness=damping=0 as a rigid lock, so "motor off" uses tiny gains instead.
      if (sat !== c.sat || !sat) c.h.configureMotorPosition(tgt - j.q0, sat ? 1e-9 : c.kp, sat ? 1e-9 : c.kd);
      c.sat = sat;
      C.resetTorques(false); P.resetTorques(false);
      c.q = q; c.qd = qd; c.tau = tau; c.tgt = tgt;
      if (S.fallen) continue; // load statistics only while the robot is still upright
      const s = c.st;
      s.util += Math.abs(tau) / c.eff; if (sat) s.sat++; s.peak = Math.max(s.peak, Math.abs(demand));
      s.vpeak = Math.max(s.vpeak, Math.abs(qd));
      if (j.lower !== null && (q < j.lower + 0.03 || q > j.upper - 0.03)) s.lim++;
      s.E += Math.abs(tau * qd) * p.dt;
    }
    // Apply saturated torques after all resets (bodies are shared between joints).
    for (const c of J) if (c.sat) {
      const j = c.j, P = rbs[j.pb], C = rbs[j.cb], pq = P.rotation();
      tq1.set(pq.x, pq.y, pq.z, pq.w); tv.copy(j.axisW).applyQuaternion(tq1).multiplyScalar(c.tau);
      if (j.type === 'revolute') { C.addTorque(xyz(tv), true); P.addTorque(xyz(tv.negate()), true); }
      else { C.addForce(xyz(tv), true); P.addForce(xyz(tv.negate()), true); }
    }
    world.step();
    S.t += p.dt; S.n++;
    const rp = root.translation(), rq = root.rotation();
    tq1.set(rq.x, rq.y, rq.z, rq.w);
    const upz = tv.set(0, 0, 1).applyQuaternion(tq1).z, tilt = Math.acos(Math.min(1, Math.max(-1, upz))) * 180 / Math.PI;
    if (S.t > 0.2) S.maxTilt = Math.max(S.maxTilt, tilt);
    if (S.startX === null && S.t >= p.settle) { S.startX = rp.x; S.startY = rp.y; }
    if (!S.fallen && (upz < 0.766 || rp.z < 0.55 * z0)) { S.fallen = true; S.tFall = S.t; }
    if (S.n % 10 === 0) S.series.push([S.t, rp.x - x0, rp.y - y0, rp.z, tilt]);
    return S;
  }

  function result() {
    const rp = root.translation(), sx = S.startX ?? x0, sy = S.startY ?? y0;
    const dx = rp.x - sx, dy = rp.y - sy, dur = Math.max(0.01, S.t - p.settle);
    const joints = J.map(c => ({ name: c.j.name, effort: c.eff, kp: c.kp,
      util: c.st.util / (S.nUp || 1), sat: c.st.sat / (S.nUp || 1), peak: c.st.peak, vpeak: c.st.vpeak, vlim: c.j.velocity, lim: c.st.lim / (S.nUp || 1), E: c.st.E }));
    const E = joints.reduce((a, j) => a + j.E, 0);
    const cot = Math.abs(dx) > 0.02 ? E / (model.M * 9.81 * Math.abs(dx)) : null;
    const score = S.fallen ? -2 + S.tFall / p.T : dx - 0.5 * Math.abs(dy) - 0.003 * S.maxTilt;
    return { p, fallen: S.fallen, tFall: S.tFall, maxTilt: S.maxTilt, dx, dy, speed: dx / dur, E, cot, joints, series: S.series, score,
      zEnd: rp.z, z0 };
  }

  return { world, rbs, J, S, p, step, result, free: () => world.free() };
}

export async function runTrial(RAPIER, model, opt, { stopOnFall = true } = {}) {
  const sim = createSim(RAPIER, model, opt), N = Math.round(sim.p.T / sim.p.dt);
  let bad = false;
  for (let i = 0; i < N; i++) {
    sim.step();
    if (stopOnFall && sim.S.fallen && sim.S.t > sim.S.tFall + 0.3) break;
    if (i % 200 === 0) { const r = sim.rbs[model.rootIdx].translation(); if (!isFinite(r.x) || Math.abs(r.z) > 50) { bad = true; break; } }
  }
  const r = sim.result(); r.unstable = bad; sim.free();
  return r;
}

const xyz = v => ({ x: v.x, y: v.y, z: v.z });
const xyzw = q => ({ x: q.x, y: q.y, z: q.z, w: q.w });
