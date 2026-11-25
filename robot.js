// URDF parsing and model preparation (rest pose = all joints at 0).
import * as THREE from 'three';

const V = (x = 0, y = 0, z = 0) => new THREE.Vector3(x, y, z);
const nums = (s, d) => (s ? s.trim().split(/\s+/).map(Number) : d);
const rpyQuat = ([r, p, y]) => new THREE.Quaternion().setFromEuler(new THREE.Euler(r, p, y, 'ZYX'));
const Q_Y2Z = new THREE.Quaternion().setFromAxisAngle(V(1, 0, 0), Math.PI / 2); // three/rapier cylinders run along Y

function origin(el) {
  const o = el && el.querySelector(':scope > origin');
  return { p: V(...nums(o?.getAttribute('xyz'), [0, 0, 0])), q: rpyQuat(nums(o?.getAttribute('rpy'), [0, 0, 0])) };
}
function geom(el) {
  const g = el.querySelector('geometry');
  if (!g) return null;
  const b = g.querySelector('box'), c = g.querySelector('cylinder'), s = g.querySelector('sphere'), m = g.querySelector('mesh');
  const o = origin(el);
  if (b) return { type: 'box', size: nums(b.getAttribute('size')), ...o };
  if (c) return { type: 'cylinder', r: +c.getAttribute('radius'), l: +c.getAttribute('length'), ...o };
  if (s) return { type: 'sphere', r: +s.getAttribute('radius'), ...o };
  if (m) return { type: 'mesh', file: m.getAttribute('filename'), scale: nums(m.getAttribute('scale'), [1, 1, 1]), ...o };
  return null;
}

export function parseURDF(text) {
  const doc = new DOMParser().parseFromString(text, 'application/xml');
  if (doc.querySelector('parsererror')) throw new Error('XMLとして読み込めません（xacroの場合は事前に展開してください）');
  const robot = doc.querySelector('robot');
  if (!robot) throw new Error('<robot> 要素がありません。URDFファイルを指定してください');
  const links = {}, joints = [], mats = {};
  const rgba = el => { const c = el?.querySelector('color'); return c ? nums(c.getAttribute('rgba'), null) : null; };
  for (const m of robot.querySelectorAll(':scope > material')) mats[m.getAttribute('name')] = rgba(m);
  for (const l of robot.querySelectorAll(':scope > link')) {
    const ie = l.querySelector(':scope > inertial');
    let inertial = null;
    if (ie) {
      const i = ie.querySelector('inertia'), a = k => +(i?.getAttribute(k) || 0);
      inertial = { m: +(ie.querySelector('mass')?.getAttribute('value') || 0), ...origin(ie),
        I: [a('ixx'), a('ixy'), a('ixz'), a('ixy'), a('iyy'), a('iyz'), a('ixz'), a('iyz'), a('izz')] };
    }
    links[l.getAttribute('name')] = {
      name: l.getAttribute('name'), inertial,
      col: [...l.querySelectorAll(':scope > collision')].map(geom).filter(Boolean),
      vis: [...l.querySelectorAll(':scope > visual')].map(v => {
        const g = geom(v), m = v.querySelector('material');
        if (g) g.rgba = rgba(m) || mats[m?.getAttribute('name')] || null;
        return g;
      }).filter(Boolean),
    };
  }
  for (const j of robot.querySelectorAll(':scope > joint')) {
    const lim = j.querySelector('limit');
    joints.push({
      name: j.getAttribute('name'), type: j.getAttribute('type'),
      parent: j.querySelector('parent').getAttribute('link'), child: j.querySelector('child').getAttribute('link'),
      ...origin(j), axis: V(...nums(j.querySelector('axis')?.getAttribute('xyz'), [1, 0, 0])).normalize(),
      lower: lim ? +(lim.getAttribute('lower') ?? 0) : null, upper: lim ? +(lim.getAttribute('upper') ?? 0) : null,
      effort: lim ? +(lim.getAttribute('effort') || 0) : 0, velocity: lim ? +(lim.getAttribute('velocity') || 0) : 0,
    });
  }
  const childSet = new Set(joints.map(j => j.child));
  const roots = Object.keys(links).filter(n => !childSet.has(n));
  if (roots.length !== 1) throw new Error(`ルートリンクが ${roots.length} 個あります（1個である必要があります）`);
  return { name: robot.getAttribute('name') || 'robot', links, joints, root: roots[0] };
}

// Symmetric 3x3 eigen decomposition (Jacobi). m: row-major array(9). Returns {vals, vecs(columns)}.
function eigSym(m) {
  const a = [[m[0], m[1], m[2]], [m[3], m[4], m[5]], [m[6], m[7], m[8]]];
  const v = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
  for (let it = 0; it < 50; it++) {
    let p = 0, q = 1;
    if (Math.abs(a[0][2]) > Math.abs(a[p][q])) [p, q] = [0, 2];
    if (Math.abs(a[1][2]) > Math.abs(a[p][q])) [p, q] = [1, 2];
    if (Math.abs(a[p][q]) < 1e-12) break;
    const th = 0.5 * Math.atan2(2 * a[p][q], a[q][q] - a[p][p]), c = Math.cos(th), s = Math.sin(th);
    for (let k = 0; k < 3; k++) { const akp = a[k][p], akq = a[k][q]; a[k][p] = c * akp - s * akq; a[k][q] = s * akp + c * akq; }
    for (let k = 0; k < 3; k++) { const apk = a[p][k], aqk = a[q][k]; a[p][k] = c * apk - s * aqk; a[q][k] = s * apk + c * aqk; }
    for (let k = 0; k < 3; k++) { const vkp = v[k][p], vkq = v[k][q]; v[k][p] = c * vkp - s * vkq; v[k][q] = s * vkp + c * vkq; }
  }
  return { vals: [a[0][0], a[1][1], a[2][2]], vecs: v };
}

// Point extent along world z for a shape (used for spawn height and support polygon).
function shapePoints(s, bodyPos) {
  const c = s.p.clone().add(bodyPos), pts = [];
  const add = (x, y, z) => pts.push(V(x, y, z).applyQuaternion(s.q).add(c));
  if (s.type === 'box') { const [a, b, d] = s.half; for (const i of [-1, 1]) for (const j of [-1, 1]) for (const k of [-1, 1]) add(i * a, j * b, k * d); }
  else if (s.type === 'sphere') { pts.push(c.clone().setZ(c.z - s.r)); }
  else if (s.type === 'hull') { const a = s.few; for (let i = 0; i < a.length; i += 3) pts.push(V(a[i] + bodyPos.x, a[i + 1] + bodyPos.y, a[i + 2] + bodyPos.z)); }
  else { // cylinder / capsule along local Y
    const hh = s.hh, r = s.r;
    for (const y of [-hh, hh]) for (const t of [0, 1, 2, 3, 4, 5, 6, 7]) {
      const ang = t * Math.PI / 4; add(r * Math.cos(ang), y, r * Math.sin(ang));
    }
    if (s.type === 'capsule') for (const y of [-hh, hh]) { const e = V(0, y, 0).applyQuaternion(s.q).add(c); pts.push(e.setZ(e.z - r)); }
  }
  return pts;
}

export function buildModel(urdf, pose = {}, meshes = {}) {
  const warn = [];
  const { links, joints, root } = urdf;
  const childJoints = {}; for (const j of joints) (childJoints[j.parent] ||= []).push(j);
  // Rest-pose forward kinematics and body ownership (fixed joints merge into the parent body).
  const linkPose = { [root]: { p: V(), q: new THREE.Quaternion() } }, owner = { [root]: root }, order = [root];
  // Rest angle: 0, unless 0 lies outside the joint's limits (common for knees); then just inside the nearest limit.
  for (const j of joints) {
    const lim = (j.type === 'revolute' || j.type === 'prismatic') && j.lower !== null && j.upper > j.lower;
    j.qAuto = !lim || (j.lower <= 0 && 0 <= j.upper) ? 0 : 0 < j.lower ? j.lower + 0.1 * (j.upper - j.lower) : j.upper - 0.1 * (j.upper - j.lower);
    const o = pose[j.name];
    j.q0 = typeof o === 'number' && isFinite(o) ? (lim ? Math.min(Math.max(o, j.lower), j.upper) : o) : j.qAuto;
  }
  const walk = n => {
    for (const j of childJoints[n] || []) {
      const P = linkPose[n], jq = P.q.clone().multiply(j.q);
      const p = j.p.clone().applyQuaternion(P.q).add(P.p);
      if (j.type === 'prismatic') p.add(j.axis.clone().multiplyScalar(j.q0).applyQuaternion(jq));
      if (j.type === 'revolute' || j.type === 'continuous') jq.multiply(new THREE.Quaternion().setFromAxisAngle(j.axis, j.q0));
      linkPose[j.child] = { p, q: jq };
      owner[j.child] = j.type === 'fixed' ? owner[n] : j.child;
      order.push(j.child); walk(j.child);
    }
  };
  walk(root);
  const bodyNames = order.filter(n => owner[n] === n);
  const bidx = Object.fromEntries(bodyNames.map((n, i) => [n, i]));
  const bodies = bodyNames.map(n => ({ name: n, pos: linkPose[n].p.clone(), shapes: [], visuals: [], parts: [], links: [] }));
  for (const n of order) bodies[bidx[owner[n]]].links.push(n);

  let meshCount = 0, noInertia = [];
  for (const n of order) {
    const L = links[n], B = bodies[bidx[owner[n]]], LP = linkPose[n];
    const toBody = g => ({ p: g.p.clone().applyQuaternion(LP.q).add(LP.p).sub(B.pos), q: LP.q.clone().multiply(g.q) });
    for (const g of L.vis) if (g.type === 'mesh' && meshes[g.file]?.geo) B.visuals.push({ file: g.file, scale: g.scale, rgba: g.rgba, ...toBody(g) });
    let geoms = L.col.length ? L.col : L.vis;
    // Meshes that were downloaded become convex hulls (what MuJoCo does with mesh colliders too).
    geoms = geoms.map(g => (g.type === 'mesh' && meshes[g.file]?.pts ? { ...g, type: 'hull' } : g));
    const prim = geoms.filter(g => g.type !== 'mesh');
    if (prim.length < geoms.length) meshCount++;
    if (geoms.length && !prim.length) {
      // Mesh not available: capsules from the link origin to each child joint, or a sphere sized from the inertia.
      const kids = (childJoints[n] || []).map(j => j.p).filter(p => p.length() > 0.01);
      const I = L.inertial, rI = I && I.m > 0 ? Math.sqrt(2.5 * Math.max(I.I[0], I.I[4], I.I[8]) / I.m) : 0.03;
      geoms = kids.length ? kids.map(e => ({ type: 'seg', a: V(), b: e.clone(), p: V(), q: new THREE.Quaternion() }))
        : [{ type: 'sphere', r: Math.min(0.04, Math.max(0.004, rI)), p: I ? I.p.clone() : V(), q: new THREE.Quaternion() }];
    } else geoms = prim;
    for (const g of geoms) {
      if (g.type === 'hull') {
        const t = toBody(g), src = meshes[g.file], v = V();
        const tf = arr => { const o = new Float32Array(arr.length); for (let i = 0; i < arr.length; i += 3) {
          v.set(arr[i] * g.scale[0], arr[i + 1] * g.scale[1], arr[i + 2] * g.scale[2]).applyQuaternion(t.q).add(t.p); o[i] = v.x; o[i + 1] = v.y; o[i + 2] = v.z; } return o; };
        B.shapes.push({ type: 'hull', pts: tf(src.pts), few: tf(src.few), p: V(), q: new THREE.Quaternion() });
        continue;
      }
      if (g.type === 'seg') {
        const len = g.b.length(), r = Math.min(0.05, Math.max(0.012, len * 0.12));
        const mid = { p: g.b.clone().multiplyScalar(0.5), q: new THREE.Quaternion().setFromUnitVectors(V(0, 1, 0), g.b.clone().normalize()) };
        B.shapes.push({ type: 'capsule', r, hh: Math.max(0.001, len / 2 - r), approx: true, ...toBody(mid) });
      } else if (g.type === 'box') B.shapes.push({ type: 'box', half: g.size.map(x => x / 2), ...toBody(g) });
      else if (g.type === 'sphere') B.shapes.push({ type: 'sphere', r: g.r, ...toBody(g) });
      else if (g.type === 'cylinder') { const t = toBody(g); t.q.multiply(Q_Y2Z); B.shapes.push({ type: 'cylinder', r: g.r, hh: g.l / 2, ...t }); }
    }
    if (L.inertial && L.inertial.m > 0) {
      const R = new THREE.Matrix3().setFromMatrix4(new THREE.Matrix4().makeRotationFromQuaternion(LP.q.clone().multiply(L.inertial.q)));
      const Il = new THREE.Matrix3().set(...L.inertial.I);
      const Ib = R.clone().multiply(Il).multiply(R.clone().transpose());
      B.parts.push({ m: L.inertial.m, c: L.inertial.p.clone().applyQuaternion(LP.q).add(LP.p).sub(B.pos), I: Ib });
    } else if (n !== root || !L.inertial) noInertia.push(n);
  }
  const hulls = bodies.reduce((a, B) => a + B.shapes.filter(s => s.type === 'hull').length, 0);
  if (meshCount) warn.push({ lv: 'info', t: `${meshCount} 個のリンクのメッシュを取得できず、関節間をつなぐカプセルで近似しました（接触形状の精度が下がります）` });
  if (hulls) warn.push({ lv: 'info', t: `${hulls} 個のメッシュを凸包（へこみを埋めた形）として接触計算しています。くぼみに物がはまる接触は再現されません` });
  // Combine mass parts into each body.
  let M = 0; const comW = V();
  for (const B of bodies) {
    let m = 0; const c = V();
    for (const pt of B.parts) { m += pt.m; c.addScaledVector(pt.c, pt.m); }
    if (m <= 0) { m = 0.05; B.massless = true; } else c.divideScalar(m);
    const I = [0, 0, 0, 0, 0, 0, 0, 0, 0];
    for (const pt of B.parts) {
      const d = pt.c.clone().sub(c), e = pt.I.elements; // column-major, symmetric anyway
      const dd = d.lengthSq(), dv = [d.x, d.y, d.z];
      for (let r = 0; r < 3; r++) for (let k = 0; k < 3; k++) I[r * 3 + k] += e[k * 3 + r] + pt.m * ((r === k ? dd : 0) - dv[r] * dv[k]);
    }
    const { vals, vecs } = eigSym(I);
    const basis = vecs.map((_, k) => V(vecs[0][k], vecs[1][k], vecs[2][k]));
    if (basis[0].clone().cross(basis[1]).dot(basis[2]) < 0) basis[2].negate();
    const frame = new THREE.Quaternion().setFromRotationMatrix(new THREE.Matrix4().makeBasis(...basis));
    const floor = Math.max(1e-5, m * 1e-4);
    Object.assign(B, { m, com: c, inertia: vals.map(v => Math.max(v, floor)), iframe: frame });
    M += m; comW.addScaledVector(c.clone().add(B.pos), m);
  }
  comW.divideScalar(M);
  const massless = bodies.filter(b => b.massless).map(b => b.name);
  if (massless.length) warn.push({ lv: 'warn', t: `質量（inertial）が無いリンク: ${massless.slice(0, 6).join(', ')}${massless.length > 6 ? ' ほか' : ''}。0.05kg を仮定しました。実機の質量を記入すると精度が上がります` });

  // Spawn height: lowest point touches z = 0.
  let minZ = Infinity;
  for (const B of bodies) for (const s of B.shapes) for (const p of shapePoints(s, B.pos)) minZ = Math.min(minZ, p.z);
  if (!isFinite(minZ)) throw new Error('接触形状が1つもありません');
  const lift = -minZ + 0.003;
  for (const B of bodies) B.pos.z += lift;
  comW.z += lift;
  const height = Math.max(...bodies.flatMap(B => B.shapes.flatMap(s => shapePoints(s, B.pos).map(p => p.z))));

  // Movable joints.
  const mj = [];
  for (const j of joints) {
    if (j.type === 'fixed' || j.type === 'floating' || j.type === 'planar') {
      if (j.type === 'floating' || j.type === 'planar') warn.push({ lv: 'warn', t: `関節 ${j.name} (${j.type}) は未対応のため固定扱いにしました` });
      continue;
    }
    const pb = bidx[owner[j.parent]], cb = bidx[j.child];
    const axisW = j.axis.clone().applyQuaternion(linkPose[j.child].q).normalize();
    const anchorW = bodies[cb].pos.clone();
    const limited = j.type !== 'continuous' && j.lower !== null && j.upper > j.lower;
    if (j.type === 'revolute' && !limited) warn.push({ lv: 'warn', t: `関節 ${j.name} に可動範囲（limit）がありません` });
    mj.push({ name: j.name, type: j.type === 'prismatic' ? 'prismatic' : 'revolute', pb, cb, axisW, anchorW,
      a1: anchorW.clone().sub(bodies[pb].pos), lower: limited ? j.lower : null, upper: limited ? j.upper : null,
      effort: j.effort, velocity: j.velocity, effortMissing: !(j.effort > 0), q0: j.q0, qAuto: j.qAuto, axisLocal: j.axis.clone() });
  }
  const noEff = mj.filter(j => j.effortMissing);
  if (noEff.length) warn.push({ lv: 'warn', t: `トルク上限（effort）が無い関節が ${noEff.length} 個あります。仮に 30 N·m としました` });
  for (const j of noEff) j.effort = 30;
  // Reflected rotor inertia (armature), estimated from the torque rating. Real geared actuators always have it,
  // and without it tiny intermediate links (hip/ankle yokes) make the contact+joint solver drift.
  for (const j of mj) {
    j.armature = 0.0003 * j.effort;
    const b = bodies[j.cb]; b.inertia = b.inertia.map(v => v + j.armature);
  }
  warn.push({ lv: 'info', t: 'URDF にはモーターの反射慣性（アーマチュア）が無いため、トルク上限から推定した値（0.0003×上限 kg·m²）を各関節に加えています' });

  // Subtrees, limbs and gains.
  const kids = bodies.map(() => []); for (const j of mj) kids[j.pb].push(j);
  const subtree = b => [b, ...kids[b].flatMap(j => subtree(j.cb))];
  const rootIdx = bidx[root];
  const limbs = kids[rootIdx].map(j0 => {
    const bs = subtree(j0.cb), jset = mj.filter(j => bs.includes(j.cb));
    const low = Math.min(...bs.flatMap(b => bodies[b].shapes.flatMap(s => shapePoints(s, bodies[b].pos).map(p => p.z))));
    return { joints: jset, bodies: bs, start: j0.anchorW, isLeg: low < 0.02 + 0.03 * height, low };
  });
  const legs = limbs.filter(l => l.isLeg);
  // Phases alternate along each side and between sides: biped alternation, quadruped trot, hexapod tripod.
  for (const sgn of [1, -1]) {
    const side = legs.filter(l => (Math.sign(l.start.y - comW.y) || 1) === sgn).sort((a, b) => b.start.x - a.start.x);
    side.forEach((l, k) => { l.side = sgn; l.phase = ((k + (sgn > 0 ? 0 : 1)) % 2) * Math.PI; });
  }
  for (const l of limbs.filter(l => !l.isLeg)) {
    const sy = Math.sign(l.start.y - comW.y) || 1, sameLeg = legs.find(g => g.side === sy);
    l.phase = (sameLeg ? sameLeg.phase : 0) + Math.PI;
  }
  for (const l of limbs) {
    // Foot = lowest point of the limb. A leg that reaches out sideways more than down is "sprawled" (crab / insect):
    // it swings with its vertical-axis joint and lifts with the joints whose axis lies along the leg's horizontal span.
    let foot = null;
    for (const b of l.bodies) for (const s of bodies[b].shapes) for (const p of shapePoints(s, bodies[b].pos)) if (!foot || p.z < foot.z) foot = p;
    foot ||= l.start.clone();
    const reach = V(foot.x - l.start.x, foot.y - l.start.y, 0), drop = l.start.z - foot.z;
    l.sprawled = reach.length() > 0.8 * Math.max(drop, 0.001);
    let k = 0;
    for (const j of l.joints) {
      j.limb = l; j.isLeg = l.isLeg; j.gsign = 1; j.osc = false; j.lift = false; j.swing = false;
      const rev = j.type === 'revolute', ax = j.axisW;
      j.pitch = rev && Math.abs(ax.y) > 0.7;
      j.roll = rev && Math.abs(ax.x) > 0.7;
      // Foot velocity per unit joint speed (column of the foot Jacobian): axis × (foot − joint).
      j.footVel = rev ? ax.clone().cross(foot.clone().sub(j.anchorW)) : ax.clone();
      if (!l.sprawled && j.pitch) { j.osc = true; j.chainIdx = k++; }
    }
    if (l.sprawled && l.isLeg) {
      // Sprawled (crab / insect) leg: one joint swings the foot fore–aft, another lifts it during the forward swing.
      // Chosen from the Jacobian rather than axis names, so any leg layout or joint order works.
      const cand = l.joints.filter(j => j.type === 'revolute');
      const sw = cand.reduce((a, j) => (!a || Math.abs(j.footVel.x) > Math.abs(a.footVel.x) ? j : a), null);
      const li = cand.filter(j => j !== sw).reduce((a, j) => (!a || Math.abs(j.footVel.z) > Math.abs(a.footVel.z) ? j : a), null);
      const liftH = Math.max(0.008, 0.25 * (l.start.z - foot.z));
      if (sw && Math.abs(sw.footVel.x) > 1e-4) Object.assign(sw, { osc: true, swing: true, gsign: Math.sign(sw.footVel.x) });
      if (li && Math.abs(li.footVel.z) > 1e-4) Object.assign(li, { osc: true, lift: true, gsign: Math.sign(li.footVel.z), liftAmp: Math.min(1, liftH / Math.abs(li.footVel.z)) });
    }
  }
  const nLegs = Math.max(1, legs.length);
  for (const j of mj) {
    const bs = subtree(j.cb);
    let msub = 0, Ieff = 1e-4, Lmax = 0.02;
    for (const b of bs) {
      const B = bodies[b], r = B.com.clone().add(B.pos).sub(j.anchorW);
      msub += B.m; Ieff += B.m * r.clone().cross(j.axisW).lengthSq() + Math.max(...B.inertia);
      for (const s of B.shapes) for (const p of shapePoints(s, B.pos)) Lmax = Math.max(Lmax, p.distanceTo(j.anchorW));
    }
    const load = (j.isLeg ? M / nLegs : 0) + msub;
    Object.assign(j, { msub, Ieff, L: Lmax, load });
    // Stiff enough to carry its share of the weight, but never so stiff that a 0.25 rad error already exceeds the limit.
    j.kp0 = Math.max(1, Math.min(load * 9.81 * Lmax / 0.06, 4 * (j.effort || 30)));
    j.Iload = Ieff + (j.isLeg ? (M / nLegs) * Lmax * Lmax * 0.5 : 0);
  }
  if (!legs.length) warn.push({ lv: 'warn', t: '接地している脚が見つかりません。歩行テストは関節を振るだけになります' });

  // Support polygon at rest.
  const pts = [];
  for (const B of bodies) for (const s of B.shapes) for (const p of shapePoints(s, B.pos)) if (p.z < 0.012) pts.push([p.x, p.y]);
  const hull = convexHull(pts);
  const margin = polyMargin(hull, [comW.x, comW.y]);
  return { name: urdf.name, bodies, joints: mj, limbs, legs, rootIdx, M, com: comW, height, hull, margin, warn, nLinks: Object.keys(links).length };
}

function convexHull(P) {
  const p = [...P].sort((a, b) => a[0] - b[0] || a[1] - b[1]);
  if (p.length < 3) return p;
  const cr = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  const lo = [], up = [];
  for (const q of p) { while (lo.length >= 2 && cr(lo[lo.length - 2], lo[lo.length - 1], q) <= 0) lo.pop(); lo.push(q); }
  for (const q of p.reverse()) { while (up.length >= 2 && cr(up[up.length - 2], up[up.length - 1], q) <= 0) up.pop(); up.push(q); }
  return lo.slice(0, -1).concat(up.slice(0, -1));
}
// Signed distance from point to polygon boundary: positive inside.
function polyMargin(h, c) {
  if (h.length < 3) return -Infinity;
  let d = Infinity, inside = true;
  for (let i = 0; i < h.length; i++) {
    const a = h[i], b = h[(i + 1) % h.length], ex = b[0] - a[0], ey = b[1] - a[1], len = Math.hypot(ex, ey) || 1;
    const s = (ex * (c[1] - a[1]) - ey * (c[0] - a[0])) / len;
    if (s < 0) inside = false;
    d = Math.min(d, Math.abs(s));
  }
  return inside ? d : -d;
}

export function toRawURL(u) {
  u = u.trim();
  const m = u.match(/^https?:\/\/github\.com\/([^/]+)\/([^/]+)\/(?:blob|raw)\/(.+)$/);
  return m ? `https://raw.githubusercontent.com/${m[1]}/${m[2]}/${m[3]}` : u;
}
