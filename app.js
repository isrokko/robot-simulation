import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';
import RAPIER from 'https://cdn.jsdelivr.net/npm/@dimforge/rapier3d-compat@0.14.0/+esm';
import { parseURDF, buildModel, toRawURL } from './robot.js';
import { createSim } from './sim.js';
import { diagnose, GRID } from './analyze.js';
import { QUAD, BIPED } from './samples.js';

const $ = id => document.getElementById(id);
const esc = s => String(s).replace(/[&<>"']/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
await RAPIER.init();

// ---------- viewer ----------
const view = $('view'), renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
renderer.shadowMap.enabled = true; view.prepend(renderer.domElement);
const scene = new THREE.Scene(), camera = new THREE.PerspectiveCamera(40, 1, 0.01, 100);
camera.up.set(0, 0, 1); camera.position.set(1.6, -1.8, 1.0);
const controls = new OrbitControls(camera, renderer.domElement); controls.enableDamping = true;
scene.add(new THREE.HemisphereLight(0xffffff, 0x777777, 1.6));
const sun = new THREE.DirectionalLight(0xffffff, 2.2); sun.position.set(2, -3, 5); sun.castShadow = true;
Object.assign(sun.shadow.camera, { left: -3, right: 3, top: 3, bottom: -3 }); sun.shadow.mapSize.set(2048, 2048); scene.add(sun, sun.target);
const ground = new THREE.Mesh(new THREE.PlaneGeometry(60, 60), new THREE.ShadowMaterial({ opacity: 0.25 })); ground.receiveShadow = true; scene.add(ground);
const grid = new THREE.GridHelper(60, 120, 0x888888, 0xbbbbbb); grid.rotation.x = Math.PI / 2; grid.material.opacity = 0.35; grid.material.transparent = true; scene.add(grid);
new ResizeObserver(() => { const w = view.clientWidth, h = view.clientHeight; renderer.setSize(w, h, false); camera.aspect = w / h; camera.updateProjectionMatrix(); }).observe(view);

const matBody = new THREE.MeshStandardMaterial({ color: 0x2b2e33, roughness: 0.55, metalness: 0.3 });
const matLimb = new THREE.MeshStandardMaterial({ color: 0xc9ccd1, roughness: 0.4, metalness: 0.5 });
const matHot = new THREE.MeshStandardMaterial({ color: 0xff4f00, roughness: 0.4, emissive: 0x551a00 });
let model = null, groups = [], sim = null, playing = false, busy = false;

const colorMats = {};
const matFor = c => (colorMats[c.join()] ||= new THREE.MeshStandardMaterial({ color: new THREE.Color(c[0], c[1], c[2]), roughness: 0.5, metalness: 0.2 }));
function buildMeshes(m, keepCamera) {
  for (const g of groups) scene.remove(g);
  groups = m.bodies.map((B, i) => {
    const g = new THREE.Group();
    for (const v of B.visuals) { // real STL visuals when they were downloaded
      const mesh = new THREE.Mesh(meshes[v.file].geo, v.rgba ? matFor(v.rgba) : i === m.rootIdx ? matBody : matLimb);
      mesh.userData.base = mesh.material; mesh.position.copy(v.p); mesh.quaternion.copy(v.q); mesh.scale.set(...v.scale); mesh.castShadow = true; g.add(mesh);
    }
    if (!B.visuals.length) for (const s of B.shapes) {
      if (s.type === 'hull') continue;
      const geo = s.type === 'box' ? new THREE.BoxGeometry(...s.half.map(x => x * 2)) : s.type === 'sphere' ? new THREE.SphereGeometry(s.r, 24, 16)
        : s.type === 'cylinder' ? new THREE.CylinderGeometry(s.r, s.r, s.hh * 2, 24) : new THREE.CapsuleGeometry(s.r, s.hh * 2, 6, 16);
      const mesh = new THREE.Mesh(geo, i === m.rootIdx ? matBody : matLimb);
      mesh.userData.base = mesh.material; mesh.position.copy(s.p); mesh.quaternion.copy(s.q); mesh.castShadow = true; g.add(mesh);
    }
    g.position.copy(B.pos); scene.add(g); return g;
  });
  if (keepCamera) return;
  controls.target.set(m.com.x, m.com.y, m.com.z * 0.8);
  const span = Math.max(...m.bodies.map(B => Math.hypot(B.pos.x - m.com.x, B.pos.y - m.com.y)));
  const d = Math.max(0.3, m.height * 3, span * 2.5); camera.position.set(m.com.x + d * 0.8, m.com.y - d, m.height * 0.9 + 0.2);
}
function syncMeshes() {
  if (!sim) return;
  sim.rbs.forEach((rb, i) => { const t = rb.translation(), q = rb.rotation(); groups[i].position.set(t.x, t.y, t.z); groups[i].quaternion.set(q.x, q.y, q.z, q.w); });
  const hot = new Set(sim.J.filter(c => c.sat).map(c => c.j.cb));
  groups.forEach((g, i) => g.children.forEach(mesh => { mesh.material = hot.has(i) ? matHot : mesh.userData.base; }));
}

renderer.setAnimationLoop(() => {
  if (playing && sim) {
    const n = Math.round(1 / 60 / sim.p.dt);
    for (let i = 0; i < n; i++) sim.step();
    syncMeshes();
    const r = sim.rbs[model.rootIdx].translation(), last = sim.S.series.at(-1);
    controls.target.lerp(new THREE.Vector3(r.x, r.y, model.com.z * 0.8), 0.05);
    const sat = sim.J.filter(c => c.sat).length;
    $('hud').innerHTML = `<span>t <b>${sim.S.t.toFixed(1)}</b> s</span><span>前進 <b>${(last?.[1] ?? 0).toFixed(2)}</b> m</span><span>傾き <b>${(last?.[4] ?? 0).toFixed(0)}</b>°</span><span>トルク飽和 <b>${sat}</b>/${sim.J.length} 関節</span>${sim.S.fallen ? '<span><b>転倒</b></span>' : ''}`;
    if (sim.S.t > 12) playing = false;
  }
  controls.update(); renderer.render(scene, camera);
});

// ---------- params ----------
const P = () => ({ freq: +$('freq').value, amp: +$('amp').value, gain: +$('gain').value, friction: +$('friction').value });
for (const k of ['freq', 'amp', 'gain', 'friction']) {
  const u = { freq: ' Hz', amp: ' rad', gain: ' ×', friction: '' }[k], f = () => ($('v' + k).textContent = $(k).value + u);
  $(k).addEventListener('input', f); f();
}
function play(mode, extra = {}) {
  if (!model || busy) return;
  sim?.free(); sim = createSim(RAPIER, model, { ...P(), mode, T: 12, ...extra }); playing = true;
}
$('playStand').onclick = () => play('stand');
$('playGait').onclick = () => play('gait');

// ---------- loading ----------
function msg(t, err) { $('msg').textContent = t; $('msg').className = err ? 'err' : ''; }
let urdf = null, pose = {}, meshes = {};
// Mesh files are fetched relative to the URDF's URL (or by locating the package name in that URL for package://).
function meshURL(file, base) {
  if (!base || !file) return null;
  const pk = file.match(/^package:\/\/([^/]+)\/(.+)$/);
  if (pk) { const i = base.indexOf('/' + pk[1] + '/'); return i < 0 ? null : base.slice(0, i + pk[1].length + 2) + pk[2]; }
  if (/^[a-z]+:\/\//i.test(file) && !/^https?:/i.test(file)) return null;
  try { return new URL(file, base).href; } catch { return null; }
}
const DIRS = []; for (const x of [-1, 0, 1]) for (const y of [-1, 0, 1]) for (const z of [-1, 0, 1]) if (x || y || z) DIRS.push([x, y, z]);
async function fetchMeshes(u, base) {
  const files = [...new Set(Object.values(u.links).flatMap(L => [...L.col, ...L.vis]).filter(g => g.type === 'mesh').map(g => g.file))];
  const out = {}; let done = 0, ok = 0;
  if (!base || !files.length) return out;
  const loader = new STLLoader();
  await Promise.all(files.map(async f => {
    const url = meshURL(f, base);
    try {
      if (!url || !/\.stl$/i.test(url)) throw 0;
      const r = await fetch(url); if (!r.ok) throw 0;
      const geo = loader.parse(await r.arrayBuffer()); geo.computeVertexNormals();
      const a = geo.attributes.position.array, n = a.length / 3, stride = Math.max(1, Math.floor(n / 1500));
      const few = [];
      for (const d of DIRS) { let best = -Infinity, bi = 0; for (let i = 0; i < n; i++) { const v = a[3 * i] * d[0] + a[3 * i + 1] * d[1] + a[3 * i + 2] * d[2]; if (v > best) { best = v; bi = i; } } few.push(a[3 * bi], a[3 * bi + 1], a[3 * bi + 2]); }
      const pts = [...few]; for (let i = 0; i < n; i += stride) pts.push(a[3 * i], a[3 * i + 1], a[3 * i + 2]);
      out[f] = { geo, pts: new Float32Array(pts), few: new Float32Array(few) }; ok++;
    } catch {}
    msg(`メッシュを取得中… ${++done} / ${files.length}`);
  }));
  out._summary = `メッシュ ${ok}/${files.length} 個を取得`;
  return out;
}
async function load(text, label, base) {
  try {
    const u = parseURDF(text);
    meshes = await fetchMeshes(u, base);
    const m = buildModel(u, {}, meshes);
    urdf = u; pose = {}; model = m; sim?.free(); sim = null; playing = false; buildMeshes(m);
    msg(`${label}: ${m.name}（リンク ${m.nLinks}・可動関節 ${m.joints.length}・脚 ${m.legs.length} 本・${m.M.toFixed(m.M < 5 ? 2 : 1)} kg${meshes._summary ? '・' + meshes._summary : ''}）`);
    $('hud').textContent = '「立位を再生」「歩行を再生」で動かせます。オレンジ色の部位はトルク上限に達しています。';
    clearReport(); renderPoseUI(); poseStats();
    play('stand');
  } catch (e) { msg('読み込み失敗: ' + e.message, true); }
}
const clearReport = () => ($('report').innerHTML = '<h2>診断レポート</h2><p class="small">「総合診断と改善提案」を押すと結果が出ます。</p>');

// ---------- initial pose editor ----------
// Left/right partner by name (L_/R_, left/right, FL/FR …) and the sign that mirrors the angle across the x–z plane.
function swapLR(n) {
  for (const [a, b] of [['left', 'right'], ['Left', 'Right'], ['LEFT', 'RIGHT']]) {
    if (n.includes(a)) return n.replaceAll(a, b);
    if (n.includes(b)) return n.replaceAll(b, a);
  }
  const sw = { L: 'R', R: 'L', l: 'r', r: 'l' };
  return n.split(/([_\-.])/).map(t => (t.length === 1 && sw[t] ? sw[t] : /^[FRMHfrmh][LRlr]$/.test(t) ? t[0] + sw[t[1]] : t)).join('');
}
// Two-letter limb codes are ambiguous: RR is "rear right" (FR/RL style) or "right rear" (LF/RM style). Try both, keep the one that exists.
const swapLead = n => n.split(/([_\-.])/).map(t => (/^[LRlr][A-Za-z]$/.test(t) ? ({ L: 'R', R: 'L', l: 'r', r: 'l' })[t[0]] + t[1] : t)).join('');
function partner(j) {
  const o = [swapLR(j.name), swapLead(j.name)].filter(n => n !== j.name).map(n => model.joints.find(k => k.name === n)).find(Boolean);
  if (!o) return null;
  const a = j.axisW, refl = new THREE.Vector3(-a.x, a.y, -a.z); // reflected axial vector
  return { j: o, sign: Math.sign(o.axisW.dot(refl)) || 1 };
}
const range = j => (j.lower !== null ? [j.lower, j.upper] : j.type === 'prismatic' ? [-0.2, 0.2] : [-Math.PI, Math.PI]);
function renderPoseUI() {
  $('poseList').innerHTML = model.joints.map((j, i) => {
    const [lo, hi] = range(j);
    return `<label>${esc(j.name)} <span class="val" id="pv${i}">${j.q0.toFixed(2)}</span>
      <input type="range" data-i="${i}" min="${lo}" max="${hi}" step="0.01" value="${j.q0}"></label>`;
  }).join('');
}
let poseTimer = 0;
function rebuildPose(playAfter) {
  const m = buildModel(urdf, pose, meshes);
  model = m; sim?.free(); sim = null; playing = false; buildMeshes(m, true);
  model.joints.forEach((j, i) => { const s = $('poseList').querySelector(`[data-i="${i}"]`); if (s) { s.value = j.q0; $('pv' + i).textContent = j.q0.toFixed(2); } });
  poseStats(); clearReport();
  if (playAfter) play('stand');
}
function poseStats() {
  const m = model, tip = m.margin > 0 ? Math.atan2(m.margin, m.com.z) * 180 / Math.PI : null;
  $('poseStats').innerHTML = `重心高 <b>${m.com.z.toFixed(2)} m</b>・転倒余裕角 <b class="${tip === null || tip < 8 ? 'bad' : ''}">${tip === null ? '支持外' : tip.toFixed(1) + '°'}</b>`;
}
$('poseList').addEventListener('input', e => {
  const i = e.target.dataset.i; if (i === undefined || busy) return;
  const j = model.joints[+i], v = +e.target.value;
  pose[j.name] = v;
  const pr = $('mirror').checked && partner(j);
  if (pr) pose[pr.j.name] = v * pr.sign;
  $('pv' + i).textContent = v.toFixed(2);
  cancelAnimationFrame(poseTimer); poseTimer = requestAnimationFrame(() => rebuildPose(false));
});
$('poseList').addEventListener('change', () => !busy && rebuildPose(true));
$('poseReset').onclick = () => { if (!busy && urdf) { pose = {}; rebuildPose(true); } };
$('poseCopy').onclick = async () => {
  const txt = JSON.stringify(Object.fromEntries(model.joints.map(j => [j.name, +j.q0.toFixed(3)])));
  try { await navigator.clipboard.writeText(txt); msg('姿勢をクリップボードにコピーしました'); } catch { $('posePaste').value = txt; msg('下の欄に姿勢を書き出しました'); }
};
$('poseApply').onclick = () => {
  try { const o = JSON.parse($('posePaste').value); if (typeof o !== 'object' || !o) throw 0; pose = { ...pose, ...o }; rebuildPose(true); msg('姿勢を適用しました'); }
  catch { msg('姿勢は {"関節名": 角度(rad), …} の JSON で入力してください', true); }
};
$('sample').onchange = e => load(e.target.value === 'quad' ? QUAD : BIPED, 'サンプル');
$('file').onchange = async e => { const f = e.target.files[0]; if (f) load(await f.text(), f.name); };
$('loadUrl').onclick = async () => {
  const u = toRawURL($('url').value);
  if (!/^https?:\/\//.test(u)) return msg('http(s) の URL を入力してください', true);
  msg('取得中…');
  try { const r = await fetch(u); if (!r.ok) throw new Error('HTTP ' + r.status); await load(await r.text(), u.split('/').pop(), u); }
  catch (e) { msg('取得できません: ' + e.message + '（raw ファイルの URL か確認してください）', true); }
};

// ---------- diagnosis ----------
$('diag').onclick = async () => {
  if (!model || busy) return;
  busy = true; playing = false; document.querySelectorAll('button').forEach(b => (b.disabled = true));
  const t0 = performance.now();
  try {
    const R = await diagnose(RAPIER, model, P(), (f, l) => { $('prog').style.width = (f * 100).toFixed(0) + '%'; $('hud').textContent = `診断中: ${l}`; });
    render(R, (performance.now() - t0) / 1000);
    busy = false;
    if (model.legs.length) { $('freq').value = R.best.p.freq; $('amp').value = R.best.p.amp; ['freq', 'amp'].forEach(k => $(k).dispatchEvent(new Event('input'))); play('gait', { off: R.best.p.off }); }
  } catch (e) { msg('診断中にエラー: ' + e.message, true); console.error(e); }
  busy = false; document.querySelectorAll('button').forEach(b => (b.disabled = false));
};

const TAG = { high: '重要', mid: '改善', low: '検討', good: '良好', info: '情報' };
function render(R, secs) {
  const m = model, s = R.stand, b = R.best, tip = m.margin > 0 ? Math.atan2(m.margin, m.com.z) * 180 / Math.PI : 0;
  const okN = R.grid.filter(r => !r.fallen).length;
  const figs = [['総質量', m.M.toFixed(1) + ' kg'], ['重心高', m.com.z.toFixed(2) + ' m'], ['転倒余裕角', tip.toFixed(1) + '°'],
    ['立位', s.fallen ? '転倒' : '保持'], ['最良の前進速度', okN ? b.speed.toFixed(2) + ' m/s' : '—'], ['歩けた条件', `${okN} / ${R.grid.length}`]];
  const heat = () => {
    const cell = (f, a) => { const rs = R.grid.filter(r => r.p.freq === f && r.p.amp === a), r = rs.reduce((x, y) => (y.score > x.score ? y : x));
      const v = r.fallen ? null : r.dx, c = v === null ? '#7a7d83' : `hsl(${Math.max(0, Math.min(140, 20 + v * 160))} 70% 38%)`;
      return `<td style="background:${c}" title="${r === b ? '最良' : ''}">${v === null ? '転倒' : v.toFixed(2)}${r === b ? ' ★' : ''}</td>`; };
    return `<table class="heat"><tr><th>周波数 \\ 振幅</th>${GRID.amp.map(a => `<th>${a} rad</th>`).join('')}</tr>${GRID.freq.map(f => `<tr><th>${f} Hz</th>${GRID.amp.map(a => cell(f, a)).join('')}</tr>`).join('')}</table>
      <p class="small">セルの数値は4秒間の前進距離（m）。位相 ±90° のうち良い方。</p>`;
  };
  const gj = Object.fromEntries(b.joints.map(j => [j.name, j]));
  const bar = u => `<span class="bar"><i style="width:${Math.min(100, u * 100).toFixed(0)}%"></i></span>`;
  $('report').innerHTML = `<h2>診断レポート: ${esc(m.name)}</h2>
  <div class="figs">${figs.map(([k, v]) => `<div><span>${k}</span><b>${esc(v)}</b></div>`).join('')}</div>
  <h2>改善の提案</h2><ul class="sugg">${R.suggestions.map(x => `<li data-lv="${x.lv}" data-tag="${TAG[x.lv]}"><b>${esc(x.title)}</b><p>${esc(x.body)}</p></li>`).join('')}</ul>
  <div class="cols"><div><h2>関節ごとの負荷</h2><div class="scroll"><table><tr><th>関節</th><th>上限 N·m</th><th>立位 平均負荷</th><th>歩行 平均負荷</th><th>必要ピーク</th><th>飽和</th><th>可動域端</th></tr>
  ${s.joints.map(j => { const g = gj[j.name]; return `<tr><td>${esc(j.name)}</td><td>${j.effort.toFixed(0)}</td><td>${(j.util * 100).toFixed(0)}%${bar(j.util)}</td><td>${(g.util * 100).toFixed(0)}%${bar(g.util)}</td><td>${Math.max(j.peak, g.peak).toFixed(1)}</td><td>${(Math.max(j.sat, g.sat) * 100).toFixed(0)}%</td><td>${(g.lim * 100).toFixed(0)}%</td></tr>`; }).join('')}
  </table></div></div><div><h2>歩き方の探索結果</h2><div class="scroll">${heat()}</div><h2 style="margin-top:20px">最良条件での胴体の高さと前進</h2><div class="fig" id="chart"></div></div></div>
  <p class="small" style="margin-top:20px">${R.grid.length + 5} 回のシミュレーションを ${secs.toFixed(1)} 秒で実行。歩行は関節を正弦波で動かす開ループ制御、接触は簡略形状、自己干渉は無視しています。提案は目安で、実機での確認が必要です。</p>`;
  const el = $('chart'), svg = document.createElementNS('http://www.w3.org/2000/svg', 'svg'); el.append(svg);
  const draw = () => {
    const W = el.clientWidth, H = el.clientHeight, ser = b.series, T = ser.at(-1)?.[0] || 1, pad = 34;
    const xs = t => pad + (t / T) * (W - pad - 10);
    const line = (k, lo, hi, col) => `<polyline fill="none" stroke="${col}" stroke-width="2" points="${ser.map(r => `${xs(r[0]).toFixed(1)},${(H - 24 - ((r[k] - lo) / (hi - lo || 1)) * (H - 44)).toFixed(1)}`).join(' ')}"/>`;
    const zs = ser.map(r => r[3]), xsv = ser.map(r => r[1]);
    svg.innerHTML = `<line x1="${pad}" x2="${W - 10}" y1="${H - 24}" y2="${H - 24}" stroke="currentColor" opacity=".3"/>
      ${line(3, 0, Math.max(...zs) * 1.1, 'var(--ink)')}${line(1, Math.min(0, ...xsv), Math.max(0.01, ...xsv) * 1.1, '#ff4f00')}
      <text x="${pad}" y="14" font-size="13" fill="currentColor">黒: 胴体の高さ（最大 ${Math.max(...zs).toFixed(2)} m）　橙: 前進距離（最大 ${Math.max(...xsv).toFixed(2)} m）</text>
      <text x="${W - 10}" y="${H - 6}" font-size="13" text-anchor="end" fill="currentColor">${T.toFixed(1)} 秒</text>`;
  };
  const ro = new ResizeObserver(draw); ro.observe(el);
}

load(QUAD, 'サンプル');
