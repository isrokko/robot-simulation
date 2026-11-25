// Diagnosis pipeline: stand test -> gait search -> sensitivity runs -> rule-based improvement suggestions.
import { runTrial } from './sim.js';

export const GRID = { freq: [0.8, 1.2, 1.8, 2.5], amp: [0.15, 0.3, 0.45], off: [Math.PI / 2, -Math.PI / 2] };

export async function diagnose(RAPIER, model, base, onProgress) {
  const tick = () => new Promise(r => setTimeout(r, 0));
  const total = 1 + GRID.freq.length * GRID.amp.length * GRID.off.length + 4;
  let done = 0; const prog = l => onProgress?.(++done / total, l);
  const stand = await runTrial(RAPIER, model, { ...base, mode: 'stand', T: 3 }, { stopOnFall: false });
  prog('立位テスト'); await tick();
  const grid = [];
  for (const freq of GRID.freq) for (const amp of GRID.amp) for (const off of GRID.off) {
    const r = await runTrial(RAPIER, model, { ...base, mode: 'gait', freq, amp, off, T: 4 });
    grid.push(r); prog(`歩容探索 f=${freq}Hz A=${amp}rad`); await tick();
  }
  const best = grid.reduce((a, b) => (b.score > a.score ? b : a));
  const bp = { ...base, mode: 'gait', freq: best.p.freq, amp: best.p.amp, off: best.p.off, T: 4 };
  const sens = {};
  for (const [k, o] of [['lowFriction', { friction: 0.4 }], ['weakMotor', { effortScale: 0.6 }], ['strongMotor', { effortScale: 1.5 }], ['highGain', { gain: base.gain * 2 }]]) {
    sens[k] = await runTrial(RAPIER, model, { ...bp, ...o }); prog('感度分析'); await tick();
  }
  return { stand, grid, best, sens, suggestions: suggest(model, stand, grid, best, sens) };
}

const f1 = x => x.toFixed(1), f2 = x => x.toFixed(2), pct = x => `${Math.round(x * 100)}%`;
const deg = r => Math.round(r * 180 / Math.PI);

export function suggest(m, stand, grid, best, sens) {
  const S = [], add = (lv, title, body) => S.push({ lv, title, body });
  // 1. Static geometry
  const comH = m.com.z, tipDeg = m.margin > 0 ? Math.atan2(m.margin, comH) * 180 / Math.PI : 0;
  if (m.margin <= 0) add('high', '静止姿勢で重心が支持多角形の外にあります',
    `重心の真下が足裏の接地範囲から ${f2(-m.margin)} m はみ出しています。足を重心の下へ移すか、重い部品の位置を接地範囲の中心へ寄せてください。`);
  else if (tipDeg < 8) add('mid', `転倒までの余裕角が ${f1(tipDeg)}° しかありません`,
    `重心高 ${f2(comH)} m に対し、支持多角形の縁まで ${f2(m.margin)} m です。スタンスを広げる、足裏を大きくする、バッテリーなど重い部品を低い位置に置くと余裕が増えます。`);
  else add('good', `静的安定性は十分です（余裕角 ${f1(tipDeg)}°）`, `重心高 ${f2(comH)} m、支持多角形の縁まで ${f2(m.margin)} m。`);

  // 2. Stand test
  const satStand = stand.joints.filter(j => j.sat > 0.05).sort((a, b) => b.sat - a.sat);
  if (stand.fallen || stand.unstable) add('high', `立位を保てず ${f1(stand.tFall ?? 0)} 秒で転倒しました`,
    satStand.length ? `トルク上限に張り付いている関節があります: ${satStand.slice(0, 4).map(j => `${j.name}（飽和 ${pct(j.sat)}）`).join('、')}。下の関節別の提案を確認してください。`
      : 'トルクは足りていますが姿勢を保てていません。静的安定性、または初期姿勢（関節角0、または可動範囲内の最も近い角度）が立ち姿として妥当かを確認してください。');
  for (const j of satStand.slice(0, 5)) add('high', `${j.name}: 立っているだけでトルク不足`,
    `必要ピーク ${f1(j.peak)} N·m に対し上限 ${f1(j.effort)} N·m（飽和 ${pct(j.sat)}）。減速比を上げるか、上限 ${f1(j.peak * 1.3)} N·m 以上のアクチュエータを推奨します。`);
  // Inverted-pendulum check: the joints nearest the ground must together be stiffer than M·g·h.
  const kpOf = Object.fromEntries(stand.joints.map(j => [j.name, j]));
  for (const [cls, dir] of [['pitch', '前後'], ['roll', '左右']]) {
    const low = m.legs.map(l => l.joints.filter(j => j[cls]).sort((a, b) => a.anchorW.z - b.anchorW.z)[0]).filter(Boolean);
    if (!low.length || m.legs.length > 2) continue; // 3+ legs are statically supported
    const sum = low.reduce((a, j) => a + kpOf[j.name].kp, 0), need = m.M * 9.81 * comH;
    if (sum < 1.2 * need) add(sum < 0.9 * need ? 'high' : 'mid', `${low.map(j => j.name).join('・')} の${dir}方向の保持力が倒立振子の限界以下です`,
      `体を${dir}に支える最下部の関節の剛性の合計 ${f1(sum)} N·m/rad が、質量×重力×重心高 ${f1(need)} N·m/rad を下回り、少し傾くと戻れません。トルク上限が小さくゲインを上げられないことが原因です。各関節のトルク上限を ${f1(1.3 * need / (4 * low.length))} N·m 以上にするか、重心を下げてください。`);
  }
  if (!stand.fallen && stand.maxTilt > 10) add('mid', `立っているだけで胴体が ${f1(stand.maxTilt)}° 傾きます`,
    '初期姿勢（関節角0、または可動範囲内の最も近い角度）で重心が足の中心からずれているか、足首・股関節の保持トルクが足りていません。重心を足裏の中心の真上に寄せる、足裏を前後に長くすることを検討してください。');
  const maxU = stand.joints.reduce((a, j) => (j.util > a.util ? j : a), stand.joints[0] || { util: 0 });
  if (!stand.fallen && maxU && maxU.util > 0.6) add('mid', `${maxU.name} は立位だけで上限の ${pct(maxU.util)} を使っています`,
    '歩行や外乱に使える余裕が少ないです。その関節より先の質量を軽くするか、トルクに余裕のあるモーターを選んでください。');

  // 3. Gait
  if (!m.legs.length) { add('info', '脚が見つからないため歩行評価はしていません', '接地しているリンクを持つ肢がありません。'); return S; }
  const ok = grid.filter(r => !r.fallen && !r.unstable);
  if (!ok.length) {
    add('high', `試した ${grid.length} 通りの歩き方はすべて転倒しました`,
      m.legs.length <= 2 ? '二足は関節を周期的に動かすだけの開ループ歩行ではほぼ必ず倒れます。重心やZMPを見て足の着地位置を変えるバランス制御が必要です。まず立位の安定から改善してください。'
        : '周波数を下げる、振幅を小さくする、静的安定性（スタンス幅）を上げると転倒しにくくなります。');
  } else {
    add('good', `最良の歩き方: ${best.p.freq} Hz・振幅 ${best.p.amp} rad・関節位相 ${best.p.off > 0 ? '+' : '−'}90°`,
      `${f2(best.speed)} m/s で前進（4秒で ${f2(best.dx)} m、横ずれ ${f2(best.dy)} m、最大傾き ${f1(best.maxTilt)}°）。${grid.length} 通り中 ${ok.length} 通りが転倒せず歩けました。`);
    if (best.dx < 0.05) add('mid', 'ほとんど前に進めていません', '足が滑っているか、前後の脚の位相が合っていません。足先の摩擦、膝の向き、脚の振り幅を見直してください。');
    if (Math.abs(best.dy) > 0.35 * Math.abs(best.dx) && Math.abs(best.dx) > 0.05) add('mid', '直進できず横に流れます',
      `前進 ${f2(best.dx)} m に対し横ずれ ${f2(best.dy)} m。左右の質量配置やリンク長の非対称を確認してください。`);
    if (best.maxTilt > 25) add('mid', `歩行中の胴体の傾きが最大 ${f1(best.maxTilt)}° と大きいです`, '胴体の揺れが大きいとセンサや積載物に悪影響があります。周波数を下げる、脚の付け根の間隔を広げると減ります。');
    if (best.cot !== null && best.cot > 2) add('mid', `移動効率が悪い（輸送コスト ${f1(best.cot)}）`,
      '同じ距離を進むのに多くのエネルギーを使っています。脚先の軽量化、無駄な関節の振りを減らすことが有効です。');
  }
  const legBodies = m.legs.flatMap(l => l.bodies), tips = legBodies.filter(b => !m.joints.some(j => j.pb === b));
  const tipMass = tips.reduce((a, b) => a + m.bodies[b].m, 0) / m.M;
  if (tipMass > 0.12) add('mid', `脚先が重いです（全質量の ${pct(tipMass)}）`, '脚の先端ほど慣性が効き、振り出しに大きなトルクと電力が要ります。モーターを付け根側へ移し、ベルトやリンクで先端を駆動する構造を検討してください。');
  const gj = best.joints;
  for (const j of gj.filter(j => j.sat > 0.1).sort((a, b) => b.sat - a.sat).slice(0, 4))
    add('high', `${j.name}: 歩行中にトルク上限へ ${pct(j.sat)} の時間張り付きます`, `必要ピーク ${f1(j.peak)} N·m / 上限 ${f1(j.effort)} N·m。上限 ${f1(j.peak * 1.2)} N·m 程度あると狙った動きを再現できます。`);
  for (const j of gj.filter(j => j.vlim > 0 && j.vpeak > j.vlim).slice(0, 4))
    add('mid', `${j.name}: 速度上限を超えています`, `最大 ${f1(j.vpeak)} rad/s に対し上限 ${f1(j.vlim)} rad/s。減速比を下げるか、より高速なモーターが必要です。`);
  for (const j of gj.filter(j => j.lim > 0.15).slice(0, 4))
    add('low', `${j.name}: 可動域の端に ${pct(j.lim)} の時間当たっています`, '機械的なストッパーへの衝突は破損の原因です。可動域を広げるか、この関節の振り幅を小さくしてください。');

  // 4. Sensitivity (ratios are meaningless when the best gait barely moves)
  if (ok.length && best.dx >= 0.05) {
    const rel = r => (r.fallen ? -1 : r.dx / Math.max(0.01, best.dx));
    const lf = sens.lowFriction, wm = sens.weakMotor, sm = sens.strongMotor, hg = sens.highGain;
    if (lf.fallen || rel(lf) < 0.6) add('mid', '滑りやすい床（摩擦0.4）に弱いです',
      `前進距離が ${lf.fallen ? '転倒' : pct(rel(lf))} に低下。足裏にゴムなど高摩擦の素材を付ける、接地面積を広げることを検討してください。`);
    else add('good', '滑りやすい床でも性能を保てます', `摩擦0.4で前進距離 ${pct(rel(lf))}。`);
    if (!wm.fallen && rel(wm) > 0.85) add('low', 'モーターを小型化できる可能性があります',
      `トルク上限を60%にしても前進距離 ${pct(rel(wm))} を維持しました。小さいモーターにすれば軽量化・低コスト化につながります（外乱への余裕は要確認）。`);
    if (!sm.fallen && rel(sm) > 1.2) add('mid', 'アクチュエータ強化の効果が大きいです', `トルク上限を1.5倍にすると前進距離が ${pct(rel(sm))} に向上しました。`);
    if (!hg.fallen && rel(hg) > 1.2) add('mid', '制御ゲインを上げると改善します', `PDゲイン2倍で前進距離 ${pct(rel(hg))}。関節の追従性が不足しています。`);
    else if (hg.fallen) add('low', '制御ゲインを上げると不安定になります', '今のゲインが上限付近です。ゲインより機構側（質量・トルク）の改善が効果的です。');
  }
  for (const w of m.warn) add(w.lv === 'warn' ? 'low' : 'info', 'モデルデータ: ' + (w.lv === 'warn' ? '要確認' : '近似'), w.t);
  const rank = { high: 0, mid: 1, low: 2, good: 3, info: 4 };
  return S.sort((a, b) => rank[a.lv] - rank[b.lv]);
}
