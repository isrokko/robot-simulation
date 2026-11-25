# ロボット歩行ラボ（Robot Gait Lab）

URDF 形式のロボット（動物型・人型・多脚）を読み込み、ブラウザの中で物理シミュレーションを行って
**立位・歩行を試験し、設計と制御の改善点を提案する**ツールです。インストール不要で、サーバーも使いません。

> English: A browser-only tool that loads a URDF robot (quadruped, humanoid, hexapod…), simulates standing
> and walking with the Rapier physics engine, and suggests design/control improvements (torque limits,
> stability margin, gait parameters, friction sensitivity). No install, no backend.

## できること

- **読み込み**：GitHub 上の URDF の URL を貼るだけ（`github.com/.../blob/...` のままで可）。同じリポジトリの STL メッシュも自動取得します。手元の `.urdf` ファイルも読めます。
- **初期姿勢の編集**：関節ごとのスライダー、左右対称の同時編集、JSON でのコピー／貼り付け。重心高と転倒余裕角をその場で表示します。
- **再生**：立位・歩行を 3D で再生。トルク上限に達した部位はオレンジ色に光ります。
- **総合診断**：立位テスト → 24 通りの歩き方の探索 → 感度分析（滑りやすい床・モーター強弱・制御ゲイン）を数秒で実行し、重要度つきの改善提案を出します。
  - 例：「RM_J1_joint は必要 1.0 N·m に対し上限 0.7 N·m」「転倒余裕角が 2.9° しかない」「足首の保持力が倒立振子の限界以下」

## 使い方

### 公開ページで使う

GitHub Pages で公開している場合は、そのページを開くだけです。

### 手元で動かす

ES モジュールを使うため、`index.html` をダブルクリックで開いても動きません。簡易サーバー経由で開いてください。

```bash
python -m http.server 8000
```

ブラウザで `http://localhost:8000` を開きます（インターネット接続が必要です：ライブラリを CDN から読み込みます）。

### 手順

1. サンプルを選ぶか、GitHub の URDF の URL を入れて「URL から読み込む」
2. 寝そべったり倒れたりする場合は「初期姿勢」で立ち姿勢を作る（多くのロボットは関節角 0 では立てません）
3. 「総合診断と改善提案」を押す

### 例：KingKongRobotics/jumper（六脚）

URL：`https://github.com/KingKongRobotics/jumper/blob/main/assets/jumper/urdf/jumper/urdf/jumper.urdf`

関節角 0 では脚が横に寝るので、リポジトリの `tasks/jumper/common/constants.py` にある `HOME` を初期姿勢に貼り付けます。

```json
{"LF_J0_joint":-0.5507,"LF_J1_joint":-1.0658,"LF_J2_joint":-0.4408,"LF_J3_joint":-1.374,"LF_J4_joint":0,"RF_J0_joint":0.5524,"RF_J1_joint":-1.066,"RF_J2_joint":0.4414,"RF_J3_joint":1.4173,"RF_J4_joint":0,"LM_J0_joint":0.0012,"LM_J1_joint":0.5356,"LM_J2_joint":-1.8271,"RM_J0_joint":-0.0012,"RM_J1_joint":-0.5376,"RM_J2_joint":1.8313,"LR_J0_joint":0.6313,"LR_J1_joint":0.5534,"LR_J2_joint":-1.8648,"RR_J0_joint":-0.6326,"RR_J1_joint":-0.5548,"RR_J2_joint":1.8678}
```

## 仕組み

| ファイル | 役割 |
|---|---|
| `index.html` | 画面 |
| `app.js` | 3D 表示、読み込み、姿勢エディタ、レポート |
| `robot.js` | URDF の解析、剛体モデル化、脚の判定、ゲイン設計 |
| `sim.js` | Rapier による物理シミュレーション（トルク上限の再現を含む） |
| `analyze.js` | 診断の流れと改善提案のルール |
| `samples.js` | 内蔵サンプル（四足・人型、オリジナル） |

- 関節は PD 制御。要求トルクが上限以内なら Rapier の陰的モーター、上限を超えたら上限トルクだけを直接加えて、モーターの頭打ちを再現します。
- URDF にないモーターの反射慣性（アーマチュア）は、トルク上限から推定して加えています。
- 歩行は開ループ（関節を周期的に動かすだけ）です。脚を下ろすタイプは関節のピッチ振動、横に広がった多脚は足先のヤコビアンから「振り出し」と「持ち上げ」の関節を選んだ三脚歩行を使います。

## 制限

- 結果は簡略化したシミュレーションによる**目安**です。実機の安全性や設計判断を保証しません。
- 歩行にバランス制御はありません（二足は基本的に転倒します）。
- 対応形式は URDF のみ。xacro（未展開）と MJCF は未対応です。
- メッシュは凸包として接触計算します。ロボット自身の部位どうしの衝突は無視します。

## ロボットデータのライセンスについて

このリポジトリには外部のロボットデータを含めていません。URL から読み込むデータは、それぞれの作者のライセンスに従います。

## ライセンス

[MIT License](LICENSE)。使用しているライブラリは [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) を参照してください。
