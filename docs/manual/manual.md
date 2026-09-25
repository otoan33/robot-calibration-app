---
marp: true
lang: ja
paginate: true
title: robot-calibration 使い方マニュアル
style: |
  section {
    font-family: "Noto Sans JP", "Yu Gothic UI", "Meiryo", sans-serif;
    font-size: 24px;
    line-height: 1.6;
    padding: 56px 44px 48px;
    color: #1d2b3a;
    background: #eef2f7;
  }
  h1 { color: #1f4e79; font-size: 46px; }
  h2 {
    color: #1f4e79;
    font-size: 30px;
    border-left: 8px solid #5898d4;
    padding-left: 14px;
    margin-bottom: 28px;
  }
  header { color: #5898d4; font-size: 16px; font-weight: bold; }
  strong { color: #d32f2f; }
  code { font-size: 0.85em; }
  ul { padding-left: 1.1em; }
  li + li { margin-top: 12px; }
  table { font-size: 19px; }
  th, td { background: #ffffff !important; }
  section.cover { justify-content: center; background: #ffffff; border-left: 24px solid #5898d4; }
  section.cover p { color: #52667a; }
---

<!-- _class: cover -->
<!-- _paginate: false -->

# robot-calibration<br>使い方マニュアル

計測 CSV から精度グラフを作り、キャリブレーションを行う社内ツール

2026-09 版

---

## このアプリでできること

| ページ | タブ | できること |
|---|---|---|
| 描画 | 姿勢精度 | 計測点ごとの位置誤差を Before/After で比べる |
| | 軌跡精度 | 直線軌跡の横ずれを比べる |
| | 単軸 | 1 軸の角度伝達誤差とその周波数成分を見る |
| 解析 | キネマ補正 | 機構パラメータと関節の伝達誤差を推定し、補正前後の精度を示す |
| | 軌跡キャリブ | 動かしながら測ったデータから、キネマ・伝達誤差・時刻ずれを推定する |
| | 関節補正 | 減速機の周期誤差（振幅・位相）を推定する |
| | ツール補正 | 工具オフセットを推定する |

スクリーンショットの **赤枠** が操作する場所です。

---

<!-- header: はじめに -->

## 1. アプリを開く

![bg right:64% contain](img/01_start.png)

- Docker でアプリを起動する
  `docker compose up -d`
- ブラウザで http://localhost:8080 を開くと、描画ページが表示される
- 機能は **タブ** で選ぶ

---

## 2. ページを切り替える

![bg right:64% contain](img/02_header.png)

- 画面上部の **「描画」「解析」** で切り替える
- 描画：CSV からグラフを作る
- 解析：補正パラメータを推定する

---

<!-- header: 描画ページ｜姿勢精度 -->

## 3. 系列の名前と色を決める

![bg right:64% contain](img/03_series.png)

- 1 枚のカードがグラフの 1 系列
- **名前** は凡例に、**色** は線の色に使われる
- 最初は Before（オレンジ）と After（青）

---

## 4. CSV を入れる

![bg right:64% contain](img/04_upload.png)

- 系列ごとに **＋** を押すか、欄へドラッグ&ドロップ
- 入れたファイルは下に表示され、× で取り消せる
- 列：`RobotX/Y/Z` と `MeasureX/Y/Z`

---

## 5. 系列を増やす・減らす

![bg right:64% contain](img/05_add_series.png)

- **系列を追加** でカードが増える
- 要らないカードは **ゴミ箱** で消す
- ファイルが入っていないカードが残っていると描画できない

---

## 6. グラフの種類を選ぶ

![bg right:64% contain](img/06_options.png)

- `norm_with_bar`：誤差の推移＋平均・最大の棒グラフ
- `norm`：誤差の推移のみ / `each_axis(_overlay)`：X・Y・Z 別
- **fitlocal**：計測器の設置位置のずれを除いて比べる

---

## 7. 描画する

![bg right:64% contain](img/07_draw.png)

- **描画** を押すと下にグラフが出る
- 設定を変えたら、もう一度「描画」を押す

---

## 8. グラフを保存する

![bg right:64% contain](img/08_save_png.png)

- **PNG を保存** で画像をダウンロードする
- 資料にはこの PNG を貼る

---

<!-- header: 描画ページ｜軌跡精度 -->

## 9. 軌跡の CSV を入れる

![bg right:64% contain](img/09_path_upload.png)

- **軌跡精度** タブを開く
- 系列ごとに `*_BT.csv`（FARO）と `*_FM.csv`（ロボット）を **1 組** 入れる
- 2 つは末尾以外が同じ名前にしておく

---

## 10. 軌跡精度を描画する

![bg right:64% contain](img/10_path_draw.png)

- **描画** を押す
- 左：進行方向に直交する 2 方向のずれ
- 右：ずれの軌跡と最大ずれの円（数値は半径）

---

<!-- header: 描画ページ｜単軸 -->

## 11. 軸を選んで CSV を入れる

![bg right:64% contain](img/11_single_settings.png)

- **軸** を選ぶと **減速比** が自動で入る（変更も可）
- 系列ごとに `*_FM.csv` と `*_BT.csv` の組を入れる
- 区間を分けて測った場合は、組を複数入れる

---

## 12. 角度伝達誤差を描画する

![bg right:64% contain](img/12_single_draw.png)

- 上：関節角ごとの周期誤差
- 下：周波数成分。赤の破線は減速比から決まる周波数
- 破線の位置のピークが小さいほど良い

---

<!-- header: 解析ページ -->

## 13. 解析ページの注意

![bg right:64% contain](img/13_analysis_note.png)

- 学習済みモデルは **1 つだけ** 保持される
- 別のタブで学習すると、前の結果は消える
- 残したい結果は先に **パラメータを保存** する

---

<!-- header: 解析ページ｜キネマ補正 -->

## 14. 機種とモードを選ぶ

![bg right:64% contain](img/14_kinema_mode.png)

- **機種**：R6A〜R6E
- **mode**：actual（たわみ補正あり）/ ideal（公称 DH）
- **calibration_mode**：推定するパラメータの範囲

---

## 15. 同定パターンを選ぶ

![bg right:64% contain](img/15_kinema_pattern.png)

- **同定パターン** で推定する対象を選ぶ
- 伝達誤差：関節ごとの周期的な角度誤差（周期は機種で固定、振幅・位相を推定）
- 「同時」は一度に、「2 段階」はキネマの後に伝達誤差を推定。ideal では「キネマのみ」

---

## 16. 荷重と工具を入力する

![bg right:64% contain](img/16_kinema_payload.png)

- 計測時の **可搬質量・重心・重力方向** を入れる
- **工具オフセット** は 1 行に 1 工具（x, y, z）
- 1 行目が CSV の ToolID=1 に対応

---

## 17. CSV を入れて学習する

![bg right:64% contain](img/17_kinema_train.png)

- 同定パターンは「キネマのみ」のまま、FARO の計測 CSV を入れる（複数可）
- **学習** を押す。ボタンが回っている間は待つ
- 学習には数十秒〜数分かかる

---

## 18. 補正の効果を確認する

![bg right:64% contain](img/18_kinema_result.png)

- Before：計測 − 指令位置、After：計測 − 補正モデル
- 棒グラフで平均・最大誤差を比べる
- **R²** は 1 に近いほどモデルが計測に合っている

---

## 19. パラメータを保存する

![bg right:64% contain](img/19_save_param.png)

- **パラメータを保存** で JSON をダウンロードする
- ファイル名は `kinema_機種.json`
- 次ページの「保存済みパラメータで評価」で使える

---

## 20. 保存済みパラメータで評価する

![bg right:64% contain](img/20_load_param.png)

- **保存済みパラメータで評価する** を開き、JSON を入れる
- **読み込んで評価** を押すと、学習せずに補正後の精度が出る
- 別の日の計測データで効果を確かめるときに使う

---

## 21. キネマと伝達誤差を同時に推定する

![bg right:64% contain](img/21_joint_kinema_train.png)

- **同定パターン** で「キネマ＋伝達誤差 全軸（同時）」を選ぶ
- CSV はそのままで **学習** を押す
- J1 だけを見たいときは「キネマ＋伝達誤差 J1（同時）」を選ぶ

---

## 22. 伝達誤差の補正効果を確認する

![bg right:64% contain](img/22_joint_kinema_result.png)

- **グラフ** の見方は「キネマのみ」と同じ
- 伝達誤差も補正した分、After の平均・最大が「キネマのみ」より小さくなる
- 例：平均 0.043 → 0.015 mm

---

## 23. 伝達誤差の推定結果を見る

![bg right:64% contain](img/23_joint_kinema_table.png)

- **表** に軸・周期ごとの振幅 [deg] と位相 [deg] が出る
- 推定しなかった軸は 0 のまま
- 工具オフセットが J6 軸上（0, 0, z）だと、J6 は正しく求まらない

---

## 24. キネマを固定して伝達誤差だけを推定する

![bg right:64% contain](img/24_trans_only.png)

- 同定パターンで「伝達誤差 全軸」（または J1）を選ぶ
- 保存済みのキネマ JSON を入れ、**このパラメータを学習の初期値にする** にチェックして **学習**
- チェックがないと公称値から始まり、キネマの誤差が残る

---

<!-- header: 解析ページ｜軌跡キャリブ -->

## 25. 軌跡キャリブの設定を選ぶ

![bg right:64% contain](img/25_traj_settings.png)

- 動作中の関節角（FM）と手先の計測（BT）から推定する
- **機種**・**同定パターン** を選ぶ。パターンはキネマ補正と同じ＋「時刻・座標のみ」
- **計測点の間引き間隔**：小さいほど点が増え、学習に時間がかかる

---

## 26. FM/BT の組を入れて学習する

![bg right:64% contain](img/26_traj_train.png)

- **FM と BT の CSV** の欄に、動作ごとの `*_FM.csv` と `*_BT.csv` を入れる（**複数動作** 可）
- 2 つは末尾以外が同じ名前にしておく
- 可搬物・工具オフセットを入れて **学習** を押す

---

## 27. 補正の効果を確認する

![bg right:64% contain](img/27_traj_result.png)

- **グラフ**：Before は公称のキネマで時刻と座標だけを合わせた誤差、After は推定後の誤差
- **R²** の横に、使った計測点と動作の数が出る

---

## 28. 時刻ずれと計測器の位置を見る

![bg right:64% contain](img/28_traj_offsets.png)

- **表**：BT の先頭が FM の何秒目に当たるか（動作ごと）
- **計測器の座標**：ロボットから見た計測器の位置 [mm] と向き [deg]
- 伝達誤差を含むパターンでは、下に振幅・位相の表が出る

---

## 29. 保存済みパラメータで別の日のデータを評価する

![bg right:64% contain](img/29_traj_evaluate.png)

- 同定パターンで「時刻・座標のみ」を選ぶ
- 「保存済みパラメータを使う」を開いて **JSON** を入れ、**このパラメータを学習の初期値にする** にチェックして **学習**
- キネマ補正で保存した JSON も使える

---

<!-- header: 解析ページ｜関節補正 -->

## 30. 関節補正を学習する

![bg right:64% contain](img/30_joint_train.png)

- **関節補正** タブで **軸・減速比** を選ぶ
- `*_FM.csv` と `*_BT.csv` の組を入れて **学習**
- maxfev（反復回数の上限）は通常そのままでよい

---

## 31. 周期誤差の推定結果

![bg right:64% contain](img/31_joint_result.png)

- グラフ：補正前後の誤差と周波数成分
- 表：減速比の 1 次・2 次の **周期・振幅・位相**
- 必要なら「パラメータを保存」で JSON に残す

---

<!-- header: 解析ページ｜ツール補正 -->

## 32. ツール補正を学習する

![bg right:64% contain](img/32_tool_train.png)

- **ツール補正** タブで計測 CSV を入れて **学習**
- 列：`RobotX/Y/Z/U/V/W`、`MeasureX/Y/Z`、`ToolID`
- 姿勢（U/V/W）を変えて測ったデータが必要

---

## 33. 工具オフセットを確認する

![bg right:64% contain](img/33_tool_result.png)

- **相対 RMSE** の補正前 → 補正後で効果を見る
- 表が推定した工具ごとのオフセット [mm]
- 「パラメータを保存」で JSON に残せる

---

<!-- header: 困ったとき -->

## 34. エラーが出たとき

![bg right:64% contain](img/34_error.png)

- 失敗すると画面下に **赤い通知** が出る
- よくある原因：FM/BT の片方がない、組の名前が合っていない、列名が違う
- ファイルと列名（次ページ）を確認して、もう一度実行する

---

<!-- header: 付録 -->

## 付録：入力ファイルの形式

| 使う場所 | ファイル | 必要な列 |
|---|---|---|
| 姿勢精度 | CSV 1 つ | `RobotX/Y/Z`, `MeasureX/Y/Z` |
| 軌跡精度 | `*_BT.csv` + `*_FM.csv` | BT：`#X(mm)`, `Y(mm)`, `Z(mm)`<br>FM：`RefPos(X/Y/Z)[mm]` |
| 単軸・関節補正 | `*_FM.csv` + `*_BT.csv`（区間ごとに 1 組） | FM：時刻, `Joint(J1〜J6)[deg]`<br>BT：`TIMESTAMP`, `#X(mm)`, `Y(mm)`, `Z(mm)` |
| キネマ補正 | FARO の CSV（複数可） | `J1〜J6`, `RobotX/Y/Z`, `MeasureX/Y/Z`, `ToolID` |
| 軌跡キャリブ | `*_FM.csv` + `*_BT.csv`（動作ごとに 1 組） | FM：`Time[ms]`, `Joint(J1〜J6)[deg]`<br>BT：`TIMESTAMP`, `#X(mm)`, `Y(mm)`, `Z(mm)` |
| ツール補正 | CSV 1 つ | `RobotX/Y/Z/U/V/W`, `MeasureX/Y/Z`, `ToolID` |

FM と BT は、末尾（`_FM.csv` / `_BT.csv`）以外が同じ名前のものを組にします。
