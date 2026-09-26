# robot-calibration-app

フロントエンドを NiceGUI、バックエンドを FastAPI で作るアプリのサンプル。Docker での実行と、Windows 用 exe / インストーラのビルドができる。

## 構成

```
robot-calibration-app/
├── compose.yaml              # frontend + backend（描画・解析）を Docker で起動（自動起動あり）
├── .dockerignore             # ルートをコンテキストにするビルド用（requirements.txt のみ送る）
├── .devcontainer/            # VSCode Dev Container 設定
├── backend/                  # FastAPI（API サーバー）
│   ├── draw.py               # 描画 API（:8000）。/plot/... で matplotlib の PNG を返す
│   ├── analysis.py           # 解析 API（:8001）。キャリブレーションモデルの学習・推定・評価
│   ├── robot_sim.py          # ロボット API のシミュレータ（:8002）。逐次最適化で実機の代わりに使う
│   ├── plot/                 # 描画処理（reference/DrawServer から移植）
│   ├── model/                # 解析モデル（reference/AnalysisServer から移植）
│   ├── requirements.txt
│   └── Dockerfile
├── frontend/                 # NiceGUI（画面）。描画 API・解析 API を HTTP で呼び出す
│   ├── main.py               # 起動。/ は /draw へ移動
│   ├── draw.py               # 描画ページ /draw（姿勢精度・軌跡精度・単軸）
│   ├── analysis.py           # 解析ページ /analysis（キネマ補正・軌跡キャリブ・関節補正・ツール補正）
│   ├── optimize.py           # 逐次最適化ページ /optimize（計測 → パラメータ更新の反復とライブ描画）
│   ├── layout.py             # 共通のヘッダー・ファイル選択・グラフ表示
│   ├── api.py                # 描画 API・解析 API・ロボット API の呼び出し
│   ├── loaders.py            # 計測 CSV → API の入力形式への変換
│   ├── joint_wave.py         # 単軸計測（FM/BT の組）→ 関節角ごとの周期誤差
│   ├── requirements.txt
│   └── Dockerfile
├── desktop/
│   └── launcher.py           # Windows exe 用: 描画 API・解析 API・frontend をまとめて起動
├── windows/
│   ├── build.sh              # Windows 版ビルドスクリプト
│   ├── build-in-container.sh # ビルド用コンテナ内で実行される処理
│   ├── Dockerfile            # ビルド用イメージ（Wine + Windows 版 Python + NSIS）
│   └── installer.nsi         # インストーラ定義
├── docs/manual/              # 使い方マニュアル（Marp のスライドと書き出した HTML、スクリーンショット）
└── scripts/
    └── make_manual.py        # マニュアル用のスクリーンショットをダミーデータで撮る
```

`backend/` と `frontend/` はそれぞれ Python パッケージ（`backend.draw`、`backend.analysis`、`frontend.main`）として import できる。Docker でも exe でも同じ import パスで動かしている。

### 通信の流れ

```
ブラウザ ──> frontend (NiceGUI :8080) ──HTTP──> 描画 API (FastAPI :8000)
                                                解析 API (FastAPI :8001)
                                                ロボット API (実機 or シミュレータ :8002)
```

描画 API と解析 API は別プロセスで動かす。解析 API の学習（scipy の最適化）は重く GIL を占有するため、同じプロセスにすると学習中は描画が止まってしまう。解析 API は `/init` で作ったモデルを 1 つだけ保持する。

frontend から各 API への接続先は環境変数 `DRAW_URL`（デフォルトは `http://127.0.0.1:8000`）、`ANALYSIS_URL`（デフォルトは `http://127.0.0.1:8001`）、`ROBOT_URL`（デフォルトは `http://127.0.0.1:8002`）で指定する。計測 CSV は frontend で読み込んで JSON に変換し、backend には JSON だけを送る。

### 画面

| ページ | タブ | 入力するファイル |
|---|---|---|
| 描画 `/draw` | 姿勢精度 | 系列ごとに Robot/Measure の CSV |
| | 軌跡精度 | 系列ごとに `*_BT.csv` と `*_FM.csv` の組 |
| | 単軸 | 系列ごとに `*_FM.csv` と `*_BT.csv` の組（複数区間なら複数組） |
| 解析 `/analysis` | キネマ補正 | FARO の CSV（J1〜J6、RobotXYZ、MeasureXYZ、ToolID）。同定パターンで関節の伝達誤差（周期は機種 JSON の `TransErrPeriod`）も同時に推定できる。保存したパラメータ JSON で評価・学習の初期値にもできる |
| | 軌跡キャリブ | 動作ごとに `*_FM.csv`（関節角の軌道）と `*_BT.csv`（計測器の手先軌跡）の組。キネマ・伝達誤差に加え、動作ごとの時刻ずれと計測器の座標も推定する |
| | 関節補正 | `*_FM.csv` と `*_BT.csv` の組 |
| | ツール補正 | RobotXYZUVW、MeasureXYZ、ToolID の CSV |
| 逐次最適化 `/optimize` | – | 目標の関節角軌道 `*_FM.csv`（無ければデモ軌道）。計測はロボット API から受け取る |

FM と BT の組は、ファイル名の末尾（`_FM.csv` / `_BT.csv`）が違うだけの同じ名前で対応付ける。解析 API は学習済みモデルを 1 つだけ保持するため、別のタブで学習するとそれまでのモデルは置き換わる。

### 逐次最適化

目標の関節角軌道（公称キネマでの手先軌跡が目標）をロボットに動かして計測し、次の反復を自動で（または 1 反復ずつ）繰り返す。

1. ロボット API `/run` で指令軌道を動かし、FM（モータへ出した関節角）と BT（計測器の手先位置）を受け取る
2. これまでの全反復の FM/BT で軌跡キャリブ（`trajectory` モデル）を学習する。前回の結果を初期値にし、評価回数の上限で打ち切る
3. 解析 API `/tracking_error` で今回の計測をロボット座標・ロボット時刻に直し、目標との誤差（RMS・最大）を求める
4. 誤差が改善している間は同定したパラメータをロボット API `/parameters` に送る。改善率が閾値を下回ったら、パラメータを固定して解析 API `/correct` で指令関節角の軌道を修正する（公称キネマの位置ヤコビアンの擬似逆行列 × 誤差 × ゲイン）
5. RMS が目標精度以下になるか、最大反復回数に達したら終わる

ロボット API（実機もこの仕様で作る）:

| エンドポイント | 入力 | 出力 |
|---|---|---|
| `GET /info` | – | `{robot_type, tool_offsets}` |
| `POST /parameters` | 解析 API `/save` の JSON（`robot_model`、`transmission_error`） | `{status}` |
| `POST /run` | `{time: [s], joints: [[J1..J6], ...]}`（公称キネマ基準の指令軌道） | `{fm: {time, joints}, bt: {time, xyz}}` |

ロボットは受け取ったパラメータで、補正キネマの手先が公称キネマの手先（指令）に一致する関節角を解いてモータへ出す。`fm.joints` はその補正後の関節角、`bt` は計測器の座標・時刻の手先位置。同定は FM から行うため、送ったパラメータによらず全反復のデータをまとめて使える。

シミュレータ（`backend/robot_sim.py`）は公称キネマに乱数の幾何誤差・剛性率・伝達誤差を加えたロボットで、計測器の座標・時計のずれとノイズを付けて返す。同定モデルにない誤差としてバックラッシも入れており、パラメータ更新で頭打ちになった後の軌道修正を試せる。環境変数 `SIM_SEED`（乱数）と `SIM_BACKLASH`（バックラッシの倍率、0 で無し）で変えられる。

1 つの軌跡から同定するため、軌跡で動かない方向のパラメータは決まらない（例: 工具が J6 軸上にあると J6 の原点や伝達誤差、可搬物が軽いと剛性率）。こうしたパラメータは値がふらつくので、同定パターンと `calibration_mode` で推定対象から外す（既定は剛性を推定しない `kinema_actual`）。

## Docker で実行する

```bash
docker compose up -d --build
```

- 画面（NiceGUI）: http://localhost:8080
- 描画 API: http://localhost:8000（ドキュメント: http://localhost:8000/docs）
- 解析 API: http://localhost:8001（ドキュメント: http://localhost:8001/docs）
- ロボット API のシミュレータ: http://localhost:8002（ドキュメント: http://localhost:8002/docs）

compose 内では frontend に `DRAW_URL=http://backend:8000`、`ANALYSIS_URL=http://analysis:8001`、`ROBOT_URL=http://robot-sim:8002` を渡している（実機を使うときは `ROBOT_URL` を実機に向ける）。解析 API は backend と同じイメージを `analysis` サービスとして別コンテナで起動する。どのサービスも `restart: unless-stopped` を設定しているため、Docker の起動時に自動で起動する（`docker compose stop` で止めた場合を除く）。

| 操作 | コマンド |
|---|---|
| 状態確認 | `docker compose ps` |
| ログ確認 | `docker compose logs -f frontend`（または `backend`、`analysis`、`robot-sim`） |
| 停止 | `docker compose stop` |
| 削除 | `docker compose down` |

## Windows 版をビルドする

### 必要なもの

- Docker（WSL / Linux 上で実行する）

Windows 側に Python やビルドツールをインストールする必要はない。

### ビルド

プロジェクト直下で実行する。

```bash
# 単体 exe
./windows/build.sh exe
# → dist/robot-calibration.exe

# インストーラ
./windows/build.sh installer
# → dist/robot-calibration-setup.exe
```

初回はビルド用イメージの作成に時間がかかる。NiceGUI を含むため、PyInstaller の処理も 10 分前後かかる。

### 仕組み

PyInstaller は実行中の OS 向けのバイナリしか作れないため、Linux 上で Windows の exe を作るには Windows 版 Python が必要になる。そこでビルド用イメージ（`windows/Dockerfile`）では Wine 上の Windows 版 Python で PyInstaller を実行している。インストーラは Linux 版の NSIS（`makensis`）で作成する。

ビルド用コンテナは実行用コンテナとは別。Wine などのビルド用ツールを実行用イメージに入れないためで、ビルドが終わるとコンテナは削除される（`--rm`）。

exe のエントリポイントは `desktop/launcher.py`。描画 API を別スレッド、解析 API とロボットシミュレータを別プロセス、frontend をメインスレッドで起動する（`--robot-url` で実機を指定するとシミュレータは起動しない）。Docker では別コンテナに分かれている 3 つを、1 つの exe にまとめて配布できる。

| モード | PyInstaller | 出力 |
|---|---|---|
| `exe` | `--onefile`（1 ファイルにまとめる） | `dist/robot-calibration.exe` |
| `installer` | `--onedir` + NSIS | `dist/robot-calibration-setup.exe` |

- NiceGUI の静的ファイル（JS/CSS）と、キネマモデルの機種パラメータ（`backend/model/kinema/param/*.json`）は `--add-data` で同梱している。
- UPX 圧縮は無効（`--noupx`）。UPX で圧縮するとウイルス対策ソフトに誤検知されやすいため。

## Windows 版の使い方

### 単体 exe

```
robot-calibration.exe                       # 起動してブラウザで http://127.0.0.1:8080 を開く
robot-calibration.exe --no-browser          # ブラウザを自動で開かない
robot-calibration.exe --frontend-port 9080 --backend-port 9000 --analysis-port 9001   # ポートを変更
```

コンソールウィンドウを閉じるか `Ctrl+C` で停止する。

単体 exe は起動のたびに一時フォルダへ展開するため、画面が表示されるまで 20 秒ほどかかる。起動を速くしたい場合はインストーラ版（`--onedir`）を使う。

### インストーラ

`robot-calibration-setup.exe` を実行すると、以下が行われる（管理者権限が必要）。

- `C:\Program Files\RobotCalibration` にインストール
- スタートメニューに「robot-calibration」「Uninstall」を作成
- 「設定 → アプリ」に登録（ここからアンインストールできる）

### 注意

- 署名していないため、初回の実行時に SmartScreen の警告が出ることがある。「詳細情報 → 実行」で起動できる。

## 使い方マニュアル

`docs/manual/manual.md`（Marp 形式のスライド）と、書き出した `docs/manual/manual.html` がある。スクリーンショット（`docs/manual/img/`）は `scripts/make_manual.py` がダミーデータで画面を操作して撮る。画面を改修したら、アプリを起動した状態でプロジェクト直下から次を実行して撮り直す。

```bash
# 逐次最適化の画面はシミュレータを使うため、起動し直して初期状態にしておく
docker compose up -d --build && docker compose restart robot-sim
# スクリーンショットを撮る（Playwright の公式イメージ内で実行する）
docker run --rm --network host -u "$(id -u):$(id -g)" -e HOME=/tmp -v "$PWD":/work -w /work \
    mcr.microsoft.com/playwright/python:v1.63.0-noble \
    sh -c "pip install -q --user --break-system-packages playwright==1.63.0 && python scripts/make_manual.py"
# スライドを HTML に書き出す
docker run --rm -v "$PWD":/home/marp/app -e MARP_USER="$(id -u):$(id -g)" marpteam/marp-cli docs/manual/manual.md -o docs/manual/manual.html
```

- 逐次最適化のスライドは、シミュレータで最後まで反復した結果を撮る（数十秒かかる）
- ダミーデータの一部（キネマ補正・ツール補正）は解析 API のモデルで作るため、実行すると解析 API が保持しているモデルは置き換わる
- 画面の部品やボタン名を変えた場合は、`scripts/make_manual.py` の操作手順とスライドの説明文も合わせて直す
- HTML は画像を `img/` から読むため、配布するときは `docs/manual/` フォルダごと渡す

## 開発（Dev Container）

VSCode のコマンドパレットで「Dev Containers: Reopen in Container」を選択する。

- `.devcontainer/Dockerfile` で backend と frontend の依存関係をまとめてインストールする
- 起動時に描画 API・解析 API（`uvicorn --reload`）、ロボットシミュレータと frontend を立ち上げる
  - backend は保存すると自動で再読み込みされる（解析 API の学習済みモデルは消える）。frontend は再起動が必要
  - ログは `/tmp/backend.log`、`/tmp/analysis.log`、`/tmp/robot_sim.log`、`/tmp/frontend.log`
- compose のサービスがポート 8000 / 8001 / 8002 / 8080 を使っている場合、VSCode は空いている別のポートに転送する（「ポート」タブで確認）
