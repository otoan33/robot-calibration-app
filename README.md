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
│   ├── plot/                 # 描画処理（reference/DrawServer から移植）
│   ├── model/                # 解析モデル（reference/AnalysisServer から移植）
│   ├── requirements.txt
│   └── Dockerfile
├── frontend/                 # NiceGUI（画面）。描画 API・解析 API を HTTP で呼び出す
│   ├── main.py               # 起動。/ は /draw へ移動
│   ├── draw.py               # 描画ページ /draw（姿勢精度・軌跡精度・単軸）
│   ├── analysis.py           # 解析ページ /analysis（キネマ補正・関節補正・ツール補正）
│   ├── layout.py             # 共通のヘッダー・ファイル選択・グラフ表示
│   ├── api.py                # 描画 API・解析 API の呼び出し
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
```

描画 API と解析 API は別プロセスで動かす。解析 API の学習（scipy の最適化）は重く GIL を占有するため、同じプロセスにすると学習中は描画が止まってしまう。解析 API は `/init` で作ったモデルを 1 つだけ保持する。

frontend から各 API への接続先は環境変数 `DRAW_URL`（デフォルトは `http://127.0.0.1:8000`）と `ANALYSIS_URL`（デフォルトは `http://127.0.0.1:8001`）で指定する。計測 CSV は frontend で読み込んで JSON に変換し、backend には JSON だけを送る。

### 画面

| ページ | タブ | 入力するファイル |
|---|---|---|
| 描画 `/draw` | 姿勢精度 | 系列ごとに Robot/Measure の CSV |
| | 軌跡精度 | 系列ごとに `*_BT.csv` と `*_FM.csv` の組 |
| | 単軸 | 系列ごとに `*_FM.csv` と `*_BT.csv` の組（複数区間なら複数組） |
| 解析 `/analysis` | キネマ補正 | FARO の CSV（J1〜J6、RobotXYZ、MeasureXYZ、ToolID）。保存したパラメータ JSON で評価もできる |
| | 関節補正 | `*_FM.csv` と `*_BT.csv` の組 |
| | ツール補正 | RobotXYZUVW、MeasureXYZ、ToolID の CSV |

FM と BT の組は、ファイル名の末尾（`_FM.csv` / `_BT.csv`）が違うだけの同じ名前で対応付ける。解析 API は学習済みモデルを 1 つだけ保持するため、別のタブで学習するとそれまでのモデルは置き換わる。

## Docker で実行する

```bash
docker compose up -d --build
```

- 画面（NiceGUI）: http://localhost:8080
- 描画 API: http://localhost:8000（ドキュメント: http://localhost:8000/docs）
- 解析 API: http://localhost:8001（ドキュメント: http://localhost:8001/docs）

compose 内では frontend に `DRAW_URL=http://backend:8000` と `ANALYSIS_URL=http://analysis:8001` を渡している。解析 API は backend と同じイメージを `analysis` サービスとして別コンテナで起動する。どのサービスも `restart: unless-stopped` を設定しているため、Docker の起動時に自動で起動する（`docker compose stop` で止めた場合を除く）。

| 操作 | コマンド |
|---|---|
| 状態確認 | `docker compose ps` |
| ログ確認 | `docker compose logs -f frontend`（または `backend`、`analysis`） |
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

exe のエントリポイントは `desktop/launcher.py`。描画 API を別スレッド、解析 API を別プロセス、frontend をメインスレッドで起動する。Docker では別コンテナに分かれている 3 つを、1 つの exe にまとめて配布できる。

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
docker compose up -d --build
# スクリーンショットを撮る（Playwright の公式イメージ内で実行する）
docker run --rm --network host -u "$(id -u):$(id -g)" -e HOME=/tmp -v "$PWD":/work -w /work \
    mcr.microsoft.com/playwright/python:v1.63.0-noble \
    sh -c "pip install -q --user --break-system-packages playwright==1.63.0 && python scripts/make_manual.py"
# スライドを HTML に書き出す
docker run --rm -v "$PWD":/home/marp/app -e MARP_USER="$(id -u):$(id -g)" marpteam/marp-cli docs/manual/manual.md -o docs/manual/manual.html
```

- ダミーデータの一部（キネマ補正・ツール補正）は解析 API のモデルで作るため、実行すると解析 API が保持しているモデルは置き換わる
- 画面の部品やボタン名を変えた場合は、`scripts/make_manual.py` の操作手順とスライドの説明文も合わせて直す
- HTML は画像を `img/` から読むため、配布するときは `docs/manual/` フォルダごと渡す

## 開発（Dev Container）

VSCode のコマンドパレットで「Dev Containers: Reopen in Container」を選択する。

- `.devcontainer/Dockerfile` で backend と frontend の依存関係をまとめてインストールする
- 起動時に描画 API・解析 API（`uvicorn --reload`）と frontend を立ち上げる
  - backend は保存すると自動で再読み込みされる（解析 API の学習済みモデルは消える）。frontend は再起動が必要
  - ログは `/tmp/backend.log`、`/tmp/analysis.log`、`/tmp/frontend.log`
- compose のサービスがポート 8000 / 8001 / 8080 を使っている場合、VSCode は空いている別のポートに転送する（「ポート」タブで確認）
