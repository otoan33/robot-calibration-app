# robot-calibration-app

フロントエンドを NiceGUI、バックエンドを FastAPI で作るアプリのサンプル。Docker での実行と、Windows 用 exe / インストーラのビルドができる。

## 構成

```
robot-calibration-app/
├── compose.yaml              # frontend + backend を Docker で起動（自動起動あり）
├── .dockerignore             # ルートをコンテキストにするビルド用（requirements.txt のみ送る）
├── .devcontainer/            # VSCode Dev Container 設定
├── backend/                  # FastAPI（API サーバー）
│   ├── main.py
│   ├── requirements.txt
│   └── Dockerfile
├── frontend/                 # NiceGUI（画面）。backend を HTTP で呼び出す
│   ├── main.py
│   ├── requirements.txt
│   └── Dockerfile
├── desktop/
│   └── launcher.py           # Windows exe 用: backend と frontend を 1 プロセスで起動
└── windows/
    ├── build.sh              # Windows 版ビルドスクリプト
    ├── build-in-container.sh # ビルド用コンテナ内で実行される処理
    ├── Dockerfile            # ビルド用イメージ（Wine + Windows 版 Python + NSIS）
    └── installer.nsi         # インストーラ定義
```

`backend/` と `frontend/` はそれぞれ Python パッケージ（`backend.main`、`frontend.main`）として import できる。Docker でも exe でも同じ import パスで動かしている。

### 通信の流れ

```
ブラウザ ──> frontend (NiceGUI :8080) ──HTTP──> backend (FastAPI :8000)
```

frontend から backend への接続先は環境変数 `BACKEND_URL` で指定する（デフォルトは `http://127.0.0.1:8000`）。

## Docker で実行する

```bash
docker compose up -d --build
```

- 画面（NiceGUI）: http://localhost:8080
- API（FastAPI）: http://localhost:8000
- API ドキュメント: http://localhost:8000/docs

compose 内では frontend に `BACKEND_URL=http://backend:8000` を渡している。どちらのサービスも `restart: unless-stopped` を設定しているため、Docker の起動時に自動で起動する（`docker compose stop` で止めた場合を除く）。

| 操作 | コマンド |
|---|---|
| 状態確認 | `docker compose ps` |
| ログ確認 | `docker compose logs -f frontend`（または `backend`） |
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

exe のエントリポイントは `desktop/launcher.py`。backend を別スレッドで起動し、frontend をメインスレッドで起動する。Docker では別コンテナに分かれている 2 つを、1 つの exe にまとめて配布できる。

| モード | PyInstaller | 出力 |
|---|---|---|
| `exe` | `--onefile`（1 ファイルにまとめる） | `dist/robot-calibration.exe` |
| `installer` | `--onedir` + NSIS | `dist/robot-calibration-setup.exe` |

- NiceGUI の静的ファイル（JS/CSS）は `--add-data` で同梱している。
- UPX 圧縮は無効（`--noupx`）。UPX で圧縮するとウイルス対策ソフトに誤検知されやすいため。

## Windows 版の使い方

### 単体 exe

```
robot-calibration.exe                       # 起動してブラウザで http://127.0.0.1:8080 を開く
robot-calibration.exe --no-browser          # ブラウザを自動で開かない
robot-calibration.exe --frontend-port 9080 --backend-port 9000   # ポートを変更
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

## 開発（Dev Container）

VSCode のコマンドパレットで「Dev Containers: Reopen in Container」を選択する。

- `.devcontainer/Dockerfile` で backend と frontend の依存関係をまとめてインストールする
- 起動時に backend（`uvicorn --reload`）と frontend を立ち上げる
  - backend は保存すると自動で再読み込みされる。frontend は再起動が必要
  - ログは `/tmp/backend.log` と `/tmp/frontend.log`
- compose のサービスがポート 8000 / 8080 を使っている場合、VSCode は空いている別のポートに転送する（「ポート」タブで確認）
