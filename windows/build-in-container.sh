#!/usr/bin/env bash
# ビルド用コンテナの中で実行される（windows/build.sh から呼ばれる）
set -euo pipefail

APP=robot-calibration

rm -rf /tmp/build
mkdir /tmp/build
cp -r /src/backend /src/frontend /src/desktop /tmp/build/
cd /tmp/build

# NiceGUI の静的ファイル（JS/CSS など）と、キネマモデルが読む機種パラメータ JSON を同梱する
NICEGUI_DIR="$(wine python -c 'import nicegui, os; print(os.path.dirname(nicegui.__file__))' | tr -d '\r')"
COMMON=(--noconfirm --noupx --name "$APP" --paths . --add-data "$NICEGUI_DIR;nicegui" --add-data "backend/model/kinema/param;backend/model/kinema/param")

case "$MODE" in
  exe)
    wine python -m PyInstaller "${COMMON[@]}" --onefile desktop/launcher.py
    cp "dist/$APP.exe" /out/
    ;;
  installer)
    wine python -m PyInstaller "${COMMON[@]}" --onedir desktop/launcher.py
    makensis -V2 -DSRC_DIR="/tmp/build/dist/$APP" \
      -DOUT_FILE="/out/$APP-setup.exe" /src/windows/installer.nsi
    ;;
esac

chown -R "$HOST_UID:$HOST_GID" /out
