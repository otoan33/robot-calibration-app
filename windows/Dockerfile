# Windows 用 exe / インストーラのビルド環境（Wine 上の Windows 版 Python + NSIS）
# ビルドコンテキストはプロジェクト直下（.dockerignore で requirements.txt のみ送る）
FROM tobix/pywine:3.12

RUN apt-get update \
    && apt-get install -y --no-install-recommends nsis \
    && rm -rf /var/lib/apt/lists/*

COPY backend/requirements.txt /tmp/backend-requirements.txt
COPY frontend/requirements.txt /tmp/frontend-requirements.txt
RUN wine python -m pip install --no-cache-dir \
    -r /tmp/backend-requirements.txt -r /tmp/frontend-requirements.txt pyinstaller
