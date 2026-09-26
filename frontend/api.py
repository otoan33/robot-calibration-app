"""描画 API と解析 API の呼び出し。接続先は環境変数 DRAW_URL / ANALYSIS_URL で指定する（compose では各サービス名）。"""
import os

import httpx

# 学習は数分かかることがあるため、解析 API はタイムアウトしない
draw_client = httpx.AsyncClient(base_url=os.environ.get("DRAW_URL", "http://127.0.0.1:8000"), timeout=120)
analysis_client = httpx.AsyncClient(base_url=os.environ.get("ANALYSIS_URL", "http://127.0.0.1:8001"), timeout=None)


async def plot(kind: str, func: str, body: dict) -> bytes:
    """描画 API で PNG を作る。kind は pose_accuracy / path_accuracy / single_axes。"""
    response = await draw_client.post(f"/plot/{kind}/{func}", json=body)
    response.raise_for_status()
    return response.content


async def analysis(path: str, body: dict | None = None) -> dict:
    """解析 API（/init, /train, /predict, /evaluate, /save, /load）を呼ぶ。"""
    response = await analysis_client.post(path, json=body or {})
    response.raise_for_status()
    return response.json()


# ロボット API（実機またはシミュレータ）。動作と計測に時間がかかるためタイムアウトしない
robot_client = httpx.AsyncClient(base_url=os.environ.get("ROBOT_URL", "http://127.0.0.1:8002"), timeout=None)


async def robot(path: str, body: dict | None = None) -> dict:
    """ロボット API（GET /info、POST /parameters, /run）を呼ぶ。body が無ければ GET にする。"""
    response = await (robot_client.get(path) if body is None else robot_client.post(path, json=body))
    response.raise_for_status()
    return response.json()
