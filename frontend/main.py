import os

import httpx
from nicegui import ui

# バックエンド（FastAPI）への接続。compose では BACKEND_URL=http://backend:8000 を渡す
client = httpx.AsyncClient(base_url=os.environ.get("BACKEND_URL", "http://127.0.0.1:8000"))


@ui.page("/")
def index():
    ui.label("robot-calibration").classes("text-2xl font-bold")

    # バックエンドが応答するかを確かめる
    with ui.card():
        ui.label("バックエンドの疎通確認")
        message = ui.label()
        async def ping(): message.text = (await client.get("/")).json()["message"]
        ui.button("GET /", on_click=ping)

    # 入力内容をバックエンドに登録し、結果を表示する
    with ui.card():
        ui.label("アイテム登録")
        name, price, result = ui.input("名前"), ui.number("価格", value=0), ui.label()
        async def create(): result.text = str((await client.post("/items", json={"name": name.value, "price": price.value})).json()["created"])
        ui.button("POST /items", on_click=create)


# 単体起動（Docker / Dev Container）用。exe では desktop/launcher.py から起動する
if __name__ in {"__main__", "__mp_main__"}:
    ui.run(host="0.0.0.0", port=8080, title="robot-calibration", reload=False, show=False)
