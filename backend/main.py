from fastapi import FastAPI
from pydantic import BaseModel

app = FastAPI(title="robot-calibration API")


# アイテム登録で受け取るデータの形式
class Item(BaseModel):
    name: str
    price: float


# フロントエンドからの疎通確認用
@app.get("/")
def root():
    return {"message": "Hello from FastAPI in Docker!"}


# パスパラメータとクエリパラメータの受け取り例
@app.get("/items/{item_id}")
def read_item(item_id: int, q: str | None = None):
    return {"item_id": item_id, "q": q}


# JSON ボディの受け取り例（保存はせず、そのまま返す）
@app.post("/items")
def create_item(item: Item):
    return {"created": item}
