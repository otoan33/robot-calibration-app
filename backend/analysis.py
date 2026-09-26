"""キャリブレーションモデルの学習・推定・評価 API（reference/AnalysisServer から移植）。

学習が重くても描画 API（backend.draw）を止めないよう、別プロセス・別ポート（:8001）で起動する。
"""
from typing import Any

import numpy as np
import uvicorn
from fastapi import FastAPI
from pydantic import BaseModel, Field

from backend.model import MODEL_REGISTRY

app = FastAPI(title="robot-calibration analysis API")

# /init で作ったモデルを保持し、以降の学習・推定で使い回す（1 プロセスに 1 モデル）
model = None


class InitRequest(BaseModel):
    model_type: str
    settings: dict[str, Any] = Field(default_factory=dict)


class TrainRequest(BaseModel):
    X_train: list[list[float]] = Field(min_length=1)
    y_train: list[Any] = Field(min_length=1)


class PredictRequest(BaseModel):
    X: list[list[float]] = Field(min_length=1)


class EvaluateRequest(BaseModel):
    X_test: list[list[float]] = Field(min_length=1)
    y_test: list[Any] = Field(min_length=1)


# モデルの種類と設定を指定して作り直す（学習済みの状態は破棄される）
@app.post("/init")
def init(request: InitRequest):
    global model
    model = MODEL_REGISTRY[request.model_type](**request.settings)
    return {"status": "initialized", "model_type": request.model_type}


# 保存済みパラメータを読み込み、学習せずに推定できるようにする
@app.post("/load")
def load(request: dict[str, Any]):
    model.load(request)
    return {"status": "loaded"}


# 学習済みパラメータを JSON で返す（クライアント側でファイルに保存する）
@app.post("/save")
def save():
    return model.save()


@app.post("/train")
def train(request: TrainRequest):
    model.fit(request.X_train, request.y_train)
    return {"status": "trained"}


@app.post("/predict")
def predict(request: PredictRequest):
    return {"predictions": np.asarray(model.predict(request.X)).tolist()}


@app.post("/evaluate")
def evaluate(request: EvaluateRequest):
    return {"score": float(model.score(request.X_test, request.y_test))}


class CorrectRequest(BaseModel):
    time: list[float]
    joints: list[list[float]]
    error_time: list[float]
    errors: list[list[float]]
    gain: float = 1.0


# 逐次最適化（trajectory モデル）：計測をロボット座標に直した目標軌跡との誤差
@app.post("/tracking_error")
def tracking_error(request: EvaluateRequest):
    return model.tracking_error(request.X_test, request.y_test)


# 逐次最適化（trajectory モデル）：誤差を打ち消すよう指令関節角の軌道を修正する
@app.post("/correct")
def correct(request: CorrectRequest):
    return {"joints": model.correct(**request.model_dump()).tolist()}


# exe のランチャーから別プロセスで起動するための入口（pickle できるようモジュール直下に置く）
def serve(port: int):
    uvicorn.run(app, host="127.0.0.1", port=port)


if __name__ == "__main__":
    serve(8001)
