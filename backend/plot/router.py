"""描画 API のエンドポイント。URL は DrawServer と同じ /plot/{種類}/{関数名}。"""
from fastapi import APIRouter

from . import path_accuracy, pose_accuracy, single_axes

router = APIRouter(prefix="/plot", tags=["plot"])


@router.post("/pose_accuracy/{function_name}", summary="姿勢精度のグラフを描画")
def plot_pose_accuracy(function_name: str, body: pose_accuracy.PoseAccuracyInput):
    return pose_accuracy.FUNCTIONS[function_name](body)


@router.post("/path_accuracy/{function_name}", summary="直線軌跡精度のグラフを描画")
def plot_path_accuracy(function_name: str, body: path_accuracy.PathAccuracyInput):
    return path_accuracy.FUNCTIONS[function_name](body)


@router.post("/single_axes/{function_name}", summary="単軸の角度伝達誤差を描画")
def plot_single_axes(function_name: str, body: single_axes.SingleAxisInput):
    return single_axes.FUNCTIONS[function_name](body)
