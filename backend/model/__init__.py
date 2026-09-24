"""解析 API で利用するモデル実装とモデルレジストリ。"""
from .base import BaseModel
from .joint import JointCalibModel
from .kinema import KinemaModel
from .local import LocalCalibModel
from .toolcalib import ToolCalibModel

# /init の model_type とモデルクラスの対応（toolcalib は旧名）
MODEL_REGISTRY: dict[str, type[BaseModel]] = {
    "kinema": KinemaModel,
    "joint_calib": JointCalibModel,
    "local_calib": LocalCalibModel,
    "tool_calib": ToolCalibModel,
    "toolcalib": ToolCalibModel,
}

__all__ = ["BaseModel", "JointCalibModel", "KinemaModel", "LocalCalibModel", "ToolCalibModel", "MODEL_REGISTRY"]
