"""解析 API で利用するモデル実装とモデルレジストリ。"""
from .base import BaseModel
from .joint import JointCalibModel
from .kinema import KinemaJointModel, KinemaModel, TrajectoryCalibModel
from .local import LocalCalibModel
from .toolcalib import ToolCalibModel

# /init の model_type とモデルクラスの対応（toolcalib は旧名）
MODEL_REGISTRY: dict[str, type[BaseModel]] = {
    "kinema": KinemaModel,
    "kinema_joint": KinemaJointModel,
    "trajectory": TrajectoryCalibModel,
    "joint_calib": JointCalibModel,
    "local_calib": LocalCalibModel,
    "tool_calib": ToolCalibModel,
    "toolcalib": ToolCalibModel,
}

__all__ = ["BaseModel", "JointCalibModel", "KinemaJointModel", "KinemaModel", "LocalCalibModel", "ToolCalibModel", "TrajectoryCalibModel", "MODEL_REGISTRY"]
