"""手先位置を計測器の観測値へ変換するモデル。"""
from typing import Any

from .base import ObservationModel
from .identity import IdentityObservationModel
from .relative import RelativeObservationModel
from .scalar import ScalarObservationModel

OBSERVATION_MODELS = {
    "identity": IdentityObservationModel,
    "relative": RelativeObservationModel,
    "scalar": ScalarObservationModel,
}


# 設定（None / 名前 / {"type", "settings"}）から観測モデルを作る。dict で type を省略したときは default を使う
def create_observation(config: dict[str, Any] | str | None, default: str) -> ObservationModel:
    if config is None:
        return IdentityObservationModel()
    if isinstance(config, str):
        return OBSERVATION_MODELS[config]()
    return OBSERVATION_MODELS[config.get("type", default)](**config.get("settings", {}))


# 保存時に書き出す観測モデルの種類名
def observation_type(observation: ObservationModel) -> str:
    return next(name for name, model_class in OBSERVATION_MODELS.items() if isinstance(observation, model_class))


__all__ = [
    "ObservationModel",
    "IdentityObservationModel",
    "RelativeObservationModel",
    "ScalarObservationModel",
    "OBSERVATION_MODELS",
    "create_observation",
    "observation_type",
]
