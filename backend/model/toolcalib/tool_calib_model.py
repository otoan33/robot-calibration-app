"""計測した手先姿勢 XYZUVW から工具オフセットだけを推定するモデル。"""
from typing import Any

import numpy as np
from scipy.optimize import least_squares

from ..base import BaseModel, r2_score
from ..kinema.input import KinemaInput, parse_kinema_input
from ..kinema.tool import apply_tool_offsets
from ..observation import create_observation, observation_type


class ToolCalibModel(BaseModel):
    """ロボットの機構は補正せず、XYZ の工具オフセットだけを推定する。

    ``X`` は ``xyzuvw`` に ``tool_id``（1 始まり）と ``sequence_id, time`` を加えた ``(N, 6)`` / ``(N, 7)`` / ``(N, 9)``。
    """

    def __init__(self, tool_offsets: list[list[float]] | np.ndarray | None = None, observation_model: dict[str, Any] | str | None = "relative", max_nfev: int | None = None, **_: Any) -> None:
        self.tool_offsets = np.asarray([[0.0, 0.0, 0.0]] if tool_offsets is None else tool_offsets, dtype=np.float64)
        self.observation = create_observation(observation_model, "relative")
        self.max_nfev = max_nfev
        self.calibration_result_: dict[str, Any] | None = None

    def fit(self, X: Any, y: Any) -> "ToolCalibModel":
        data = parse_kinema_input(X)
        targets = self.observation.validate_targets(y, len(data.joints))

        # データに現れる工具のオフセットだけを最小二乗推定する
        parameter_indices = np.unique(data.tool_indices).astype(np.int64)
        initial = self.tool_offsets[parameter_indices].reshape(-1).copy()
        # least_squares は未知数より観測が少なくても解を返してしまうため、ここで止める
        if targets.size < initial.size:
            raise ValueError("training data contains fewer values than tool-offset parameters")
        result = least_squares(self._residuals, initial, args=(data, targets, parameter_indices), max_nfev=self.max_nfev)
        self.tool_offsets[parameter_indices] = result.x.reshape(-1, 3)
        self.calibration_result_ = {
            "success": bool(result.success), "message": result.message,
            "cost": float(result.cost), "nfev": int(result.nfev),
            "parameters": {
                f"tool_offsets[{tool_index},{axis}]": float(value)
                for tool_index, offset in zip(parameter_indices, result.x.reshape(-1, 3), strict=True)
                for axis, value in enumerate(offset)
            },
        }
        # 収束しなかった結果を使わないよう、失敗は呼び出し側へ知らせる
        if not result.success:
            raise RuntimeError(f"tool-offset calibration failed: {result.message}")
        return self

    def predict(self, X: Any) -> np.ndarray:
        data = parse_kinema_input(X)
        return self.observation.transform(self._positions(data), data.sequence_ids, data.times)

    def predict_positions(self, X: Any) -> np.ndarray:
        """観測モデルを通さない工具先端の XYZ を返す。"""
        return self._positions(parse_kinema_input(X))

    def score(self, X: Any, y: Any) -> float:
        return r2_score(self.observation.validate_targets(y, len(np.asarray(X))), self.predict(X))

    def save(self) -> dict[str, Any]:
        return {"ToolOffsets": self.tool_offsets.tolist(), "observation_model": {"type": observation_type(self.observation), "parameters": self.observation.save()}}

    def load(self, parameters: dict[str, Any]) -> None:
        self.tool_offsets = np.asarray(parameters.get("ToolOffsets", parameters.get("tool_offsets")), dtype=np.float64)
        observation = parameters.get("observation_model", {})
        observation_name = observation.get("type", observation_type(self.observation))
        if observation_name != observation_type(self.observation):
            self.observation = create_observation({"type": observation_name}, "relative")
        self.observation.load(observation.get("parameters", {}))

    # X の先頭 6 列（xyzuvw）は計測した手先姿勢そのもの
    def _positions(self, data: KinemaInput) -> np.ndarray:
        return apply_tool_offsets(data.joints, self.tool_offsets, data.tool_indices)[:, :3]

    # 最小二乗法で最小化する残差（観測値 - 現在のオフセットでの予測値）
    def _residuals(self, values: np.ndarray, data: KinemaInput, targets: np.ndarray, indices: np.ndarray) -> np.ndarray:
        self.tool_offsets[indices] = values.reshape(-1, 3)
        return (targets - self.observation.transform(self._positions(data), data.sequence_ids, data.times)).reshape(-1)
