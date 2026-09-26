"""ロボットの順運動学と観測モデルを 1 つのモデルとして同時に学習するキネマキャリブレーション。"""
from typing import Any

import numpy as np
from scipy.optimize import least_squares

from ..base import BaseModel, r2_score
from ..observation import IdentityObservationModel, create_observation, observation_type
from .corrected_kinema import CorrectedKinema
from .ideal_kinema import IdealKinema
from .input import KinemaInput, parse_kinema_input
from .kinema_parameter import KinemaModelParam
from .tool import apply_tool_offsets


class KinemaModel(BaseModel):
    """X = 関節角（+ tool_id, sequence_id, time）、y = 観測値 として機構パラメータを推定する。"""

    def __init__(
        self,
        robot_type: str = "R6E",
        mode: str = "actual",
        payload_mass: float = 0.0,
        payload_center: Any | None = None,
        gravity_direction: Any | None = None,
        tool_offsets: list[list[float]] | np.ndarray | None = None,
        calibration_mode: str | None = None,
        max_nfev: int | None = None,
        robot_model: dict[str, Any] | None = None,
        observation_model: dict[str, Any] | str | None = None,
        **_: Any,
    ) -> None:
        # robot_model.settings で指定された値を、トップレベルの引数より優先する
        robot_settings = dict((robot_model or {}).get("settings", {}))
        robot_type = robot_settings.get("robot_type", robot_type)
        mode = robot_settings.get("mode", mode)
        payload_mass = robot_settings.get("payload_mass", payload_mass)
        payload_center = robot_settings.get("payload_center", payload_center)
        gravity_direction = robot_settings.get("gravity_direction", gravity_direction)
        tool_offsets = robot_settings.get("tool_offsets", tool_offsets)
        calibration_mode = robot_settings.get("calibration_mode", calibration_mode)
        max_nfev = robot_settings.get("max_nfev", max_nfev)

        self.mode = mode
        self.params = KinemaModelParam(robot_type)
        self.params.set_payload(payload_mass, payload_center, gravity_direction)
        self.tool_offsets = np.asarray([[0.0, 0.0, 0.0]] if tool_offsets is None else tool_offsets, dtype=np.float64)

        # ideal は公称 DH（原点角度のみ補正）、actual はたわみ補正付きの詳細モデル
        if mode == "ideal":
            self.kinema = IdealKinema(self.params, calibration_mode=calibration_mode or "local", max_nfev=max_nfev)
        else:
            self.kinema = CorrectedKinema(self.params, calibration_mode=calibration_mode or "all_actual", max_nfev=max_nfev)
        self.observation = create_observation(observation_model, "identity")

    @property
    def calibration_result_(self) -> dict[str, Any] | None:
        return self.kinema.calibration_result_

    @property
    def calibration_parameter_indices(self) -> tuple[int, ...]:
        return self.kinema.calibration_parameter_indices

    def fit(self, X: Any, y: Any) -> "KinemaModel":
        data = self._parse(X)
        targets = self.observation.validate_targets(y, len(data.joints))

        # ロボット側と観測側のパラメータを 1 本のベクトルにして同時に最小二乗推定する
        tool_parameter_indices = self._tool_parameter_indices(data)
        robot_count = len(self._get_robot_parameters(tool_parameter_indices))
        initial = np.hstack((self._get_robot_parameters(tool_parameter_indices), self.observation.parameter_vector()))
        # least_squares は未知数より観測が少なくても解を返してしまうため、ここで止める
        if targets.size < initial.size:
            raise ValueError("training data contains fewer values than trainable parameters")
        result = least_squares(self._residuals, initial, args=(data, targets, tool_parameter_indices), max_nfev=self.kinema.max_nfev)
        self._set_robot_parameters(result.x[:robot_count], tool_parameter_indices)
        self.observation.set_parameter_vector(result.x[robot_count:])
        self.kinema.calibration_result_ = {
            "success": bool(result.success), "message": result.message,
            "cost": float(result.cost), "nfev": int(result.nfev),
            "parameters": self._parameter_map(result.x, robot_count, tool_parameter_indices),
        }
        # 収束しなかった結果を使わないよう、失敗は呼び出し側へ知らせる（max_nfev で打ち切った場合は、逐次最適化で続きから推定するため使う）
        if not result.success and result.status != 0:
            raise RuntimeError(f"kinematic calibration failed: {result.message}")
        return self

    def predict(self, X: Any) -> np.ndarray:
        data = self._parse(X)
        return self.observation.transform(self._predict_positions(data), data.sequence_ids, data.times)

    def score(self, X: Any, y: Any) -> float:
        actual = self.observation.validate_targets(y, len(self._parse(X).joints))
        return r2_score(actual, self.predict(X))

    def predict_positions(self, X: Any) -> np.ndarray:
        """観測モデルを通さない工具先端の XYZ を返す。"""
        return self._predict_positions(self._parse(X))

    def save(self) -> dict[str, Any]:
        # 観測モデルが identity のときは、旧形式と互換のロボットパラメータだけを返す
        if isinstance(self.observation, IdentityObservationModel):
            return self.kinema.save_parameters() if isinstance(self.kinema, CorrectedKinema) else {"DHParam": self.params.dh_param.tolist()}
        return {"robot_model": self._save_robot(), "observation_model": {"type": observation_type(self.observation), "parameters": self.observation.save()}}

    def load(self, parameters: dict[str, Any]) -> None:
        # 旧形式（ロボットパラメータのみ）と新形式（robot_model + observation_model）の両方を受け付ける
        if "robot_model" not in parameters:
            self._load_robot(parameters)
            return
        self._load_robot(parameters["robot_model"])
        observation = parameters.get("observation_model", {})
        observation_name = observation.get("type", "identity")
        if observation_name != observation_type(self.observation):
            self.observation = create_observation({"type": observation_name}, "identity")
        self.observation.load(observation.get("parameters", {}))

    # X を分解する（入力形式の違う派生モデルが差し替える）
    def _parse(self, X: Any) -> KinemaInput:
        return parse_kinema_input(X)

    # 最小二乗法で最小化する残差（観測値 - 現在のパラメータでの予測値）
    def _residuals(self, values: np.ndarray, data: KinemaInput, targets: np.ndarray, tool_parameter_indices: np.ndarray) -> np.ndarray:
        robot_count = len(self._get_robot_parameters(tool_parameter_indices))
        self._set_robot_parameters(values[:robot_count], tool_parameter_indices)
        self.observation.set_parameter_vector(values[robot_count:])
        predicted = self.observation.transform(self._predict_positions(data), data.sequence_ids, data.times)
        return (targets - predicted).reshape(-1)

    def _predict_positions(self, data: KinemaInput) -> np.ndarray:
        return apply_tool_offsets(self.kinema.predict(data.joints), self.tool_offsets, data.tool_indices)[:, :3]

    # 工具オフセットも推定するモードでは、データに現れる工具だけを推定対象にする
    def _tool_parameter_indices(self, data: KinemaInput) -> np.ndarray:
        if isinstance(self.kinema, IdealKinema) and self.kinema.optimize_tool_offsets:
            return np.unique(data.tool_indices).astype(np.int64)
        return np.empty(0, dtype=np.int64)

    # ideal / actual で異なるパラメータ入出力の呼び分け
    def _get_robot_parameters(self, tool_parameter_indices: np.ndarray) -> np.ndarray:
        if isinstance(self.kinema, IdealKinema):
            return self.kinema._get_calibration_parameters(self.tool_offsets, tool_parameter_indices)
        return self.kinema._get_calibration_parameters()

    def _set_robot_parameters(self, values: np.ndarray, tool_parameter_indices: np.ndarray) -> None:
        if isinstance(self.kinema, IdealKinema):
            self.kinema._set_calibration_parameters(values, self.tool_offsets, tool_parameter_indices)
        else:
            self.kinema._set_calibration_parameters(values)

    def _parameter_map(self, values: np.ndarray, robot_count: int, tool_parameter_indices: np.ndarray) -> dict[str, float]:
        if isinstance(self.kinema, IdealKinema):
            result = self.kinema._calibration_parameter_map(values[:robot_count], tool_parameter_indices)
        else:
            result = self.kinema._calibration_parameter_map(values[:robot_count])
        result.update({f"observation[{index}]": float(value) for index, value in enumerate(values[robot_count:])})
        return result

    def _save_robot(self) -> dict[str, Any]:
        result = self.kinema.save_parameters() if isinstance(self.kinema, CorrectedKinema) else {"DHParam": self.params.dh_param.tolist()}
        result["ToolOffsets"] = self.tool_offsets.tolist()
        return result

    def _load_robot(self, parameters: dict[str, Any]) -> None:
        if isinstance(self.kinema, CorrectedKinema):
            self.kinema.load_parameters(parameters)
        elif "DHParam" in parameters:
            self.params.dh_param[:] = np.asarray(parameters["DHParam"], dtype=np.float64)
        if "ToolOffsets" in parameters:
            self.tool_offsets = np.asarray(parameters["ToolOffsets"], dtype=np.float64)
