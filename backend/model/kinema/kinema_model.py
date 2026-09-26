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

# ベイズ推定の事前分布の標準偏差（事前平均は学習開始時の値）。単位は mm / deg / 剛性率の比 / deg、観測モデルはほぼ無情報
DEFAULT_PRIOR_STD = {"length": 1.0, "angle": 0.1, "stiffness_rate": 0.5, "trans": 0.01, "observation": 1e3}


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
        bayes: bool = False,
        prior_std: dict[str, float] | None = None,
        noise_std: float | None = None,
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

        # bayes=True のときは点推定の代わりにラプラス近似（MAP + ガウス事後分布）で推定する。noise_std=None は残差から自動推定
        self.bayes = bayes
        self.prior_std = {**DEFAULT_PRIOR_STD, **(prior_std or {})}
        self.noise_std = noise_std
        self.posterior_: dict[str, dict[str, float]] = {}
        self.posterior_noise_std_: float | None = None

    @property
    def calibration_result_(self) -> dict[str, Any] | None:
        return self.kinema.calibration_result_

    @property
    def calibration_parameter_indices(self) -> tuple[int, ...]:
        return self.kinema.calibration_parameter_indices

    def fit(self, X: Any, y: Any) -> "KinemaModel":
        data = parse_kinema_input(X)
        targets = self.observation.validate_targets(y, len(data.joints))

        # ロボット側と観測側のパラメータを 1 本のベクトルにして同時に最小二乗推定する
        tool_parameter_indices = self._tool_parameter_indices(data)
        robot_count = len(self._get_robot_parameters(tool_parameter_indices))
        initial = np.hstack((self._get_robot_parameters(tool_parameter_indices), self.observation.parameter_vector()))
        if self.bayes:
            result, noise = self._fit_bayes(initial, data, targets, tool_parameter_indices, robot_count)
        else:
            # least_squares は未知数より観測が少なくても解を返してしまうため、ここで止める（ベイズは事前分布があるので解ける）
            if targets.size < initial.size:
                raise ValueError("training data contains fewer values than trainable parameters")
            result = least_squares(self._residuals, initial, args=(data, targets, tool_parameter_indices), max_nfev=self.kinema.max_nfev)
        self._set_robot_parameters(result.x[:robot_count], tool_parameter_indices)
        self.observation.set_parameter_vector(result.x[robot_count:])
        parameters = self._parameter_map(result.x, robot_count, tool_parameter_indices)
        self.kinema.calibration_result_ = {
            "success": bool(result.success), "message": result.message,
            "cost": float(result.cost), "nfev": int(result.nfev), "parameters": parameters,
        }
        # 事後共分散 = (JᵀJ)⁻¹（残差を σ で割っているので J は σ 込み）。段階学習でも各段の結果を積み上げる
        if self.bayes:
            std = np.sqrt(np.diag(np.linalg.pinv(result.jac.T @ result.jac)))
            self.posterior_.update({name: {"mean": value, "std": float(s)} for (name, value), s in zip(parameters.items(), std, strict=True)})
            self.posterior_noise_std_ = noise
            self.kinema.calibration_result_.update({"noise_std": noise, "posterior_std": dict(zip(parameters, std.tolist()))})
        # 収束しなかった結果を使わないよう、失敗は呼び出し側へ知らせる
        if not result.success:
            raise RuntimeError(f"kinematic calibration failed: {result.message}")
        return self

    # 事前分布付き最小二乗で MAP 推定する。σ 未指定なら、残差 RMS で σ を更新して再フィットする（経験ベイズ）
    def _fit_bayes(self, initial: np.ndarray, data: KinemaInput, targets: np.ndarray, tool_parameter_indices: np.ndarray, robot_count: int) -> tuple[Any, float]:
        prior_std = self._prior_std_vector(self._parameter_map(initial, robot_count, tool_parameter_indices))
        residuals = lambda values, noise: np.hstack((self._residuals(values, data, targets, tool_parameter_indices) / noise, (values - initial) / prior_std))
        rms = lambda values: float(np.sqrt(np.mean(self._residuals(values, data, targets, tool_parameter_indices) ** 2)))
        noise, values = self.noise_std or rms(initial), initial
        for round_index in range(1 if self.noise_std else 3):
            # 2 回目以降は前回の MAP の残差で σ を更新する（最終フィットの σ とヤコビアンを揃えるため、フィット前に更新）
            if round_index:
                noise = rms(values)
            result = least_squares(residuals, values, args=(noise,), max_nfev=self.kinema.max_nfev)
            values = result.x
        return result, float(noise)

    # パラメータ名（_parameter_map のキー）から種類を判定して、事前分布の標準偏差を並べる
    def _prior_std_vector(self, parameters: dict[str, float]) -> np.ndarray:
        def kind(name: str) -> str:
            if name.startswith("geometry"):
                return "length" if int(name[-2]) < 3 else "angle"
            return {"dh": "angle", "tool_offsets": "length", "stiffness_rate": "stiffness_rate", "trans": "trans", "observation": "observation"}[name.split("[")[0]]
        return np.array([self.prior_std[kind(name)] for name in parameters], dtype=np.float64)

    # ベイズ推定したときだけ、保存結果に事後平均・標準偏差を添える（load では無視されるので旧形式とも互換）
    def _with_posterior(self, result: dict[str, Any]) -> dict[str, Any]:
        if self.bayes and self.posterior_:
            result["posterior"] = {"noise_std": self.posterior_noise_std_, "parameters": self.posterior_}
        return result

    def predict(self, X: Any) -> np.ndarray:
        data = parse_kinema_input(X)
        return self.observation.transform(self._predict_positions(data), data.sequence_ids, data.times)

    def score(self, X: Any, y: Any) -> float:
        actual = self.observation.validate_targets(y, len(parse_kinema_input(X).joints))
        return r2_score(actual, self.predict(X))

    def predict_positions(self, X: Any) -> np.ndarray:
        """観測モデルを通さない工具先端の XYZ を返す。"""
        return self._predict_positions(parse_kinema_input(X))

    def save(self) -> dict[str, Any]:
        # 観測モデルが identity のときは、旧形式と互換のロボットパラメータだけを返す
        if isinstance(self.observation, IdentityObservationModel):
            return self._with_posterior(self.kinema.save_parameters() if isinstance(self.kinema, CorrectedKinema) else {"DHParam": self.params.dh_param.tolist()})
        return self._with_posterior({"robot_model": self._save_robot(), "observation_model": {"type": observation_type(self.observation), "parameters": self.observation.save()}})

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
