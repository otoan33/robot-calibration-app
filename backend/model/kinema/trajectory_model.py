"""時系列の関節角軌道（FM）と計測器の手先軌跡（BT）から、キネマ・伝達誤差・時刻ずれ・計測器の座標を同定するモデル。"""
from typing import Any

import numpy as np

from .input import KinemaInput
from .kinema_joint_model import KinemaJointModel
from .tool import apply_tool_offsets

# 時刻ずれの初期値を探すときに、速さの時系列をそろえる刻み [s]
SPEED_GRID = 0.01


class TrajectoryCalibModel(KinemaJointModel):
    """計測点の時刻から関節角を補間し、補正キネマで求めた手先位置を計測器の座標系へ変換して計測値と比べる。

    ``trajectories`` は動作ごとの関節角軌道 ``[{"time": [s], "joints": [[J1..J6], ...]}]``。
    ``X`` は計測点の ``[sequence_id（trajectories の番号）, 計測時刻 s]``、``y`` は計測器座標の XYZ [mm]。
    ロボットの時刻 = 計測時刻 + 時刻ずれ（動作ごと）とし、時刻ずれと計測器の座標（rigid）はどの段階でも推定する。
    """

    TRAIN_PATTERNS = {"time_only": [(False, ())], **KinemaJointModel.TRAIN_PATTERNS}

    def __init__(self, trajectories: list[dict[str, Any]] = (), **settings: Any) -> None:
        super().__init__(**{**settings, "observation_model": "rigid"})
        self.trajectories = [(np.asarray(item["time"], dtype=np.float64), np.asarray(item["joints"], dtype=np.float64)) for item in trajectories]
        # 最適化では初期値からの補正量だけを動かす（絶対時刻でも数値微分の刻みが大きくならないように）
        self.time_base = np.zeros(len(self.trajectories), dtype=np.float64)
        self.time_offsets = np.zeros(len(self.trajectories), dtype=np.float64)

    def fit(self, X: Any, y: Any) -> "TrajectoryCalibModel":
        X, y = np.asarray(X, dtype=np.float64), np.asarray(y, dtype=np.float64).reshape(-1, 3)
        inside = self._initialize(self._parse(X), y)
        return super().fit(X[inside], y[inside])

    def save(self) -> dict[str, Any]:
        return {**super().save(), "time_offsets_ms": (self.time_offsets * 1000.0).tolist()}

    # キネマと伝達誤差だけを反映する（座標と時刻ずれはデータごとに fit で推定し直す）
    def load(self, parameters: dict[str, Any]) -> None:
        self._load_robot(parameters.get("robot_model", parameters))
        self.transmission.load(parameters.get("transmission_error", {}))

    # 計測器の座標変換がベースリンクの位置・姿勢（平坦化添字 0〜5）と重複するため、それらは推定しない
    def _set_stage(self, use_kinema: bool, joints: list[int] | tuple[int, ...]) -> None:
        super()._set_stage(use_kinema, joints)
        self.kinema.calibration_parameter_indices = tuple(index for index in self.kinema.calibration_parameter_indices if index >= 6)

    def _initialize(self, data: KinemaInput, measured: np.ndarray) -> np.ndarray:
        """時刻ずれと計測器の座標の初期値を決め、関節角軌道の範囲に入る計測点を返す。"""
        # 手先の速さは座標系によらないので、ロボット側と計測側の速さの時系列が最もよく重なるずらし量を時刻ずれの初期値にする
        for sequence, (time, joints) in enumerate(self.trajectories):
            robot_time = np.arange(time[0], time[-1], SPEED_GRID)
            robot_speed = self._speed(self._positions(np.column_stack([np.interp(robot_time, time, values) for values in joints.T])))
            indices = np.flatnonzero(data.sequence_ids == sequence)
            order = indices[np.argsort(data.times[indices])]
            measure_time = np.arange(data.times[order][0], data.times[order][-1], SPEED_GRID)
            measure_speed = self._speed(np.column_stack([np.interp(measure_time, data.times[order], values) for values in measured[order].T]))
            lag = self._best_lag(robot_speed, measure_speed)
            self.time_base[sequence] = robot_time[0] - measure_time[0] + lag * SPEED_GRID
        self.time_offsets = self.time_base.copy()

        # 補間が軌道の端で打ち切られる点は除き、残りの点の組から座標変換の初期値を決める
        shifted = data.times + self.time_offsets[data.sequence_ids]
        bounds = np.array([(time[0], time[-1]) for time, _ in self.trajectories])[data.sequence_ids]
        inside = (bounds[:, 0] <= shifted) & (shifted <= bounds[:, 1])
        self.observation.initialize(self._predict_positions(data)[inside], measured[inside])
        return inside

    # robot[n + lag] と measure[n] の重なる区間での二乗誤差が最小になる lag（重なりが計測の半分未満のずらし量は除く）
    @staticmethod
    def _best_lag(robot: np.ndarray, measure: np.ndarray) -> int:
        lags = np.arange(-len(measure) // 2, len(robot) - len(measure) // 2)
        errors = [np.mean((robot[max(lag, 0):lag + len(measure)] - measure[max(-lag, 0):len(robot) - lag]) ** 2) for lag in lags]
        return int(lags[np.argmin(errors)])

    @staticmethod
    def _speed(positions: np.ndarray) -> np.ndarray:
        return np.linalg.norm(np.gradient(positions, SPEED_GRID, axis=0), axis=1)

    # 関節角（指令）から伝達誤差・補正キネマ・工具オフセットを通したロボット座標の手先位置
    def _positions(self, joints: np.ndarray) -> np.ndarray:
        return apply_tool_offsets(self.kinema.predict(self.transmission.apply(joints)), self.tool_offsets, np.zeros(len(joints), dtype=np.int64))[:, :3]

    # 関節角は時刻ずれを加えた時刻で都度補間するため、X からは系列番号と時刻だけを取り出す
    def _parse(self, X: Any) -> KinemaInput:
        values = np.asarray(X, dtype=np.float64)
        return KinemaInput(np.empty((len(values), 0)), np.zeros(len(values), dtype=np.int64), values[:, 0].astype(np.int64), values[:, 1])

    def _predict_positions(self, data: KinemaInput) -> np.ndarray:
        joints = np.empty((len(data.times), 6), dtype=np.float64)
        for sequence, (time, trajectory) in enumerate(self.trajectories):
            indices = np.flatnonzero(data.sequence_ids == sequence)
            shifted = data.times[indices] + self.time_offsets[sequence]
            joints[indices] = np.column_stack([np.interp(shifted, time, values) for values in trajectory.T])
        return self._positions(joints)

    # 最適化ベクトルは [キネマ, 伝達誤差, 時刻ずれの補正量] の順（観測モデルの座標変換はその後ろ）
    def _get_robot_parameters(self, tool_parameter_indices: np.ndarray) -> np.ndarray:
        return np.hstack((super()._get_robot_parameters(tool_parameter_indices), self.time_offsets - self.time_base))

    def _set_robot_parameters(self, values: np.ndarray, tool_parameter_indices: np.ndarray) -> None:
        count = len(values) - len(self.time_offsets)
        super()._set_robot_parameters(values[:count], tool_parameter_indices)
        self.time_offsets = self.time_base + values[count:]

    def _parameter_map(self, values: np.ndarray, robot_count: int, tool_parameter_indices: np.ndarray) -> dict[str, float]:
        count = robot_count - len(self.time_offsets)
        result = super()._parameter_map(np.hstack((values[:count], values[robot_count:])), count, tool_parameter_indices)
        result.update({f"time_offset[{sequence}]": float(base + value) for sequence, (base, value) in enumerate(zip(self.time_base, values[count:robot_count]))})
        return result
