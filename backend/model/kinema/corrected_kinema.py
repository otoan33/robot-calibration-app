"""重力たわみを補正した 6 軸ロボットの順運動学。"""
from typing import Any

import numpy as np

from ..util.Rotation import TransRotToEulerZYX
from .kinema_parameter import KinemaModelParam

GRAVITY = np.float64(9.80665)


class CorrectedKinema:
    """重力荷重に起因する関節・アームのたわみを補正して姿勢を計算する。"""

    # 旧 KinemaCalib の各 TRAINMODE で補正する幾何パラメータ（7x6 を平坦化した添字）
    CALIBRATION_INDICES_BY_MODE = {
        "none": (),
        "origin_actual": (0, 1, 2, 3, 4, 5, 11, 17, 23, 29, 35),
        "arm_actual": (0, 1, 2, 3, 4, 5, 11, 17, 23, 29, 35, 6, 12, 18, 19, 31),
        "kinema_actual": (0, 1, 2, 3, 4, 5, 11, 17, 23, 29, 35, 6, 12, 18, 19, 31, 7, 24, 25, 30, 9, 15, 16, 21, 27, 33, 36),
        "all_actual": (0, 1, 2, 3, 4, 5, 11, 17, 23, 29, 35, 6, 12, 18, 19, 31, 7, 24, 25, 30, 9, 15, 16, 21, 27, 33, 36),
    }
    # ALL_ACTUAL で追加で推定する剛性率の添字
    ALL_ACTUAL_STIFFNESS_INDICES = (1, 2, 3, 4)

    def __init__(self, params: KinemaModelParam, calibration_mode: str = "all_actual", max_nfev: int | None = None) -> None:
        self.params = params
        self.set_calibration_mode(calibration_mode)
        self.max_nfev = max_nfev
        self.calibration_result_: dict[str, Any] | None = None

    # 段階的な同定で、推定対象の幾何パラメータ・剛性率を切り替える（"none" はキネマを固定する）
    def set_calibration_mode(self, calibration_mode: str) -> None:
        self.calibration_mode = calibration_mode.lower()
        self.calibration_parameter_indices = self.CALIBRATION_INDICES_BY_MODE[self.calibration_mode]
        self.calibration_stiffness_indices = self.ALL_ACTUAL_STIFFNESS_INDICES if self.calibration_mode == "all_actual" else ()

    def predict(self, joints: np.ndarray) -> np.ndarray:
        """関節角 ``(N, 6)`` [deg] から補正済み手先姿勢を返す（全点をまとめて計算する）。"""
        joints = np.asarray(joints, dtype=np.float64)
        delta1, delta2, delta3, delta4, delta5, delta6, phi2, w2, phi4, w4, psi1 = self.calc_deform(joints)

        # 関節角（J4, J6 は向きが逆）を W 回転に加え、たわみによる角度・変位を足し込む
        parameters = np.repeat(self.params.geometry_param_corr[None], len(joints), axis=0)
        parameters[:, :6, 5] += joints * np.array([1, 1, 1, -1, 1, -1], dtype=np.float64)
        parameters[:, :, 3:] = np.radians(parameters[:, :, 3:])
        parameters[:, :6, 5] += np.column_stack((delta1, delta2, delta3 + phi2, delta4, delta5, delta6))
        parameters[:, 2, 1] += w2
        parameters[:, 3, 0] += w4
        parameters[:, 3, 4] += phi4

        # 各リンクの XYZ 並進 + UVW 回転を順に掛ける（ベースの倒れ psi1 は第 1 リンクの後に挟む）
        transform = np.eye(4, dtype=np.float64)
        for index in range(7):
            x, y, z, u, v, w = parameters[:, index].T
            cu, su, cv, sv, cw, sw = np.cos(u), np.sin(u), np.cos(v), np.sin(v), np.cos(w), np.sin(w)
            transform = transform @ _matrices([
                [cv * cw, -cv * sw, sv, x],
                [su * sv * cw + cu * sw, -su * sv * sw + cu * cw, -su * cv, y],
                [-cu * sv * cw + su * sw, cu * sv * sw + su * cw, cu * cv, z],
                [0, 0, 0, 1],
            ])
            if index == 0:
                cosine, sine = np.cos(psi1), np.sin(psi1)
                transform = transform @ _matrices([[cosine, 0, sine, 0], [0, 1, 0, 0], [-sine, 0, cosine, 0], [0, 0, 0, 1]])
        return np.hstack((transform[:, :3, 3], TransRotToEulerZYX(transform[:, :3, :3])))

    def forward(self, joint: np.ndarray) -> np.ndarray:
        """1 組の関節角から手先姿勢を計算する。"""
        return self.predict(np.asarray(joint, dtype=np.float64)[None])[0]

    # 剛性は公称値との比（stiffness_rate）で推定しているので、保存時は実際の剛性値に戻す
    def _stiffness_reference(self) -> np.ndarray:
        return np.concatenate((self.params.stiffness.joints, self.params.stiffness.arm, [self.params.stiffness.base]))

    def save_parameters(self) -> dict[str, list]:
        """補正後のキネマパラメータ（公称値からの幾何誤差と剛性）を JSON 化可能な辞書で返す。"""
        return {
            "GeomErr": (self.params.geometry_param_corr - self.params.geometry_param_ideal).tolist(),
            "StiffnessParam": (self._stiffness_reference() / self.params.stiffness_rate).tolist(),
        }

    def load_parameters(self, parameters: dict[str, Any]) -> None:
        """保存済みの補正後キネマパラメータを反映する。"""
        self.params.geometry_param_corr[:] = self.params.geometry_param_ideal + np.asarray(parameters["GeomErr"], dtype=np.float64)
        self.params.stiffness_rate[:] = self._stiffness_reference() / np.asarray(parameters["StiffnessParam"], dtype=np.float64)

    def calc_deform(self, joints: np.ndarray) -> tuple[np.ndarray, ...]:
        """重力荷重に起因する関節角・アームのたわみを、関節角 ``(N, 6)`` の各点について算出する。"""
        # リンク長・重心位置 [m] と各リンクの質量・重心距離
        l1, l2, l3, l4, l5, l6 = self.params.link_param / 1000.0
        l1a, _, _ = self.params.link_param2 / 1000.0
        lgtx, lgty, lgtz = self.params.mass_center / 1000.0
        weight, mx, my, mz = self.params.kinematic.weights, self.params.kinematic.moment_x, self.params.kinematic.moment_y, self.params.kinematic.moment_z
        m1, m2, m3, m5 = weight[0], weight[1], weight[2] + weight[3], weight[4] + weight[5]
        lg1 = (l1 - l1a) + mz[0] / weight[0]
        lg2, lg3, lg4 = mx[0] / weight[0], mx[1] / weight[1], -my[1] / weight[1]
        lg5 = (mx[2] + mx[3]) / (weight[2] + weight[3])
        lg6 = (-my[2] + mz[3] + l5 * weight[3]) / (weight[2] + weight[3])
        lg7 = (-my[4] + mz[5] + l6 * weight[5]) / (weight[4] + weight[5])

        th1, th2, th3, th4, th5, th6 = np.radians(joints).T
        c1, s1, c2, s2 = np.cos(th1), np.sin(th1), np.cos(th2), np.sin(th2)
        c23, s23, c4, s4, c5, s5, c6, s6 = np.cos(th2 + th3), np.sin(th2 + th3), np.cos(th4), np.sin(th4), np.cos(th5), np.sin(th5), np.cos(th6), np.sin(th6)
        rotation5 = _matrices([
            [c4, -s4 * s5, -s4 * c5],
            [-s23 * s4, c23 * c5 - s23 * c4 * s5, -c23 * s5 - s23 * c4 * c5],
            [c23 * s4, s23 * c5 + c23 * c4 * s5, -s23 * s5 + c23 * c4 * c5],
        ])

        # 手先側から順に、各関節まわりの重力モーメントを積み上げる
        g = GRAVITY * self.params.gravity_direction
        gravity = _vectors(g[0] * c1 + g[1] * s1, -g[0] * s1 + g[1] * c1, g[2])
        mass = self.params.mass
        moment5_vector = np.array([0, m5 * lg7, 0]) + mass * _vectors(lgty * c6 - lgtx * s6, l6 + lgtz, lgty * s6 + lgtx * c6)
        moment5 = np.cross(np.einsum("nij,nj->ni", rotation5, moment5_vector), gravity)
        moment3_vector = np.array([0, m3 * lg6 + m5 * l5, m3 * lg5 + m5 * l4]) + mass * np.array([0, l5, l4])
        moment3 = moment5 + np.cross(_vectors(0, c23 * moment3_vector[1] - s23 * moment3_vector[2], s23 * moment3_vector[1] + c23 * moment3_vector[2]), gravity)
        moment2_vector = np.array([0, m2 * lg4, m2 * lg3 + (m3 + m5) * l3]) + mass * np.array([0, 0, l3])
        moment2 = moment3 + np.cross(_vectors(0, c2 * moment2_vector[1] - s2 * moment2_vector[2], s2 * moment2_vector[1] + c2 * moment2_vector[2]), gravity)
        moment1_vector = np.array([0, m1 * lg2 + (m2 + m3 + m5) * l2, m1 * lg1 + (m2 + m3 + m5) * (l1 - l1a)]) + mass * np.array([0, l2, l1 - l1a])
        moment1 = moment2 + np.cross(moment1_vector, gravity)

        # 関節のねじれ角 = 関節軸まわりのモーメント / 関節剛性
        joint_moments = np.column_stack((
            moment1[:, 2], moment2[:, 0], moment3[:, 0], moment5[:, 1] * c23 + moment5[:, 2] * s23,
            np.einsum("ni,ni->n", moment5, rotation5[:, :, 0]), np.einsum("ni,ni->n", moment5, rotation5[:, :, 1]),
        ))
        deltas = joint_moments / (self.params.stiffness.joints * 1e4) * self.params.stiffness_rate[:6]

        # 第 2・第 3 アームを片持ち梁として、先端のたわみ角 phi とたわみ量 w を求める
        moment2_arm, moment3_arm = moment3[:, 0], moment5[:, 0]
        weight2_arm = (m3 + m5 + mass) * (gravity[:, 1] * c2 + gravity[:, 2] * s2)
        weight3_arm = (m5 + mass) * (gravity[:, 1] * s23 - gravity[:, 2] * c23)
        ei2 = self.params.stiffness.arm[0] * 1e4 / self.params.stiffness_rate[6]
        ei3 = self.params.stiffness.arm[1] * 1e4 / self.params.stiffness_rate[7]
        phi2 = (2 * moment2_arm * l3 - weight2_arm * l3 ** 2) / (2 * ei2)
        w2 = (3 * moment2_arm * l3 ** 2 - 2 * weight2_arm * l3 ** 3) / (6 * ei2) * 1000
        phi4 = (2 * moment3_arm * l5 - weight3_arm * l5 ** 2) / (2 * ei3)
        w4 = (3 * moment3_arm * l5 ** 2 - 2 * weight3_arm * l5 ** 3) / (6 * ei3) * 1000

        # ベースの倒れ
        psi1 = -moment1[:, 0] / (self.params.stiffness.base * 1e4) * self.params.stiffness_rate[8]
        return (*deltas.T, phi2, w2, phi4, w4, psi1)

    # 最適化で動かすパラメータ（補正対象の幾何パラメータ + 剛性率）を 1 本のベクトルにする
    def _get_calibration_parameters(self) -> np.ndarray:
        geometry = np.array([self.params.geometry_param_corr[index // 6, index % 6] for index in self.calibration_parameter_indices], dtype=np.float64)
        return np.hstack((geometry, self.params.stiffness_rate[list(self.calibration_stiffness_indices)]))

    def _set_calibration_parameters(self, values: np.ndarray) -> None:
        geometry_count = len(self.calibration_parameter_indices)
        for index, value in zip(self.calibration_parameter_indices, values[:geometry_count], strict=True):
            self.params.geometry_param_corr[index // 6, index % 6] = value
        self.params.stiffness_rate[list(self.calibration_stiffness_indices)] = values[geometry_count:]

    # 推定結果を人が読める名前付きの辞書にする
    def _calibration_parameter_map(self, values: np.ndarray) -> dict[str, float]:
        geometry_count = len(self.calibration_parameter_indices)
        parameters = {f"geometry[{index // 6},{index % 6}]": float(value) for index, value in zip(self.calibration_parameter_indices, values[:geometry_count], strict=True)}
        parameters.update({f"stiffness_rate[{index}]": float(value) for index, value in zip(self.calibration_stiffness_indices, values[geometry_count:], strict=True)})
        return parameters


# 点ごとの値（配列）と定数が混ざった成分表から、(N, 行, 列) の行列を作る
def _matrices(rows: list[list[Any]]) -> np.ndarray:
    entries = np.broadcast_arrays(*(value for row in rows for value in row))
    return np.stack(entries, axis=-1).reshape(*entries[0].shape, len(rows), len(rows[0]))


def _vectors(*components: Any) -> np.ndarray:
    return np.stack(np.broadcast_arrays(*components), axis=-1)
