"""機種別の順運動学・たわみ補正パラメータを管理する。"""
import json
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

PARAMETER_DIR = Path(__file__).resolve().parent / "param"


@dataclass
class KinematicParameters:
    """リンク質量から得られる力学パラメータ。"""

    weights: np.ndarray
    moment_x: np.ndarray
    moment_y: np.ndarray
    moment_z: np.ndarray


@dataclass
class StiffnessParameters:
    """関節・アームの剛性パラメータ。"""

    joints: np.ndarray
    arm: np.ndarray
    base: float


class KinemaModelParam:
    """機種 JSON、可搬物、重力方向を順運動学用に保持する。"""

    def __init__(self, robot_type: str) -> None:
        self.robot_type = robot_type.upper()
        self.stiffness_rate = np.ones(9, dtype=np.float64)
        self.mass = 0.0
        self.mass_center = np.zeros(3, dtype=np.float64)
        self.gravity_direction = np.array([0.0, 0.0, -1.0], dtype=np.float64)
        self._load()

    def set_payload(self, mass: float = 0.0, mass_center: Any | None = None, gravity_direction: Any | None = None) -> None:
        """可搬物の質量・重心と重力方向を設定する（たわみ計算に使う）。"""
        self.mass = float(mass)
        self.mass_center = np.zeros(3, dtype=np.float64) if mass_center is None else np.asarray(mass_center, dtype=np.float64)
        if gravity_direction is not None:
            self.gravity_direction = np.asarray(gravity_direction, dtype=np.float64)

    def _load(self) -> None:
        # 機種 JSON はコメント付き（JSONC）なので、コメントを除いてから読む
        text = (PARAMETER_DIR / f"robotparam_{self.robot_type}.json").read_text(encoding="utf-8")
        data = json.loads(re.sub(r"/\*[\s\S]*?\*/|//.*", "", text))
        self.link_param = np.asarray(data["LinkParam"], dtype=np.float64)
        self.link_param2 = np.asarray(data["LinkParam2"], dtype=np.float64)

        # リンク長から公称 DH パラメータと、補正キネマ用の幾何パラメータ（各リンク XYZ + 回転 UVW）を組み立てる
        l1, l2, l3, l4, l5, l6 = self.link_param
        l1a, l5a, l6a = self.link_param2
        self.dh_param = np.array([
            [0, 0, l1, 90], [l2, 90, 0, 90], [l3, 0, 0, 0],
            [l4, 90, l5, 0], [0, -90, 0, 0], [0, 90, 0, 0], [0, 0, l6, 0],
        ], dtype=np.float64)
        self.geometry_param_ideal = np.array([
            [0, 0, l1a, 0, 0, 90], [l2, 0, l1 - l1a, 90, 0, 90],
            [l3, 0, 0, 0, 0, 0], [l4, -l5a, 0, 90, 0, 0],
            [0, 0, l5 - l5a, -90, 0, 0], [0, -l6a, 0, 90, 0, 0],
            [0, 0, l6 - l6a, 0, 0, 0],
        ], dtype=np.float64)
        self.geometry_param_corr = self.geometry_param_ideal.copy()

        # たわみ計算に使う質量・モーメントと剛性
        kp = data["KP"]
        self.kinematic = KinematicParameters(
            weights=np.asarray(kp["Wx"], dtype=np.float64),
            moment_x=np.asarray(kp["Mxx"], dtype=np.float64),
            moment_y=np.asarray(kp["Mxy"], dtype=np.float64),
            moment_z=np.asarray(kp["Mxz"], dtype=np.float64),
        )
        stiffness = data["StiffnessParam"]
        self.stiffness = StiffnessParameters(
            joints=np.asarray(stiffness["k"], dtype=np.float64),
            arm=np.asarray(stiffness["EI"], dtype=np.float64),
            base=float(stiffness["km1"]),
        )
