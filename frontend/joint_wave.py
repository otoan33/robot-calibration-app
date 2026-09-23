"""単軸計測の FM/BT の組から、関節角ごとの周期的な角度伝達誤差を求める（reference の joint_calib_data.py から移植）。

FM はロボットの関節角、BT は FARO で計測した手先の円弧軌跡。単軸の描画と関節補正の両方で使う。
"""
import numpy as np
from scipy.interpolate import interp1d

from .loaders import read_csv

# 各軸（J1〜J6）の減速比の既定値
GEAR_RATES = (120, 120, 120, 50, 80, 50)


# 3 次元の点群に円を当てはめ、平面の法線・中心・半径・平面内の基底を返す
def _circle_parameters(points: np.ndarray):
    center_plane = points.mean(axis=0)
    _, _, vh = np.linalg.svd(points - center_plane, full_matrices=False)
    normal, basis_x = vh[-1], vh[0]
    basis_y = np.cross(normal, basis_x)
    basis_y /= np.linalg.norm(basis_y)
    xy = np.column_stack(((points - center_plane) @ basis_x, (points - center_plane) @ basis_y))

    # 代数的な最小二乗円: x^2 + y^2 + Ax + By + C = 0
    a, b, c = np.linalg.lstsq(np.column_stack((xy[:, 0], xy[:, 1], np.ones(len(xy)))), -(xy[:, 0] ** 2 + xy[:, 1] ** 2), rcond=None)[0]
    circle_xy = np.array([-a / 2.0, -b / 2.0])
    center = center_plane + circle_xy[0] * basis_x + circle_xy[1] * basis_y
    return normal, center, float(np.sqrt(np.dot(circle_xy, circle_xy) - c)), basis_x, basis_y


# FARO の円弧上の点を、始点を start_angle とする回転角 [deg] に変換する
def _faro_angle(points: np.ndarray, start_angle: float) -> np.ndarray:
    _, center, _, basis_x, basis_y = _circle_parameters(points)
    relative = points - center
    angle = np.unwrap(np.arctan2(relative @ basis_y, relative @ basis_x))
    if angle[-1] < angle[0]:
        angle *= -1
    return np.degrees(angle - angle[0]) + start_angle


def _load_pair(fm_data: bytes, bt_data: bytes, joint_no: int) -> tuple[np.ndarray, np.ndarray]:
    fm = read_csv(fm_data, skiprows=2, encoding="shift-jis", low_memory=False).iloc[:-2]
    time_fm = fm.iloc[:, 0].to_numpy(dtype=np.float64) / 1000.0
    angle_fm = fm[f"Joint(J{joint_no})[deg]"].to_numpy(dtype=np.float64)
    direction = np.sign(angle_fm[-1] - angle_fm[0])

    # FARO の回転角は、回転の向きを FM に合わせる
    bt = read_csv(bt_data, skiprows=1, encoding="shift-jis", low_memory=False)
    time_bt = bt["TIMESTAMP"].to_numpy(dtype=np.float64) / 1000.0
    angle_bt = _faro_angle(bt[["#X(mm)", "Y(mm)", "Z(mm)"]].to_numpy(dtype=np.float64), angle_fm[0])
    angle_bt = angle_fm[0] + direction * (angle_bt - angle_bt[0])

    # 2 つの計測は時計が別なので、最高速の半分を超えて動いている区間の中央を時刻 0 として揃える
    speed_bt = np.abs(np.gradient(angle_bt, time_bt))
    speed_fm = np.abs(np.gradient(angle_fm, time_fm))
    threshold = np.max(speed_fm) / 2.0
    bt_active, fm_active = np.flatnonzero(speed_bt > threshold), np.flatnonzero(speed_fm > threshold)
    time_bt -= (time_bt[bt_active[0]] + time_bt[bt_active[-1]]) / 2.0
    time_fm -= (time_fm[fm_active[0]] + time_fm[fm_active[-1]]) / 2.0

    # 共通の時刻 (1 ms 刻み) で補間し、関節角ごとの誤差 (FARO − FM) を 0.001° 刻みで求める（両端 2° は加減速域として除く）
    time = np.arange(max(time_fm[0], time_bt[0]), min(time_fm[-1], time_bt[-1]), 0.001)
    fm_interp = interp1d(time_fm, angle_fm, bounds_error=True)(time)
    bt_interp = interp1d(time_bt, angle_bt, bounds_error=True)(time)
    angles = np.arange(min(fm_interp[0], fm_interp[-1]) + 2.0, max(fm_interp[0], fm_interp[-1]) - 2.0, 0.001)
    errors = interp1d(fm_interp, bt_interp - fm_interp, bounds_error=True)(angles)

    # ゆっくりした傾向（5 次多項式）を除き、周期的な伝達誤差だけを残す
    design = np.column_stack([angles ** power for power in range(6)])
    return angles, errors - design @ np.linalg.lstsq(design, errors, rcond=None)[0]


# 複数区間の計測を、隣り合う区間の境目（重なりの中央）で切り替えながら 1 本につなぐ
def _concat_spans(spans: list[tuple[np.ndarray, np.ndarray]]) -> tuple[np.ndarray, np.ndarray]:
    if len(spans) == 1:
        return spans[0]
    boundaries = [-np.inf] + [(previous[0][-1] + current[0][0]) / 2.0 for previous, current in zip(spans, spans[1:])] + [np.inf]
    masks = [(angles > boundaries[index]) & (angles < boundaries[index + 1]) for index, (angles, _) in enumerate(spans)]
    angles = np.hstack([span[0][mask] for span, mask in zip(spans, masks)])
    errors = np.hstack([span[1][mask] for span, mask in zip(spans, masks)])
    order = np.argsort(angles)
    grid = np.arange(angles[order][0], angles[order][-1], 0.001)
    return grid, interp1d(angles[order], errors[order], bounds_error=True)(grid)


def prepare(pairs: list[tuple[bytes, bytes]], joint_no: int) -> tuple[np.ndarray, np.ndarray]:
    """FM/BT の組から ``(関節角, 周期誤差)`` を 0.001° 刻みで返す。"""
    return _concat_spans([_load_pair(fm, bt, int(joint_no)) for fm, bt in pairs])


def wave_periods(gear_rate: float) -> np.ndarray:
    """減速比から、補正する 2 つの周期 [deg]（1 次と 2 次）を返す。"""
    return np.array([180.0 / gear_rate, 90.0 / gear_rate], dtype=np.float64)
