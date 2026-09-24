"""計測 CSV（アップロードされたバイト列）を API の入力形式へ変換する（reference の DrawClient / AnalysisClient / sample_toolcalib から移植）。"""
import io

import numpy as np
import pandas as pd


def read_csv(data: bytes, **kwargs) -> pd.DataFrame:
    return pd.read_csv(io.BytesIO(data), **kwargs)


# ファイル名の末尾 _FM.csv（ロボット側）と _BT.csv（FARO 側）で組を作る。組は名前順に並べる
def pair_files(files: dict[str, bytes]) -> list[tuple[bytes, bytes]]:
    by_upper = {name.upper(): data for name, data in files.items()}
    return [(by_upper[name], by_upper[name[:-7] + "_BT.CSV"]) for name in sorted(by_upper) if name.endswith("_FM.CSV") and name[:-7] + "_BT.CSV" in by_upper]


# 計測座標をロボット座標へ最もよく重なるよう剛体変換する（Kabsch 法）。計測器の設置位置の違いを除いて比較するため
def fit_local(pos_base: np.ndarray, pos_exp: np.ndarray) -> np.ndarray:
    pos_base, pos_exp = np.asarray(pos_base, dtype=np.float64), np.asarray(pos_exp, dtype=np.float64)
    base_center, exp_center = pos_base.mean(axis=0), pos_exp.mean(axis=0)
    u, _, vt = np.linalg.svd((pos_exp - exp_center).T @ (pos_base - base_center))
    rotation = vt.T @ u.T
    # 鏡映にならないよう、行列式が負なら最小特異値の軸を反転する
    if np.linalg.det(rotation) < 0.0:
        vt[-1, :] *= -1.0
        rotation = vt.T @ u.T
    return pos_exp @ rotation.T + (base_center - rotation @ exp_center)


def pose_dataset_from_error(name: str, color: str, error: np.ndarray) -> dict:
    """``(N, 3)`` の誤差から姿勢精度 API の系列を作る。"""
    error = np.asarray(error, dtype=float)
    return {"name": name, "color": color, "data": {"dx": error[:, 0].tolist(), "dy": error[:, 1].tolist(), "dz": error[:, 2].tolist()}}


def pose_dataset(name: str, color: str, data: bytes, fitlocal: bool = False) -> dict:
    """Robot XYZ / Measure XYZ の CSV から、計測 − 指令 の誤差系列を作る。"""
    frame = read_csv(data, usecols=[f"{prefix}{axis}" for prefix in ("Robot", "Measure") for axis in "XYZ"]).dropna()
    measured = frame[[f"Measure{axis}" for axis in "XYZ"]].to_numpy()
    robot = frame[[f"Robot{axis}" for axis in "XYZ"]].to_numpy()
    return pose_dataset_from_error(name, color, fit_local(robot, measured) - robot if fitlocal else measured - robot)


def path_dataset(name: str, color: str, measured: bytes, reference: bytes) -> dict:
    """BT（FARO の計測軌跡）と FM（ロボットの指令軌跡）の CSV から軌跡精度 API の系列を作る。"""
    # BT は 1 行目、FM は 2 行目までがヘッダー前の情報。FM の末尾 2 行は集計行
    measured_csv = read_csv(measured, skiprows=1, encoding="shift-jis", low_memory=False)
    reference_csv = read_csv(reference, skiprows=2, encoding="shift-jis", low_memory=False).iloc[:-2]
    return {
        "name": name,
        "color": color,
        "pos_measured": measured_csv[["#X(mm)", "Y(mm)", "Z(mm)"]].to_numpy().tolist(),
        "pos_reference": reference_csv[["RefPos(X)[mm]", "RefPos(Y)[mm]", "RefPos(Z)[mm]"]].to_numpy().tolist(),
    }


def load_faro(files: list[bytes]) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """キネマ補正用の FARO 計測 CSV を結合し、関節角・指令位置・計測位置・工具番号を返す。"""
    frames = [read_csv(data, low_memory=False) for data in files]
    joints = np.vstack([frame[[f"J{index}" for index in range(1, 7)]].to_numpy() for frame in frames])
    references = np.vstack([frame[[f"Robot{axis}" for axis in "XYZ"]].to_numpy() for frame in frames])
    positions = np.vstack([frame[[f"Measure{axis}" for axis in "XYZ"]].to_numpy() for frame in frames])
    tool_ids = np.hstack([frame["ToolID"].to_numpy() for frame in frames])
    return joints, references, positions, tool_ids


def load_toolcalib_csv(data: bytes) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """ツール補正用の CSV（RobotXYZUVW、MeasureXYZ、ToolID）を読み、姿勢・工具番号・計測位置を返す。"""
    # 計測ソフトによって UTF-8（BOM 付き）と Shift_JIS のどちらでも出力されるため、順に試す
    try:
        text = data.decode("utf-8-sig")
    except UnicodeDecodeError:
        text = data.decode("cp932")
    frame = pd.read_csv(io.StringIO(text), float_precision="round_trip")
    xyzuvw = frame[[f"Robot{axis}" for axis in "XYZUVW"]].to_numpy(dtype=np.float64)
    tool_ids = frame["ToolID"].to_numpy(dtype=np.float64).astype(int).astype(np.float64)
    measured = frame[[f"Measure{axis}" for axis in "XYZ"]].to_numpy(dtype=np.float64)
    return xyzuvw, tool_ids, measured
