"""解析ページ：計測 CSV からキネマ・関節・ツールのキャリブレーションを行い、補正前後の精度を描画する。"""
import json

import numpy as np
from nicegui import ui

from . import api, joint_wave, loaders
from .layout import TAB10, download_json, file_upload, header, run_busy, show_png

# mode ごとに選べる calibration_mode（backend.model.kinema の IdealKinema / CorrectedKinema と対応）
CALIBRATION_MODES = {
    "actual": ["all_actual", "kinema_actual", "arm_actual", "origin_actual"],
    "ideal": ["local", "origin", "local_tool", "origin_tool", "local_ideal", "origin_ideal"],
}
# 同定パターン（kinema_only 以外は backend の KinemaJointModel.TRAIN_PATTERNS。伝達誤差は actual のみ）
TRAIN_PATTERNS = {
    "kinema_only": "キネマのみ", "trans_j1": "伝達誤差 J1", "trans_all": "伝達誤差 全軸",
    "kinema_trans_j1": "キネマ＋伝達誤差 J1（同時）", "kinema_trans_all": "キネマ＋伝達誤差 全軸（同時）",
    "kinema_then_trans_j1": "キネマ → 伝達誤差 J1（2 段階）", "kinema_then_trans_all": "キネマ → 伝達誤差 全軸（2 段階）",
}

# 軌跡キャリブの同定パターン（backend の TrajectoryCalibModel.TRAIN_PATTERNS。時刻ずれと計測器の座標はどのパターンでも推定する）
TRAJECTORY_PATTERNS = {"time_only": "時刻・座標のみ", **TRAIN_PATTERNS}


def payload_inputs():
    """可搬物・重力方向・工具オフセットの入力欄を置き、モデル設定の辞書を返す関数を返す（キネマ補正・軌跡キャリブで共用）。"""
    with ui.row().classes("items-center"):
        payload = [ui.number(label, value=0.0).classes("w-28") for label in ("可搬質量 [kg]", "重心 X [mm]", "重心 Y [mm]", "重心 Z [mm]")]
        gravity = [ui.number(f"重力方向 {axis}", value=value).classes("w-28") for axis, value in zip("XYZ", (0.0, 0.0, -1.0))]
    tool_offsets = ui.textarea("工具オフセット [mm]（1 行に 1 工具で x, y, z。ToolID=1 が 1 行目）", value="0, 0, 0").classes("w-full")
    return lambda: {
        "payload_mass": payload[0].value, "payload_center": [n.value for n in payload[1:]],
        "gravity_direction": [n.value for n in gravity],
        "tool_offsets": [[float(v) for v in line.split(",")] for line in tool_offsets.value.splitlines() if line.strip()],
    }


def transmission_table(transmission_error: dict):
    """軸・周期ごとの伝達誤差の振幅と位相を表で示す。"""
    ui.table(columns=[{"name": k, "label": label, "field": k} for k, label in (("joint", "軸"), ("period", "周期 [deg]"), ("amplitude", "振幅 [deg]"), ("offset", "位相 [deg]"))],
             rows=[{"joint": joint, "period": f"{p:.4f}", "amplitude": f"{a:.6f}", "offset": f"{o:.2f}"} for joint, wave in transmission_error.items() for p, a, o in zip(wave["periods"], wave["amplitudes"], wave["offsets"])])


def kinema_panel():
    with ui.row().classes("items-center"):
        robot_type = ui.select(["R6A", "R6B", "R6C", "R6D", "R6E"], value="R6E", label="機種")
        mode = ui.select(list(CALIBRATION_MODES), value="actual", label="mode", on_change=lambda e: (
            calibration_mode.set_options(CALIBRATION_MODES[e.value], value=CALIBRATION_MODES[e.value][0]),
            pattern.set_options(TRAIN_PATTERNS if e.value == "actual" else {"kinema_only": TRAIN_PATTERNS["kinema_only"]}, value="kinema_only")))
        calibration_mode = ui.select(CALIBRATION_MODES["actual"], value="all_actual", label="calibration_mode").classes("w-40")
        pattern = ui.select(TRAIN_PATTERNS, value="kinema_only", label="同定パターン").classes("w-72")
    robot_settings = payload_inputs()
    csv_files: dict[str, bytes] = {}
    param_files: dict[str, bytes] = {}
    file_upload(csv_files, "FARO 計測 CSV（J1〜J6, RobotXYZ, MeasureXYZ, ToolID。複数可）")

    def settings() -> dict:
        return {
            "robot_type": robot_type.value, "mode": mode.value, "calibration_mode": calibration_mode.value, "pattern": pattern.value, **robot_settings(),
        }

    # キネマのみは従来の kinema モデル、伝達誤差を含むパターンは kinema_joint モデルで学習する
    def init_body() -> dict:
        return {"model_type": "kinema" if pattern.value == "kinema_only" else "kinema_joint", "settings": settings()}

    # 補正前（計測 − 指令）と補正後（計測 − モデルの予測）の位置誤差を比べて、補正の効果を示す
    async def show_result():
        joints, references, positions, tool_ids = loaders.load_faro(list(csv_files.values()))
        X = np.column_stack((joints, tool_ids)).tolist()
        predicted = np.asarray((await api.analysis("/predict", {"X": X}))["predictions"])
        score = (await api.analysis("/evaluate", {"X_test": X, "y_test": positions.tolist()}))["score"]
        datasets = [loaders.pose_dataset_from_error("Before", TAB10[1], positions - references), loaders.pose_dataset_from_error("After", TAB10[0], positions - predicted)]
        show_png(result, await api.plot("pose_accuracy", "norm_with_bar", {"datasets": datasets, "title": "Kinematic calibration"}), "kinema_calibration.png")
        saved = await api.analysis("/save")
        with result:
            ui.label(f"R² = {score:.6f}")
            # 伝達誤差を含むモデルでは、軸・周期ごとの振幅と位相を示す
            if "transmission_error" in saved:
                transmission_table(saved["transmission_error"])

    async def train():
        joints, _, positions, tool_ids = loaders.load_faro(list(csv_files.values()))
        await api.analysis("/init", init_body())
        # 伝達誤差だけを同定するときなどに、保存済みのキネマパラメータから始められるようにする
        if start_from_loaded.value:
            await api.analysis("/load", json.loads(next(iter(param_files.values()))))
        await api.analysis("/train", {"X_train": np.column_stack((joints, tool_ids)).tolist(), "y_train": positions.tolist()})
        await show_result()

    # 保存済みパラメータで補正した場合の精度を、学習せずに確かめる
    async def load():
        await api.analysis("/init", init_body())
        await api.analysis("/load", json.loads(next(iter(param_files.values()))))
        await show_result()

    async def save():
        download_json(await api.analysis("/save"), f"kinema_{robot_type.value}.json")

    with ui.row():
        train_button = ui.button("学習", icon="model_training", on_click=lambda: run_busy(train_button, train))
        save_button = ui.button("パラメータを保存", icon="download", on_click=lambda: run_busy(save_button, save))
    with ui.expansion("保存済みパラメータで評価する").classes("w-full"):
        file_upload(param_files, "パラメータ JSON")
        load_button = ui.button("読み込んで評価", on_click=lambda: run_busy(load_button, load))
        start_from_loaded = ui.checkbox("このパラメータを学習の初期値にする（キネマを固定して伝達誤差だけ同定する場合など）")
    result = ui.column().classes("w-full")


def trajectory_panel():
    with ui.row().classes("items-center"):
        robot_type = ui.select(["R6A", "R6B", "R6C", "R6D", "R6E"], value="R6E", label="機種")
        calibration_mode = ui.select(CALIBRATION_MODES["actual"], value="all_actual", label="calibration_mode").classes("w-40")
        pattern = ui.select(TRAJECTORY_PATTERNS, value="kinema_only", label="同定パターン").classes("w-72")
        interval = ui.number("計測点の間引き間隔 [ms]", value=20, format="%d").classes("w-44")
    robot_settings = payload_inputs()
    files: dict[str, bytes] = {}
    param_files: dict[str, bytes] = {}
    file_upload(files, "FM と BT の CSV（*_FM.csv / *_BT.csv の組が 1 動作。複数動作なら複数組）")

    # 動作ごとの関節角軌道はモデルの設定として渡し、学習データは計測点の [動作番号, 計測時刻] と計測 XYZ にする
    def load_motions():
        motions = [(name, *loaders.load_trajectory_pair(fm, bt)) for name, fm, bt in loaders.named_pairs(files)]
        # 計測点は間引き間隔ごとに 1 点にして、学習時間を抑える
        picked = []
        for _, _, _, time, xyz in motions:
            step = max(1, round(interval.value / 1000.0 / np.median(np.diff(time))))
            picked.append((time[::step], xyz[::step]))
        X = np.vstack([np.column_stack((np.full(len(time), index), time)) for index, (time, _) in enumerate(picked)])
        trajectories = [{"time": fm_time.tolist(), "joints": joints.tolist()} for _, fm_time, joints, _, _ in motions]
        return [name for name, *_ in motions], trajectories, X.tolist(), np.vstack([xyz for _, xyz in picked])

    # 学習して、計測との位置誤差（計測 − 予測）を返す
    async def fit(trajectories: list, X: list, measured: np.ndarray, pattern_value: str, start_from_loaded: bool) -> np.ndarray:
        settings = {"robot_type": robot_type.value, "calibration_mode": calibration_mode.value, "pattern": pattern_value, "trajectories": trajectories, **robot_settings()}
        await api.analysis("/init", {"model_type": "trajectory", "settings": settings})
        if start_from_loaded:
            await api.analysis("/load", json.loads(next(iter(param_files.values()))))
        await api.analysis("/train", {"X_train": X, "y_train": measured.tolist()})
        return measured - np.asarray((await api.analysis("/predict", {"X": X}))["predictions"])

    async def train():
        names, trajectories, X, measured = load_motions()
        # Before は公称キネマで時刻と座標だけを合わせた誤差、After は選んだパターンで同定した誤差
        before = await fit(trajectories, X, measured, "time_only", False)
        after = await fit(trajectories, X, measured, pattern.value, start_from_loaded.value)
        score = (await api.analysis("/evaluate", {"X_test": X, "y_test": measured.tolist()}))["score"]
        saved = await api.analysis("/save")
        datasets = [loaders.pose_dataset_from_error("Before", TAB10[1], before), loaders.pose_dataset_from_error("After", TAB10[0], after)]
        show_png(result, await api.plot("pose_accuracy", "norm_with_bar", {"datasets": datasets, "title": "Trajectory calibration"}), "trajectory_calibration.png")
        with result:
            ui.label(f"R² = {score:.6f}（{len(X)} 点、{len(names)} 動作）")
            # 時刻ずれ（FM の時刻 = BT の時刻 + ずれ）を、BT の先頭が FM の何秒目に当たるかで示す
            X_array = np.asarray(X)
            starts = [X_array[X_array[:, 0] == index, 1].min() + offset / 1000.0 for index, offset in enumerate(saved["time_offsets_ms"])]
            ui.table(columns=[{"name": k, "label": label, "field": k} for k, label in (("motion", "動作"), ("start", "BT の先頭 → FM の時刻 [s]"))],
                     rows=[{"motion": name, "start": f"{start:.4f}"} for name, start in zip(names, starts)])
            transform = saved["observation_model"]["parameters"]["transform"]
            ui.label("計測器の座標（ロボット → 計測器）： " + ", ".join(f"{label} {value:.4f}" for label, value in zip(("X [mm]", "Y [mm]", "Z [mm]", "roll [deg]", "pitch [deg]", "yaw [deg]"), transform)))
            if any(any(wave["amplitudes"]) for wave in saved["transmission_error"].values()):
                transmission_table(saved["transmission_error"])

    async def save():
        download_json(await api.analysis("/save"), f"trajectory_{robot_type.value}.json")

    with ui.row():
        train_button = ui.button("学習", icon="model_training", on_click=lambda: run_busy(train_button, train))
        save_button = ui.button("パラメータを保存", icon="download", on_click=lambda: run_busy(save_button, save))
    with ui.expansion("保存済みパラメータを使う").classes("w-full"):
        file_upload(param_files, "パラメータ JSON（キネマ補正・軌跡キャリブで保存したもの）")
        start_from_loaded = ui.checkbox("このパラメータを学習の初期値にする（「時刻・座標のみ」と組み合わせると、別の日のデータで評価できる）")
    result = ui.column().classes("w-full")


def joint_panel():
    with ui.row().classes("items-center"):
        joint_no = ui.select([1, 2, 3, 4, 5, 6], value=1, label="軸", on_change=lambda e: gear_rate.set_value(joint_wave.GEAR_RATES[e.value - 1]))
        gear_rate = ui.number("減速比", value=joint_wave.GEAR_RATES[0])
        maxfev = ui.number("maxfev", value=10_000, format="%d")
    files: dict[str, bytes] = {}
    file_upload(files, "FM と BT の CSV（*_FM.csv / *_BT.csv の組。複数区間なら複数組）")

    async def train():
        angle, error = joint_wave.prepare(loaders.pair_files(files), joint_no.value)
        X = angle.reshape(-1, 1).tolist()
        await api.analysis("/init", {"model_type": "joint_calib", "settings": {"periods": joint_wave.wave_periods(gear_rate.value).tolist(), "maxfev": int(maxfev.value)}})
        await api.analysis("/train", {"X_train": X, "y_train": error.reshape(-1, 1).tolist()})
        corrected = error - np.asarray((await api.analysis("/predict", {"X": X}))["predictions"])

        # 補正前後の誤差を、描画ページと同じく 0.01° 刻みに間引いて描画する
        datasets = [{"name": name, "color": color, "data": {"angle": angle[::10].tolist(), "error": values[::10].tolist()}} for name, color, values in (("Before", TAB10[1], error), ("After", TAB10[0], corrected))]
        body = {"datasets": datasets, "joint_no": joint_no.value, "gear_rate": gear_rate.value, "title": "Joint calibration"}
        show_png(result, await api.plot("single_axes", "angle_error", body), f"joint_calibration_J{joint_no.value}.png")
        saved = await api.analysis("/save")
        with result:
            ui.table(columns=[{"name": k, "label": label, "field": k} for k, label in (("period", "周期 [deg]"), ("amplitude", "振幅 [deg]"), ("offset", "位相 [deg]"))],
                     rows=[{"period": f"{p:.4f}", "amplitude": f"{a:.6f}", "offset": f"{o:.2f}"} for p, a, o in zip(saved["periods"], saved["amplitudes"], saved["offsets"])])

    async def save():
        download_json(await api.analysis("/save"), f"joint_calib_J{joint_no.value}.json")

    with ui.row():
        train_button = ui.button("学習", icon="model_training", on_click=lambda: run_busy(train_button, train))
        save_button = ui.button("パラメータを保存", icon="download", on_click=lambda: run_busy(save_button, save))
    result = ui.column().classes("w-full")


def tool_panel():
    files: dict[str, bytes] = {}
    file_upload(files, "計測 CSV（RobotXYZUVW, MeasureXYZ, ToolID。先頭行が基準点の相対座標）")

    async def train():
        xyzuvw, tool_ids, measured = loaders.load_toolcalib_csv(next(iter(files.values())))
        # 全行を 1 系列とし、先頭行（時刻 0）からの相対位置として学習する
        X = np.column_stack((xyzuvw, tool_ids, np.zeros(len(xyzuvw)), np.arange(len(xyzuvw), dtype=np.float64))).tolist()
        if np.unique(xyzuvw[:, 3:], axis=0).shape[0] < 2:
            ui.notify("RobotU/V/W が一定です。相対観測では工具オフセットを同定できないため、姿勢を変えたデータを使ってください。", type="warning", multi_line=True)
        await api.analysis("/init", {"model_type": "tool_calib", "settings": {"tool_offsets": np.zeros((int(tool_ids.max()), 3)).tolist(), "observation_model": "relative"}})
        rmse = lambda predicted: float(np.sqrt(np.mean((np.asarray(predicted) - measured) ** 2)))
        before = rmse((await api.analysis("/predict", {"X": X}))["predictions"])
        await api.analysis("/train", {"X_train": X, "y_train": measured.tolist()})
        after = rmse((await api.analysis("/predict", {"X": X}))["predictions"])
        offsets = (await api.analysis("/save"))["ToolOffsets"]

        result.clear()
        with result:
            ui.label(f"相対 RMSE [mm]: 補正前 {before:.6f} → 補正後 {after:.6f}（{len(xyzuvw)} 点、{len(offsets)} 工具）")
            ui.table(columns=[{"name": k, "label": k, "field": k} for k in ("ToolID", "X [mm]", "Y [mm]", "Z [mm]")],
                     rows=[{"ToolID": i + 1, **{f"{axis} [mm]": f"{v:.4f}" for axis, v in zip("XYZ", offset)}} for i, offset in enumerate(offsets)])

    async def save():
        download_json(await api.analysis("/save"), "tool_calib.json")

    with ui.row():
        train_button = ui.button("学習", icon="model_training", on_click=lambda: run_busy(train_button, train))
        save_button = ui.button("パラメータを保存", icon="download", on_click=lambda: run_busy(save_button, save))
    result = ui.column().classes("w-full")


@ui.page("/analysis")
def page():
    header()
    ui.label("解析 API は学習済みモデルを 1 つだけ保持します。別のタブで学習すると、それまでのモデルは置き換わります。").classes("text-sm text-gray-500")
    with ui.tabs() as tabs:
        kinema, trajectory, joint, tool = ui.tab("キネマ補正"), ui.tab("軌跡キャリブ"), ui.tab("関節補正"), ui.tab("ツール補正")
    with ui.tab_panels(tabs, value=kinema).classes("w-full"):
        with ui.tab_panel(kinema):
            kinema_panel()
        with ui.tab_panel(trajectory):
            trajectory_panel()
        with ui.tab_panel(joint):
            joint_panel()
        with ui.tab_panel(tool):
            tool_panel()
