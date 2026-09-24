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


def kinema_panel():
    with ui.row().classes("items-center"):
        robot_type = ui.select(["R6A", "R6B", "R6C", "R6D", "R6E"], value="R6E", label="機種")
        mode = ui.select(list(CALIBRATION_MODES), value="actual", label="mode", on_change=lambda e: calibration_mode.set_options(CALIBRATION_MODES[e.value], value=CALIBRATION_MODES[e.value][0]))
        calibration_mode = ui.select(CALIBRATION_MODES["actual"], value="all_actual", label="calibration_mode").classes("w-40")
    with ui.row().classes("items-center"):
        payload = [ui.number(label, value=0.0).classes("w-28") for label in ("可搬質量 [kg]", "重心 X [mm]", "重心 Y [mm]", "重心 Z [mm]")]
        gravity = [ui.number(f"重力方向 {axis}", value=value).classes("w-28") for axis, value in zip("XYZ", (0.0, 0.0, -1.0))]
    tool_offsets = ui.textarea("工具オフセット [mm]（1 行に 1 工具で x, y, z。ToolID=1 が 1 行目）", value="0, 0, 0").classes("w-full")
    csv_files: dict[str, bytes] = {}
    param_files: dict[str, bytes] = {}
    file_upload(csv_files, "FARO 計測 CSV（J1〜J6, RobotXYZ, MeasureXYZ, ToolID。複数可）")

    def settings() -> dict:
        return {
            "robot_type": robot_type.value, "mode": mode.value, "calibration_mode": calibration_mode.value,
            "payload_mass": payload[0].value, "payload_center": [n.value for n in payload[1:]],
            "gravity_direction": [n.value for n in gravity],
            "tool_offsets": [[float(v) for v in line.split(",")] for line in tool_offsets.value.splitlines() if line.strip()],
        }

    # 補正前（計測 − 指令）と補正後（計測 − モデルの予測）の位置誤差を比べて、補正の効果を示す
    async def show_result():
        joints, references, positions, tool_ids = loaders.load_faro(list(csv_files.values()))
        X = np.column_stack((joints, tool_ids)).tolist()
        predicted = np.asarray((await api.analysis("/predict", {"X": X}))["predictions"])
        score = (await api.analysis("/evaluate", {"X_test": X, "y_test": positions.tolist()}))["score"]
        datasets = [loaders.pose_dataset_from_error("Before", TAB10[1], positions - references), loaders.pose_dataset_from_error("After", TAB10[0], positions - predicted)]
        show_png(result, await api.plot("pose_accuracy", "norm_with_bar", {"datasets": datasets, "title": "Kinematic calibration"}), "kinema_calibration.png")
        with result:
            ui.label(f"R² = {score:.6f}")

    async def train():
        joints, _, positions, tool_ids = loaders.load_faro(list(csv_files.values()))
        await api.analysis("/init", {"model_type": "kinema", "settings": settings()})
        await api.analysis("/train", {"X_train": np.column_stack((joints, tool_ids)).tolist(), "y_train": positions.tolist()})
        await show_result()

    # 保存済みパラメータで補正した場合の精度を、学習せずに確かめる
    async def load():
        await api.analysis("/init", {"model_type": "kinema", "settings": settings()})
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
        kinema, joint, tool = ui.tab("キネマ補正"), ui.tab("関節補正"), ui.tab("ツール補正")
    with ui.tab_panels(tabs, value=kinema).classes("w-full"):
        with ui.tab_panel(kinema):
            kinema_panel()
        with ui.tab_panel(joint):
            joint_panel()
        with ui.tab_panel(tool):
            tool_panel()
