"""描画ページ：計測 CSV から姿勢精度・軌跡精度・単軸の角度伝達誤差のグラフを作る。"""
from nicegui import ui

from . import api, joint_wave, loaders
from .layout import TAB10, file_upload, header, run_busy, show_png


def series_editor(hint: str) -> list[dict]:
    """比較する系列（名前・色・ファイル）を編集する欄。返すリストは画面の操作に合わせて増減する。"""
    series: list[dict] = []
    container = ui.column().classes("w-full")

    def add(name: str, color: str):
        item = {"name": name, "color": color, "files": {}}
        with container, ui.card().classes("w-full") as card:
            with ui.row().classes("items-center w-full"):
                ui.input("名前").bind_value(item, "name")
                ui.color_input("色", preview=True).bind_value(item, "color")
                ui.space()
                ui.button(icon="delete", on_click=lambda: (series.remove(item), card.delete())).props("flat round")
            file_upload(item["files"], hint)
        series.append(item)

    # Before（オレンジ）と After（青）の 2 系列を比較する使い方が基本
    add("Before", TAB10[1])
    add("After", TAB10[0])
    ui.button("系列を追加", icon="add", on_click=lambda: add(f"Series{len(series) + 1}", TAB10[(len(series) + 2) % 10])).props("flat")
    return series


def pose_panel():
    series = series_editor("Robot/Measure の CSV（1 ファイル）")
    with ui.row().classes("items-center"):
        func = ui.select(["norm_with_bar", "norm", "each_axis", "each_axis_overlay"], value="norm_with_bar", label="グラフ")
        title = ui.input("タイトル", value="Improvement in Pose Accuracy")
        fitlocal = ui.checkbox("局所座標に合わせる（fitlocal）")

    async def draw():
        datasets = [loaders.pose_dataset(s["name"], s["color"], next(iter(s["files"].values())), fitlocal.value) for s in series]
        show_png(result, await api.plot("pose_accuracy", func.value, {"datasets": datasets, "title": title.value}), f"pose_{func.value}.png")

    button = ui.button("描画", on_click=lambda: run_busy(button, draw))
    result = ui.column().classes("w-full")


def path_panel():
    series = series_editor("BT と FM の CSV（*_BT.csv / *_FM.csv の 1 組）")
    title = ui.input("タイトル", value="Improvement in Path Accuracy")

    async def draw():
        datasets = []
        for s in series:
            fm, bt = loaders.pair_files(s["files"])[0]
            datasets.append(loaders.path_dataset(s["name"], s["color"], bt, fm))
        show_png(result, await api.plot("path_accuracy", "straight_path", {"datasets": datasets, "title": title.value}), "path_accuracy.png")

    button = ui.button("描画", on_click=lambda: run_busy(button, draw))
    result = ui.column().classes("w-full")


def single_axis_panel():
    series = series_editor("FM と BT の CSV（*_FM.csv / *_BT.csv の組。複数区間なら複数組）")
    with ui.row().classes("items-center"):
        joint_no = ui.select([1, 2, 3, 4, 5, 6], value=1, label="軸", on_change=lambda e: gear_rate.set_value(joint_wave.GEAR_RATES[e.value - 1]))
        gear_rate = ui.number("減速比", value=joint_wave.GEAR_RATES[0])
        title = ui.input("タイトル", value="Single-axis angular transmission error")

    async def draw():
        datasets = []
        for s in series:
            angle, error = joint_wave.prepare(loaders.pair_files(s["files"]), joint_no.value)
            # 0.001° 刻みのままだと数十万点になるため、等間隔を保って 0.01° 刻みに間引いて送る
            datasets.append({"name": s["name"], "color": s["color"], "data": {"angle": angle[::10].tolist(), "error": error[::10].tolist()}})
        body = {"datasets": datasets, "joint_no": joint_no.value, "gear_rate": gear_rate.value, "title": title.value}
        show_png(result, await api.plot("single_axes", "angle_error", body), f"single_axis_J{joint_no.value}.png")

    button = ui.button("描画", on_click=lambda: run_busy(button, draw))
    result = ui.column().classes("w-full")


@ui.page("/draw")
def page():
    header()
    with ui.tabs() as tabs:
        pose, path, single = ui.tab("姿勢精度"), ui.tab("軌跡精度"), ui.tab("単軸")
    with ui.tab_panels(tabs, value=pose).classes("w-full"):
        with ui.tab_panel(pose):
            pose_panel()
        with ui.tab_panel(path):
            path_panel()
        with ui.tab_panel(single):
            single_axis_panel()
