"""逐次最適化ページ：ロボット（実機 / シミュレータ）で動作・計測し、パラメータ（詰めきれなければ動作軌道も）を反復更新して目標軌跡に近づける。"""
import time

import numpy as np
from nicegui import ui

from . import api, loaders
from .analysis import CALIBRATION_MODES, TRAIN_PATTERNS, payload_inputs
from .layout import TAB10, download_json, file_upload, header, run_busy

PHASES = {"param": "パラメータ更新", "trajectory": "軌道修正"}


# シミュレータで試すときの目標軌道（全軸を異なる周期で動かし、始点と終点で止まる 8 秒の動作）
def demo_target() -> dict:
    t = np.arange(0.0, 8.0, 0.01)
    joints = np.array([0, 20, 10, 0, 50, 0]) + np.array([40, 15, 20, 30, 20, 60]) * np.sin(2 * np.pi * t[:, None] / np.array([8, 4, 5.3, 2.7, 3.1, 6])) * np.sin(np.pi * t / 8)[:, None]
    return {"time": t.tolist(), "joints": joints.tolist()}


# タイトル・凡例・グラフ領域が重ならないようにする共通の見た目
def chart(title: str, **options) -> dict:
    return {"title": {"text": title, "textStyle": {"fontSize": 14}}, "tooltip": {"trigger": "axis"}, "legend": {"top": 28, "type": "scroll"}, "grid": {"top": 80, "right": 90}, "series": [], **options}


def line(name: str, color: str, data: list, **options) -> dict:
    return {"name": name, "type": "line", "color": color, "data": data, **options}


@ui.page("/optimize")
def page():
    header()
    ui.label("ロボット API（環境変数 ROBOT_URL。既定はシミュレータ :8002）で目標軌跡を動かして計測し、パラメータを同定してロボットへ送る反復を行います。"
             "パラメータ更新で誤差が下がらなくなったら、指令関節角の軌道も修正します。").classes("text-sm text-gray-500")

    # 設定：同定の方法、収束の判定、軌道修正の強さ
    with ui.row().classes("items-center"):
        # 1 つの軌跡では剛性率（特に可搬物が軽いとき）を見分けにくいため、既定は剛性を推定しない kinema_actual にする
        calibration_mode = ui.select(CALIBRATION_MODES["actual"], value="kinema_actual", label="calibration_mode").classes("w-40")
        pattern = ui.select(TRAIN_PATTERNS, value="kinema_trans_all", label="同定パターン").classes("w-72")
        interval = ui.number("計測点の間引き間隔 [ms]", value=20, format="%d").classes("w-44")
        max_nfev = ui.number("同定の評価回数上限", value=20, format="%d").classes("w-40")
    with ui.row().classes("items-center"):
        tolerance = ui.number("目標精度 RMS [mm]", value=0.01, format="%.4f").classes("w-40")
        max_iterations = ui.number("最大反復回数", value=10, format="%d").classes("w-32")
        threshold = ui.number("軌道修正へ移る改善率 [%]", value=5.0).classes("w-48")
        gain = ui.number("軌道修正ゲイン", value=0.8, step=0.1).classes("w-32")
        scale = ui.number("3D の誤差表示倍率", value=100, format="%d", on_change=lambda: draw()).classes("w-36")
    robot_settings = payload_inputs(tool=False)
    target_files: dict[str, bytes] = {}
    file_upload(target_files, "目標の関節角軌道（*_FM.csv を 1 つ）。無ければデモ軌道を使う")

    # 反復の状態（runs は各反復の計測、history は各反復の結果）
    state = {"runs": [], "history": [], "target": None, "info": None, "command": None, "fixed": None, "phase": "param", "stop": False}

    async def step() -> bool:
        """1 反復（動作・計測 → 同定 → 目標との誤差 → パラメータ送信 or 軌道修正）を行い、終了すべきかを返す。"""
        started = time.time()
        # 初回は目標軌道とロボットの情報を取り、指令軌道を目標そのものにする
        if state["target"] is None:
            state["target"] = dict(zip(("time", "joints"), (v.tolist() for v in loaders.load_fm(next(iter(target_files.values())))))) if target_files else demo_target()
            state["info"] = await api.robot("/info")
            state["command"] = state["target"]["joints"]
        target, runs = state["target"], state["runs"]
        runs.append(await api.robot("/run", {"time": target["time"], "joints": state["command"]}))

        # 同定には全反復の計測を使う（モータへ出た関節角 FM で学習するため、送ったパラメータによらずロボット本来の特性を推定できる）
        picked = []
        for run in runs:
            bt_time, xyz = np.asarray(run["bt"]["time"]), np.asarray(run["bt"]["xyz"])
            step_count = max(1, round(interval.value / 1000.0 / np.median(np.diff(bt_time))))
            picked.append((bt_time[::step_count], xyz[::step_count]))
        X = np.vstack([np.column_stack((np.full(len(t), index), t)) for index, (t, _) in enumerate(picked)])
        y = np.vstack([xyz for _, xyz in picked])
        phase = state["phase"]
        # 軌道修正の段階ではパラメータを固定し、時刻ずれと計測器の座標だけを合わせる
        settings = {**robot_settings(), **state["info"], "calibration_mode": calibration_mode.value, "pattern": pattern.value if phase == "param" else "time_only",
                    "trajectories": [run["fm"] for run in runs], "target": target, "max_nfev": int(max_nfev.value)}
        await api.analysis("/init", {"model_type": "trajectory", "settings": settings})
        # 前回の同定結果から続けて推定する（評価回数の上限で打ち切っても反復ごとに収束に近づく）
        if state["fixed"]:
            await api.analysis("/load", state["fixed"])
        await api.analysis("/train", {"X_train": X.tolist(), "y_train": y.tolist()})
        saved = await api.analysis("/save")

        # 今回の計測をロボット座標に直し、目標軌跡との誤差で精度を評価する
        latest = X[:, 0] == len(runs) - 1
        tracking = await api.analysis("/tracking_error", {"X_test": X[latest].tolist(), "y_test": y[latest].tolist()})
        norms = np.linalg.norm(tracking["errors"], axis=1)
        rms = float(np.sqrt(np.mean(norms ** 2)))
        previous = state["history"][-1]["rms"] if state["history"] else None
        state["history"].append({"iteration": len(runs), "phase": phase, "rms": rms, "max": float(norms.max()), "seconds": time.time() - started, "parameters": saved, "tracking": tracking})

        # 終了判定と次の反復の準備：改善が止まるまではパラメータを送り、止まったら軌道修正へ移る
        done = rms <= tolerance.value or len(runs) >= max_iterations.value
        if not done and phase == "param" and previous is not None and (previous - rms) / previous < threshold.value / 100.0:
            state["phase"] = "trajectory"
        if not done and state["phase"] == "param":
            state["fixed"] = saved
            await api.robot("/parameters", saved)
        elif not done:
            state["command"] = (await api.analysis("/correct", {"time": target["time"], "joints": state["command"], "error_time": tracking["time"], "errors": tracking["errors"], "gain": gain.value}))["joints"]
        draw()
        return done

    async def run_auto():
        state["stop"] = False
        while not state["stop"] and not await step():
            pass

    def reset():
        state.update({"runs": [], "history": [], "target": None, "info": None, "command": None, "fixed": None, "phase": "param"})
        draw()

    def save():
        download_json({"target": state["target"], "command": {"time": state["target"]["time"], "joints": state["command"]} if state["target"] else None,
                       "iterations": [{k: v for k, v in item.items() if k != "tracking"} for item in state["history"]]}, "optimization_history.json")

    with ui.row():
        start_button = ui.button("開始", icon="play_arrow", on_click=lambda: run_busy(start_button, run_auto))
        step_button = ui.button("1 反復", icon="skip_next", on_click=lambda: run_busy(step_button, step)).props("outline")
        ui.button("停止（この反復の後）", icon="stop", on_click=lambda: state.update(stop=True)).props("outline color=negative")
        ui.button("リセット", icon="restart_alt", on_click=reset).props("flat")
        ui.button("履歴を保存", icon="download", on_click=save).props("flat")
    status = ui.label()

    # ライブ描画：誤差の推移、目標と計測の軌跡、誤差の時系列、パラメータの推移、反復ごとの表
    with ui.grid(columns=2).classes("w-full"):
        convergence = ui.echart(chart("誤差の推移", xAxis={"type": "category", "name": "反復"}, yAxis={"type": "log", "name": "[mm]"})).classes("h-80")
        trajectory = ui.plotly({"data": [], "layout": {"title": {"text": "目標と計測の軌跡（ロボット座標、誤差を拡大）"}, "margin": {"l": 0, "r": 0, "b": 0, "t": 40}}}).classes("h-80")
        errors = ui.echart(chart("誤差ノルムの時系列", xAxis={"type": "value", "name": "時刻 [s]"}, yAxis={"type": "value", "name": "[mm]"})).classes("h-80")
        parameters = ui.echart(chart("パラメータの推移", xAxis={"type": "category", "name": "反復"}, yAxis=[{"type": "value", "name": "伝達誤差振幅 [deg]"}, {"type": "value", "name": "幾何変化 [mm, deg]"}])).classes("h-80")
    table = ui.table(columns=[{"name": k, "label": label, "field": k} for k, label in (("iteration", "反復"), ("phase", "段階"), ("rms", "RMS [mm]"), ("max", "最大 [mm]"), ("seconds", "所要 [s]"))], rows=[]).classes("w-full")

    def draw():
        history = state["history"]
        status.set_text(f"{len(history)} 反復済み／次の段階：{PHASES[state['phase']]}" if history else "未実行")
        iterations = [str(item["iteration"]) for item in history]
        # 誤差の推移（軌道修正の段階を網掛けし、目標精度を線で示す）
        switched = next((str(item["iteration"]) for item in history if item["phase"] == "trajectory"), None)
        convergence.options["xAxis"]["data"] = iterations
        convergence.options["series"] = [
            line("RMS", TAB10[0], [item["rms"] for item in history], markLine={"symbol": "none", "data": [{"yAxis": tolerance.value, "name": "目標精度"}]},
                 markArea={"data": [[{"xAxis": switched, "name": "軌道修正"}, {"xAxis": iterations[-1]}]]} if switched else {}),
            line("最大", TAB10[1], [item["max"] for item in history]),
        ]
        convergence.update()

        # 目標と計測の軌跡（誤差は mm 単位で小さいため、倍率を掛けて目標からのずれを見せる）
        data = []
        for name, color, item in (("初回", TAB10[1], history[0] if history else None), ("最新", TAB10[0], history[-1] if len(history) > 1 else None)):
            if item:
                measured, err = np.asarray(item["tracking"]["measured"]), np.asarray(item["tracking"]["errors"])
                if not data:
                    data.append({"type": "scatter3d", "mode": "lines", "name": "目標", "line": {"color": "black", "width": 2}, **dict(zip("xyz", (measured - err).T.tolist()))})
                data.append({"type": "scatter3d", "mode": "lines", "name": f"{name}（×{scale.value:g}）", "line": {"color": color, "width": 3}, **dict(zip("xyz", (measured - err + err * scale.value).T.tolist()))})
        trajectory.figure["data"] = data
        trajectory.update()

        # 誤差ノルムの時系列（初回・前回・最新）
        picks = {"初回": history[0], "前回": history[-2], "最新": history[-1]} if len(history) > 2 else dict(zip(("初回", "最新"), history))
        errors.options["series"] = [line(name, TAB10[index], list(zip(item["tracking"]["time"], np.linalg.norm(item["tracking"]["errors"], axis=1).tolist())), showSymbol=False)
                                    for index, (name, item) in zip((1, 7, 0), picks.items())]
        errors.update()

        # 伝達誤差の振幅（各軸で最大の周期成分）と、前回からの幾何誤差の変化量（収束の目安）
        waves = [item["parameters"]["transmission_error"] for item in history]
        geometry = [np.asarray(item["parameters"]["robot_model"]["GeomErr"]) for item in history]
        parameters.options["xAxis"]["data"] = iterations
        parameters.options["series"] = [line(f"J{joint}", TAB10[joint + 1], [max(wave[f"J{joint}"]["amplitudes"]) for wave in waves]) for joint in range(1, 7)] + [
            line("幾何変化 max|Δ|", "black", [None] + [float(np.abs(b - a).max()) for a, b in zip(geometry, geometry[1:])], yAxisIndex=1, lineStyle={"type": "dashed"})]
        parameters.update()

        table.rows = [{"iteration": item["iteration"], "phase": PHASES[item["phase"]], "rms": f"{item['rms']:.4f}", "max": f"{item['max']:.4f}", "seconds": f"{item['seconds']:.1f}"} for item in history]
        table.update()

    draw()
