"""使い方マニュアル（docs/manual/manual.md）用のスクリーンショットを、ダミーデータで操作しながら docs/manual/img/ に撮る。

アプリの改修後に画面を撮り直すためのスクリプト。アプリを起動した状態で、プロジェクト直下から実行する。
逐次最適化の画面はロボットシミュレータを使うため、撮影の前にシミュレータを起動し直して初期状態にしておく（docker compose restart robot-sim）。
ホストに Chromium の依存ライブラリが無くても動くよう、Playwright の公式イメージ内で実行する。

    docker compose up -d --build && docker compose restart robot-sim
    docker run --rm --network host -u "$(id -u):$(id -g)" -e HOME=/tmp -v "$PWD":/work -w /work \\
        mcr.microsoft.com/playwright/python:v1.63.0-noble \\
        sh -c "pip install -q --user --break-system-packages playwright==1.63.0 && python scripts/make_manual.py"
    docker run --rm -v "$PWD":/home/marp/app -e MARP_USER="$(id -u):$(id -g)" marpteam/marp-cli docs/manual/manual.md -o docs/manual/manual.html
"""
import argparse
import csv
import json
import math
import random
import re
import tempfile
import urllib.request
from pathlib import Path

from playwright.sync_api import Locator, Page, sync_playwright

OUT = Path(__file__).resolve().parent.parent / "docs" / "manual" / "img"
rng = random.Random(0)  # 撮り直しても同じ画像になるよう乱数を固定する
TOOL_OFFSET = [60.0, 0.0, 120.0]  # キネマ補正のダミーデータの工具オフセット [mm]


def api(url: str, path: str, body: dict | None = None):
    request = urllib.request.Request(url + path, json.dumps(body or {}).encode(), {"content-type": "application/json"})
    return json.load(urllib.request.urlopen(request))


def write_csv(path: Path, header: list[str], rows: list[list], pre: tuple = (), post: tuple = ()) -> Path:
    """pre / post は、実機の CSV にあるヘッダー前の情報行と末尾の集計行を再現するための行。"""
    with open(path, "w", newline="", encoding="shift_jis") as f:
        writer = csv.writer(f)
        writer.writerows([*pre, header, *[[round(v, 6) if isinstance(v, float) else v for v in row] for row in rows], *post])
    return path


# ---- ダミーデータ（個人・実機のデータを写さないため、すべて乱数と公称モデルから作る） ----

def pose_csv(path: Path, sigma: float) -> Path:
    rows = []
    for _ in range(60):
        robot = [rng.uniform(300, 700), rng.uniform(-300, 300), rng.uniform(200, 800)]
        rows.append(robot + [v + rng.gauss(0.0, sigma) for v in robot])
    return write_csv(path, ["RobotX", "RobotY", "RobotZ", "MeasureX", "MeasureY", "MeasureZ"], rows)


# X 方向 400 mm の直線移動。amp は進行方向に直交するずれの大きさ [mm]
def path_pair(folder: Path, stem: str, amp: float) -> list[Path]:
    xs = [300.0 + 0.5 * i for i in range(801)]
    bt = [[i, x, -200.0 + amp * math.sin(2 * math.pi * 1.5 * (x - 300) / 400) + rng.gauss(0, 0.005), 500.0 + 0.6 * amp * math.sin(2 * math.pi * 2.5 * (x - 300) / 400 + 1) + rng.gauss(0, 0.005)] for i, x in enumerate(xs)]
    fm = [[i, x, -200.0, 500.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] for i, x in enumerate(xs)]
    return [write_csv(folder / f"{stem}_BT.csv", ["TIMESTAMP", "#X(mm)", "Y(mm)", "Z(mm)"], bt, pre=[["FARO dummy"]]),
            write_csv(folder / f"{stem}_FM.csv", FM_HEADER, fm, pre=[["robot log dummy"], ["sampling 1ms"]], post=[["MAX"], ["MIN"]])]


FM_HEADER = ["Time[ms]", "RefPos(X)[mm]", "RefPos(Y)[mm]", "RefPos(Z)[mm]"] + [f"Joint(J{j})[deg]" for j in range(1, 7)]


# J1 を 0→30° 台形速度で回し、FARO 側は半径 800 mm の円弧に周期的な伝達誤差（減速比 120 の 1 次・2 次）を乗せる
def joint_pair(folder: Path, stem: str, a1: float, a2: float) -> list[Path]:
    speed, accel, dist, dwell = 10.0, 40.0, 30.0, 0.3
    ta = speed / accel
    tc = dist / speed - ta

    def angle(t: float) -> float:
        t = min(max(t - dwell, 0.0), 2 * ta + tc)
        return 0.5 * accel * t * t if t < ta else 0.5 * accel * ta * ta + speed * (t - ta) if t < ta + tc else dist - 0.5 * accel * (2 * ta + tc - t) ** 2

    fm, bt = [], []
    for ms in range(int((2 * dwell + 2 * ta + tc) * 1000)):
        q = angle(ms / 1000)
        actual = math.radians(q + a1 * math.sin(2 * math.pi * q / 1.5 + 0.3) + a2 * math.sin(2 * math.pi * q / 0.75 + 1.0))
        fm.append([ms, 800 * math.cos(math.radians(q)), 800 * math.sin(math.radians(q)), 600.0, q, 0.0, 0.0, 0.0, -90.0, 0.0])
        # FARO は時計が別なので、時刻の起点をずらしておく
        bt.append([5000 + ms, 800 * math.cos(actual) + rng.gauss(0, 0.001), 800 * math.sin(actual) + rng.gauss(0, 0.001), 600.0 + rng.gauss(0, 0.001)])
    return [write_csv(folder / f"{stem}_FM.csv", FM_HEADER, fm, pre=[["robot log dummy"], ["sampling 1ms"]], post=[["MAX"], ["MIN"]]),
            write_csv(folder / f"{stem}_BT.csv", ["TIMESTAMP", "#X(mm)", "Y(mm)", "Z(mm)"], bt, pre=[["FARO dummy"]])]


# 解析 API が保持している kinema_joint モデルに、真値として幾何誤差と関節の伝達誤差を乗せる
def add_errors(analysis_url: str):
    params = api(analysis_url, "/save")
    params["robot_model"]["GeomErr"] = [[rng.gauss(0, s) for s in (0.2, 0.2, 0.2, 0.0005, 0.0005, 0.0005)] for _ in params["robot_model"]["GeomErr"]]
    for wave in params["transmission_error"].values():
        wave["amplitudes"] = [rng.uniform(0.001, 0.004) for _ in wave["periods"]]
        wave["offsets"] = [rng.uniform(-180, 180) for _ in wave["periods"]]
    api(analysis_url, "/load", params)


# 公称パラメータで指令位置を、幾何誤差と関節の伝達誤差を乗せたパラメータで計測位置を作る（解析 API の R6E モデルを使う）
def faro_csvs(folder: Path, analysis_url: str) -> list[Path]:
    joints = [[rng.uniform(-60, 60), rng.uniform(-20, 40), rng.uniform(-20, 40), rng.uniform(-90, 90), rng.uniform(-60, 60), rng.uniform(-90, 90)] for _ in range(120)]
    X = [j + [1] for j in joints]
    # 工具が J6 軸上にあると J6 の伝達誤差が位置に現れないため、軸から外したオフセットにする（画面にも同じ値を入れる）
    api(analysis_url, "/init", {"model_type": "kinema_joint", "settings": {"robot_type": "R6E", "tool_offsets": [TOOL_OFFSET]}})
    robot = api(analysis_url, "/predict", {"X": X})["predictions"]
    add_errors(analysis_url)
    measure = api(analysis_url, "/predict", {"X": X})["predictions"]
    header = [f"J{j}" for j in range(1, 7)] + ["RobotX", "RobotY", "RobotZ", "MeasureX", "MeasureY", "MeasureZ", "ToolID"]
    rows = [j + r + [v + rng.gauss(0, 0.01) for v in m] + [1] for j, r, m in zip(joints, robot, measure)]
    return [write_csv(folder / f"faro_sample{i + 1}.csv", header, rows[i * 60:(i + 1) * 60]) for i in range(2)]


# 3 動作ぶんの FM（関節角の軌道、1 ms 刻み）と BT（計測器の手先軌跡、5 ms 刻み）の組。
# BT は誤差入りのモデルの手先位置を、計測器の座標（回転＋並進）へ移し、動作ごとに時刻をずらした絶対時刻で書く
def trajectory_pairs(folder: Path, analysis_url: str) -> list[Path]:
    api(analysis_url, "/init", {"model_type": "kinema_joint", "settings": {"robot_type": "R6E", "tool_offsets": [TOOL_OFFSET]}})
    add_errors(analysis_url)
    roll, pitch, yaw = (math.radians(v) for v in (10.0, -5.0, 120.0))
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    rotation = [[cr * cp, cr * sp * sy - sr * cy, cr * sp * cy + sr * sy], [sr * cp, sr * sp * sy + cr * cy, sr * sp * cy - cr * sy], [-sp, cp * sy, cp * cy]]
    shift = [1500.0, -300.0, 200.0]
    paths = []
    for motion in range(3):
        # 前後 1 秒は停止し、その間はなめらかに動く 10 秒の軌道
        center = [rng.uniform(lo, hi) for lo, hi in zip((-60, -10, 0, -60, -40, -90), (60, 30, 30, 60, 40, 90))]
        amp = [rng.uniform(lo, hi) for lo, hi in zip((20, 10, 10, 30, 20, 40), (60, 25, 25, 80, 40, 120))]
        wave = [(rng.uniform(0.05, 0.2), rng.uniform(0, 2 * math.pi)) for _ in range(6)]
        joints = lambda t: [c + a * math.sin(2 * math.pi * f * t + p) * min(max(min(t - 1.0, 9.0 - t), 0.0), 1.0) ** 2 for c, a, (f, p) in zip(center, amp, wave)]
        fm = [[ms, 0.0, 0.0, 0.0] + joints(ms / 1000) for ms in range(10_000)]
        # BT は FM の 0.3〜0.5 秒目から記録を始め、計測器の時計は FM とずれている
        robot_times = [0.3 + 0.1 * motion + 0.005 * i for i in range(1800)]
        robot = api(analysis_url, "/predict", {"X": [joints(t) + [1] for t in robot_times]})["predictions"]
        clock = 1.7e9 + rng.uniform(-5000, 5000)
        bt = [[(t * 1000 + clock), *[sum(r * v for r, v in zip(row, position)) + s + rng.gauss(0, 0.01) for row, s in zip(rotation, shift)]] for t, position in zip(robot_times, robot)]
        paths += [write_csv(folder / f"move{motion + 1}_FM.csv", FM_HEADER, fm, pre=[["robot log dummy"], ["sampling 1ms"]], post=[["MAX"], ["MIN"]]),
                  write_csv(folder / f"move{motion + 1}_BT.csv", ["TIMESTAMP", "#X(mm)", "Y(mm)", "Z(mm)"], bt, pre=[["FARO dummy"]])]
    return paths


# 逐次最適化の目標軌道（10 ms 刻み 8 秒の FM）。全軸を異なる周期で動かし、始点と終点で止まる
def target_fm(folder: Path) -> Path:
    period, amp, center = (8, 4, 5.3, 2.7, 3.1, 6), (40, 15, 20, 30, 20, 60), (0, 20, 10, 0, 50, 0)
    fm = [[ms, 0.0, 0.0, 0.0] + [c + a * math.sin(2 * math.pi * ms / 1000 / t) * math.sin(math.pi * ms / 8000) for c, a, t in zip(center, amp, period)] for ms in range(0, 8000, 10)]
    return write_csv(folder / "target_FM.csv", FM_HEADER, fm, pre=[["robot log dummy"], ["sampling 10ms"]], post=[["MAX"], ["MIN"]])


# 2 本の工具で姿勢を変えながら同じ点付近を計測したデータ。計測値は真のオフセットを入れたツール補正モデルで作る
def toolcalib_csv(folder: Path, analysis_url: str) -> Path:
    poses = [[500 + rng.uniform(-50, 50), rng.uniform(-50, 50), 400 + rng.uniform(-50, 50), rng.uniform(-30, 30), rng.uniform(-30, 30), rng.uniform(-180, 180), 1 + i % 2] for i in range(30)]
    X = [p + [0, i] for i, p in enumerate(poses)]
    api(analysis_url, "/init", {"model_type": "tool_calib", "settings": {"tool_offsets": [[5.0, -3.0, 120.0], [40.0, 2.0, 95.0]], "observation_model": "relative"}})
    measure = api(analysis_url, "/predict", {"X": X})["predictions"]
    rows = [p[:6] + [v + rng.gauss(0, 0.005) for v in m] + [p[6]] for p, m in zip(poses, measure)]
    return write_csv(folder / "toolcalib_sample.csv", [f"Robot{a}" for a in "XYZUVW"] + ["MeasureX", "MeasureY", "MeasureZ", "ToolID"], rows)


# ---- 画面操作と撮影 ----

class Manual:
    def __init__(self, page: Page):
        self.page = page

    def shot(self, name: str, *marks: Locator, top: Locator | None = None):
        """marks を赤枠で囲んで撮る。top を指定するとその要素がヘッダー直下に、無ければ最初の赤枠が画面中央に来るようスクロールする。"""
        if top:
            top.first.evaluate("e => window.scrollTo(0, e.getBoundingClientRect().top + window.scrollY - 80)")
        elif marks:
            marks[0].first.evaluate("e => e.scrollIntoView({block: 'center'})")
        self.page.wait_for_timeout(400)  # スクロールやメニューのアニメーションが落ち着くのを待つ
        for mark in marks:
            mark.evaluate_all("""els => els.forEach(e => {
                // 画面端の要素でも枠が切れないよう、画面内に収める
                const r = e.getBoundingClientRect(), d = document.createElement('div');
                const left = Math.max(r.left - 5, 1), top = Math.max(r.top - 5, 1), right = Math.min(r.right + 5, innerWidth - 1), bottom = Math.min(r.bottom + 5, innerHeight - 1);
                d.className = 'manual-mark';
                Object.assign(d.style, {position: 'fixed', left: `${left}px`, top: `${top}px`, width: `${right - left}px`, height: `${bottom - top}px`, boxSizing: 'border-box',
                    border: '3px solid #e53935', borderRadius: '6px', zIndex: 99999, pointerEvents: 'none'});
                document.body.append(d);
            })""")
        self.page.screenshot(path=OUT / f"{name}.png")
        self.page.evaluate("document.querySelectorAll('.manual-mark').forEach(e => e.remove())")
        print(name)

    def field(self, label: str) -> Locator:
        return self.page.locator(".q-field:visible").filter(has=self.page.locator(".q-field__label", has_text=re.compile(f"^{re.escape(label)}$")))

    def button(self, name: str) -> Locator:
        return self.page.get_by_role("button", name=name, exact=True)

    def tab(self, name: str) -> Locator:
        """タブを切り替える。切り替えのアニメーション中は前のタブも表示されているため、終わるまで待つ。"""
        tab = self.page.get_by_role("tab", name=name)
        tab.click()
        self.page.wait_for_timeout(800)
        return tab

    def upload(self, index: int, paths: list[Path]) -> Locator:
        """index 番目のアップロード欄にファイルを入れ、追加されたチップを返す。"""
        self.page.locator(".q-uploader:visible").nth(index).locator("input[type=file]").set_input_files(paths)
        chips = self.page.locator(".q-chip:visible").filter(has_text=re.compile("|".join(re.escape(p.name) for p in paths)))
        chips.last.wait_for()
        return chips

    def run(self, button: Locator):
        """処理中スピナーが出て消えるまで待つ（学習・描画の完了待ち）。"""
        button.click()
        button.locator(".q-spinner").wait_for(state="attached", timeout=5000)
        button.locator(".q-spinner").wait_for(state="detached", timeout=300_000)


def draw_page(m: Manual, data: dict):
    page = m.page
    m.shot("01_start", page.get_by_role("tab"))
    m.shot("02_header", page.locator(".q-header a"))

    # 姿勢精度：系列の編集 → ファイル → 系列の追加・削除 → 設定 → 描画 → 保存
    cards = page.locator(".q-card:visible")
    m.shot("03_series", cards.nth(0).locator(".q-field"), top=page.get_by_role("tab", name="姿勢精度"))
    m.upload(0, [data["pose_before"]])
    m.upload(1, [data["pose_after"]])
    m.shot("04_upload", page.locator(".q-uploader:visible"), page.locator(".q-chip:visible"), top=page.get_by_role("tab", name="姿勢精度"))
    m.button("系列を追加").click()
    m.shot("05_add_series", m.button("系列を追加"), cards.nth(2).locator("button:has(i:text('delete'))"), top=cards.nth(1))
    cards.nth(2).locator("button:has(i:text('delete'))").click()
    m.field("グラフ").click()
    m.shot("06_options", m.field("グラフ"), page.locator(".q-menu"), m.field("タイトル"), page.locator(".q-checkbox"), top=cards.nth(1))
    page.keyboard.press("Escape")
    m.run(m.button("描画"))
    m.shot("07_draw", m.button("描画"), page.locator(".q-img:visible"), top=m.button("描画"))
    m.shot("08_save_png", m.button("PNG を保存"))

    # 軌跡精度：系列ごとに BT/FM の組を入れて描画
    m.tab("軌跡精度")
    m.upload(0, data["path_before"])
    m.upload(1, data["path_after"])
    m.shot("09_path_upload", page.get_by_role("tab", name="軌跡精度"), page.locator(".q-chip:visible"), top=page.get_by_role("tab", name="軌跡精度"))
    m.run(m.button("描画"))
    m.shot("10_path_draw", m.button("描画"), page.locator(".q-img:visible"), top=m.button("描画"))

    # 単軸：軸を選ぶと減速比が入る。FM/BT の組を入れて描画
    m.tab("単軸")
    m.upload(0, data["joint_before"])
    m.upload(1, data["joint_after"])
    m.shot("11_single_settings", m.field("軸"), m.field("減速比"), page.locator(".q-chip:visible"), top=page.locator(".q-card:visible").nth(1))
    m.run(m.button("描画"))
    m.shot("12_single_draw", m.button("描画"), page.locator(".q-img:visible"), top=m.button("描画"))


def analysis_page(m: Manual, data: dict, work: Path):
    page = m.page
    page.locator(".q-header a", has_text="解析").click()
    page.wait_for_url("**/analysis")
    note = page.get_by_text("学習済みモデルを 1 つだけ保持")
    m.shot("13_analysis_note", note)

    # キネマ補正：機種・モード → 同定パターン → 荷重・工具 → CSV と学習 → 結果 → 保存 → 保存済みパラメータで評価
    m.field("calibration_mode").click()
    m.shot("14_kinema_mode", m.field("機種"), m.field("mode"), m.field("calibration_mode"), page.locator(".q-menu"))
    page.keyboard.press("Escape")
    m.field("同定パターン").click()
    m.shot("15_kinema_pattern", m.field("同定パターン"), page.locator(".q-menu"))
    page.keyboard.press("Escape")
    page.locator(".q-textarea textarea").fill(", ".join(f"{v:g}" for v in TOOL_OFFSET))
    m.shot("16_kinema_payload", page.locator(".q-field").filter(has_text=re.compile("質量|重心|重力")), page.locator(".q-textarea"), top=m.field("機種"))
    chips = m.upload(0, data["faro"])
    m.shot("17_kinema_train", page.locator(".q-uploader:visible"), chips, m.button("学習"), top=page.locator(".q-textarea"))
    m.run(m.button("学習"))
    m.shot("18_kinema_result", page.locator(".q-img:visible"), page.get_by_text("R² ="), top=page.locator(".q-img:visible"))
    with page.expect_download() as download:
        m.button("パラメータを保存").click()
    params = work / download.value.suggested_filename
    download.value.save_as(params)
    m.shot("19_save_param", m.button("パラメータを保存"))
    page.get_by_text("保存済みパラメータで評価する").click()
    chips = m.upload(1, [params])
    m.run(m.button("読み込んで評価"))
    m.shot("20_load_param", page.get_by_text("保存済みパラメータで評価する"), chips, m.button("読み込んで評価"), top=m.button("学習"))

    # キネマ＋伝達誤差：パターンを選んで学習 → 補正の効果 → 振幅・位相の表
    m.field("同定パターン").click()
    page.get_by_role("option", name="キネマ＋伝達誤差 全軸（同時）").click()
    m.shot("21_joint_kinema_train", m.field("同定パターン"), m.button("学習"), top=m.field("機種"))
    m.run(m.button("学習"))
    m.shot("22_joint_kinema_result", page.locator(".q-img:visible"), page.get_by_text("R² ="), top=page.locator(".q-img:visible"))
    m.shot("23_joint_kinema_table", page.locator(".q-table__container:visible"))

    # 保存済みのキネマ（キネマのみで学習したもの）を固定して、伝達誤差だけを同定する
    m.field("同定パターン").click()
    page.get_by_role("option", name="伝達誤差 全軸", exact=True).click()
    page.get_by_text("このパラメータを学習の初期値にする").click()
    m.shot("24_trans_only", m.button("学習"), page.locator(".q-expansion-item .q-chip"), page.locator(".q-checkbox"), top=m.button("学習"))
    m.run(m.button("学習"))

    # 軌跡キャリブ：設定 → FM/BT の組と学習 → 補正の効果 → 時刻ずれと計測器の座標 → 保存済みパラメータで別データを評価
    m.tab("軌跡キャリブ")
    page.locator(".q-textarea:visible textarea").fill(", ".join(f"{v:g}" for v in TOOL_OFFSET))
    m.field("同定パターン").click()
    m.shot("25_traj_settings", m.field("機種"), m.field("同定パターン"), m.field("計測点の間引き間隔 [ms]"), page.locator(".q-menu"), top=page.get_by_role("tab", name="軌跡キャリブ"))
    page.get_by_role("option", name="キネマ＋伝達誤差 全軸（同時）").click()
    chips = m.upload(0, data["trajectory"])
    m.shot("26_traj_train", page.locator(".q-uploader:visible").first, chips, m.button("学習"), top=page.locator(".q-textarea:visible"))
    m.run(m.button("学習"))
    m.shot("27_traj_result", page.locator(".q-img:visible"), page.get_by_text("R² ="), top=page.locator(".q-img:visible"))
    m.shot("28_traj_offsets", page.locator(".q-table__container:visible").first, page.get_by_text("計測器の座標"), top=page.get_by_text("R² ="))
    page.get_by_text("保存済みパラメータを使う").click()
    chips = m.upload(1, [params])
    page.get_by_text("このパラメータを学習の初期値にする").click()
    m.field("同定パターン").click()
    page.get_by_role("option", name="時刻・座標のみ").click()
    m.shot("29_traj_evaluate", m.button("学習"), chips, page.locator(".q-checkbox:visible"), top=m.button("学習"))
    m.run(m.button("学習"))

    # 関節補正：軸・減速比・maxfev → FM/BT の組 → 学習 → 補正前後のグラフと周期成分の表
    m.tab("関節補正")
    chips = m.upload(0, data["joint_before"])
    m.shot("30_joint_train", m.field("軸"), m.field("減速比"), m.field("maxfev"), chips, m.button("学習"), top=page.get_by_role("tab", name="関節補正"))
    m.run(m.button("学習"))
    m.shot("31_joint_result", page.locator(".q-table__container:visible"))

    # ツール補正：CSV → 学習 → RMSE と工具オフセットの表
    m.tab("ツール補正")
    chips = m.upload(0, [data["toolcalib"]])
    m.shot("32_tool_train", page.locator(".q-uploader:visible"), chips, m.button("学習"), top=page.get_by_role("tab", name="ツール補正"))
    m.run(m.button("学習"))
    m.shot("33_tool_result", page.get_by_text("相対 RMSE"), page.locator(".q-table__container:visible"), top=page.get_by_role("tab", name="ツール補正"))



def optimize_page(m: Manual, data: dict):
    """逐次最適化：設定 → 目標軌道を入れて開始 → 誤差の推移 → 軌跡と誤差の時系列 → パラメータの推移と表 → 履歴の保存。ロボットはシミュレータ（起動直後の状態）を使う。"""
    page = m.page
    page.locator(".q-header a", has_text="逐次最適化").click()
    page.wait_for_url("**/optimize")
    m.shot("34_opt_settings", m.field("calibration_mode"), m.field("同定パターン"), page.locator(".q-field").filter(has_text=re.compile("目標精度|最大反復|改善率|ゲイン")), top=page.locator(".q-field").first)
    chips = m.upload(0, [data["target"]])
    m.shot("35_opt_start", page.locator(".q-uploader"), chips, m.button("開始"), m.button("停止（この反復の後）"), top=page.locator(".q-uploader"))
    m.run(m.button("開始"))
    charts = page.locator(".nicegui-echart, .js-plotly-plot")
    m.shot("36_opt_convergence", charts.nth(0), page.get_by_text("反復済み"), top=page.get_by_text("反復済み"))
    m.shot("37_opt_trajectory", charts.nth(1), charts.nth(2), top=charts.nth(1))
    m.shot("38_opt_parameters", charts.nth(3), page.locator(".q-table__container"), top=charts.nth(3))
    m.shot("39_opt_save", m.button("履歴を保存"), m.button("リセット"), top=page.locator(".q-uploader"))


def error_page(m: Manual, data: dict):
    # エラー表示の例：関節補正で BT を入れ忘れた場合
    page = m.page
    page.locator(".q-header a", has_text="解析").click()
    page.wait_for_url("**/analysis")
    m.tab("関節補正")
    m.upload(0, [data["joint_before"][0]])
    m.button("学習").click()
    page.locator(".q-notification").wait_for()
    m.shot("40_error", page.locator(".q-notification"))


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--url", default="http://localhost:8080", help="画面（NiceGUI）の URL")
    parser.add_argument("--analysis-url", default="http://localhost:8001", help="解析 API の URL（ダミーデータの作成に使う）")
    args = parser.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)

    with tempfile.TemporaryDirectory() as tmp, sync_playwright() as p:
        work = Path(tmp)
        data = {
            "pose_before": pose_csv(work / "pose_before.csv", 0.5), "pose_after": pose_csv(work / "pose_after.csv", 0.1),
            "path_before": path_pair(work, "path_before", 0.2), "path_after": path_pair(work, "path_after", 0.05),
            "joint_before": joint_pair(work, "J1_before", 0.004, 0.0015), "joint_after": joint_pair(work, "J1_after", 0.001, 0.0004),
            "faro": faro_csvs(work, args.analysis_url), "toolcalib": toolcalib_csv(work, args.analysis_url),
            "trajectory": trajectory_pairs(work, args.analysis_url), "target": target_fm(work),
        }
        browser = p.chromium.launch()
        page = browser.new_page(viewport={"width": 1280, "height": 800}, accept_downloads=True)
        page.set_default_timeout(30_000)
        page.goto(args.url)
        page.wait_for_url("**/draw")
        m = Manual(page)
        draw_page(m, data)
        analysis_page(m, data, work)
        optimize_page(m, data)
        error_page(m, data)
        browser.close()


if __name__ == "__main__":
    main()
