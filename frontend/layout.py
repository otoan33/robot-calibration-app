"""各ページで共通に使う画面部品。"""
import base64
import json
from collections.abc import Awaitable, Callable

from nicegui import ui

# matplotlib の tab10 の先頭色。Before をオレンジ、After を青にする慣例に合わせる
TAB10 = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd", "#8c564b", "#e377c2", "#7f7f7f", "#bcbd22", "#17becf"]


def header():
    """アプリ名とページ間のリンクを表示する。"""
    with ui.header().classes("items-center gap-6"):
        ui.label("robot-calibration").classes("text-lg font-bold")
        ui.link("描画", "/draw").classes("text-white")
        ui.link("解析", "/analysis").classes("text-white")


def file_upload(files: dict[str, bytes], label: str):
    """アップロードされたファイルを files（ファイル名 → 中身）に溜め、名前をチップで一覧する。"""
    @ui.refreshable
    def names():
        with ui.row().classes("gap-1"):
            for name in sorted(files):
                ui.chip(name, removable=True, on_value_change=lambda _, name=name: (files.pop(name), names.refresh())).props("dense")

    # アップロード欄の一覧はサーバー側の状態と連動しないため、受け取るたびに空にしてチップ側で管理する
    async def on_upload(e):
        for file in e.files:
            files[file.name] = await file.read()
        upload.reset()
        names.refresh()

    upload = ui.upload(label=label, multiple=True, auto_upload=True, on_multi_upload=on_upload).props("flat bordered accept=.csv,.json").classes("w-full")
    names()


async def run_busy(button: ui.button, task: Callable[[], Awaitable[None]]):
    """処理中はボタンを無効にしてスピナーを出し、失敗したら原因を画面に通知する。"""
    button.props("loading")
    try:
        await task()
    except Exception as error:  # 画面操作では例外がログにしか出ないため、利用者に見える形で知らせる
        ui.notify(f"{type(error).__name__}: {error}", type="negative", multi_line=True)
    finally:
        button.props(remove="loading")


def show_png(container: ui.element, png: bytes, filename: str):
    """API が返した PNG を表示し、保存ボタンを付ける。"""
    container.clear()
    with container:
        ui.image(f"data:image/png;base64,{base64.b64encode(png).decode()}").classes("w-full max-w-5xl")
        ui.button("PNG を保存", icon="download", on_click=lambda: ui.download.content(png, filename)).props("flat")


def download_json(data: dict, filename: str):
    ui.download.content(json.dumps(data, ensure_ascii=False, indent=2).encode(), filename)
