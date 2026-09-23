from nicegui import ui

# 各ページは import 時に @ui.page で登録される
from . import analysis, draw  # noqa: F401


# トップページは描画ページへ移動する
@ui.page("/")
def index():
    ui.navigate.to("/draw")


# 単体起動（Docker / Dev Container）用。exe では desktop/launcher.py から起動する
if __name__ in {"__main__", "__mp_main__"}:
    ui.run(host="0.0.0.0", port=8080, title="robot-calibration", reload=False, show=False)
