"""matplotlib でグラフを描画して PNG を返す機能（reference/DrawServer から移植）。"""
import io

import matplotlib

# GUI を持たないサーバ上で描画するため、pyplot の import より前に非対話バックエンドを指定する
matplotlib.use("Agg")

import matplotlib.pyplot as plt
from fastapi.responses import Response


# 描画した figure を PNG レスポンスにし、メモリリークを防ぐため figure を破棄する
def png_response(fig, dpi: int) -> Response:
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=dpi, bbox_inches="tight")
    plt.close(fig)
    return Response(content=buf.getvalue(), media_type="image/png")
