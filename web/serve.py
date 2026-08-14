#!/usr/bin/env python3
"""把 sim/web/ 用 HTTP 開出來，讓手機連進來跑模擬。

    python3 serve.py            # 預設 8000 埠
    python3 serve.py 8080

服務根目錄只有 **sim/web/**，`sim/` 底下其他東西（模型以外的原始碼、測試）
不會被送出去。網頁需要的 `swerve_model.py` 用一條路由映射到 `sim/` 的正本，
不複製檔案 —— 複製就會多出一份會過期的公式，那是這整個設計最要避免的事。
GitHub Pages 那條路是在 CI 裡複製，複製品不進 repo，效果一樣。

模擬完全跑在手機的瀏覽器裡，這支程式只負責送檔案，關掉之後手機那邊還會繼續跑。

手機要跟這台電腦在同一個 Wi-Fi。連不上通常是防火牆擋掉 Python，
Windows 第一次跑會跳允許視窗，要勾「私人網路」。
"""

import http.server
import os
import socket
import sys

WEB_DIR = os.path.dirname(os.path.abspath(__file__))
SIM_DIR = os.path.dirname(WEB_DIR)
MODEL = os.path.join(SIM_DIR, "swerve_model.py")


def lan_ip():
    """本機在區網上的位址。不會真的送出封包，只是讓 OS 挑一張網卡。"""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        s.close()


class Handler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *a, **kw):
        super().__init__(*a, directory=WEB_DIR, **kw)

    # 白名單：跑模擬需要的就這兩個。日後加檔案（CSS、圖示）要一起加進來。
    ALLOWED = ("index.html", "swerve_model.py")

    def translate_path(self, path):
        # 比對用 super() 正規化過的結果，`..` 之類的花樣在這之前就被吃掉了
        real = super().translate_path(path)
        if os.path.normcase(real) == os.path.normcase(os.path.join(WEB_DIR, "swerve_model.py")):
            return MODEL                      # 唯一一條伸出 web/ 之外的路由
        if os.path.normcase(real.rstrip("/\\")) == os.path.normcase(WEB_DIR):
            return real                       # 根目錄，交給 SimpleHTTPRequestHandler 找 index.html
        if os.path.basename(real) not in self.ALLOWED:
            return os.path.join(WEB_DIR, "__blocked__")  # 不存在的路徑 -> 404
        return real

    def end_headers(self):
        # 改了 index.html 或 swerve_model.py 之後，手機重新整理就要拿到新的
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def log_message(self, fmt, *args):
        pass  # 每張圖每個檔都印一行太吵


def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8000
    httpd = http.server.ThreadingHTTPServer(("0.0.0.0", port), Handler)
    print(f"根目錄  {WEB_DIR}")
    print(f"模型    {MODEL}")
    print(f"本機    http://127.0.0.1:{port}/")
    print(f"手機    http://{lan_ip()}:{port}/")
    print("Ctrl-C 結束")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print()


if __name__ == "__main__":
    main()
