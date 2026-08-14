#!/usr/bin/env python3
"""重新產生手機模擬器網址的 QR code。

    pip install segno
    python3 make_qr.py

換網址（自訂網域、改 repo 名）才需要重跑，改網頁內容不用 —— QR 裡只有網址。
產出兩個檔，都直接進版控，這樣看 repo 的人不用自己裝 segno：

    qr.png   給 README 嵌入用
    qr.svg   向量，印海報或貼簡報用

M 級糾錯（約 15% 容錯）：印出來有髒污、反光或被手指擋掉一角還掃得到。
掃描端不會因為換等級而變慢，沒必要為了圖小一點降到 L。
"""

import os

import segno

URL = "https://rem3000.github.io/swerve-playground/web/"
HERE = os.path.dirname(os.path.abspath(__file__))

# 深色用頁面的底色而不是純黑，貼進簡報時比較不突兀；淺色一定要留白，
# 掃描器靠反差找定位圖案，用透明背景印出來會變成看運氣
DARK, LIGHT = "#0e1116", "#ffffff"


def main():
    qr = segno.make(URL, error="m")
    print(f"{URL}\n版本 {qr.version}　糾錯 {qr.error.upper()}　"
          f"{qr.symbol_size(scale=1, border=0)[0]} 模組")

    for name in ("qr.png", "qr.svg"):
        path = os.path.join(HERE, name)
        qr.save(path, scale=10, border=3, dark=DARK, light=LIGHT)
        print(f"{name}　{os.path.getsize(path)} bytes")


if __name__ == "__main__":
    main()
