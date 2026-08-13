"""數字鍵盤對應的測試：直接對視窗灌 key event，檢查解出來的 cmd_vel 對不對。

    python test_input.py
"""

import sys
import time

from swerve_model import MAX_LINEAR_SPEED_MPS, MAX_ANGULAR_SPEED_RPS
import swerve_sim

fails = []


def check(cond, msg):
    print(("  [ OK ] " if cond else "  [FAIL] ") + msg)
    if not cond:
        fails.append(msg)


app = swerve_sim.SwerveSim()
# 一定要先停掉自動更新迴圈，否則 update() 會排不乾淨而卡死（見 stop_loop 說明）
app.stop_loop()
# 視窗還沒 realize 之前，合成的第一個 key event 沒有 focus 目標會被丟掉，
# 所以先讓它跑一輪事件迴圈再開始測
app.update()
app.focus_force()
app.event_generate("<KeyPress-F12>", when="now")
app.update()
app.keys.clear()

app.speed_scale = 1.0
V = MAX_LINEAR_SPEED_MPS
W = MAX_ANGULAR_SPEED_RPS
SQ = 0.7071067811865476


def press(*keysyms):
    """模擬「按住這些鍵」。

    注意不能用 app.event_generate("<KeyPress-KP_8>")：Tk 在 Windows 上沒辦法
    把 KP_* 的 keysym 反查成 keycode，合成出來的事件 keysym 會變成 "??"。
    所以這裡直接把 keysym 塞進按鍵狀態表，測的是對應表與解算邏輯本身。
    """
    app.keys = {ks: time.perf_counter() for ks in keysyms}
    return app.read_cmd()


def close(got, want, tol=1e-6):
    return all(abs(g - w) < tol for g, w in zip(got, want))


print("\n[1] 數字鍵盤：NumLock 開啟時的 keysym")
cases = [
    ("KP_8", (V, 0, 0), "前進"),
    ("KP_2", (-V, 0, 0), "後退"),
    ("KP_4", (0, V, 0), "左平移"),
    ("KP_6", (0, -V, 0), "右平移"),
    ("KP_7", (V * SQ, V * SQ, 0), "左前斜走"),
    ("KP_9", (V * SQ, -V * SQ, 0), "右前斜走"),
    ("KP_1", (-V * SQ, V * SQ, 0), "左後斜走"),
    ("KP_3", (-V * SQ, -V * SQ, 0), "右後斜走"),
    ("KP_0", (0, 0, W), "逆時針自轉"),
    ("KP_Decimal", (0, 0, -W), "順時針自轉"),
]
for ks, want, name in cases:
    got = press(ks)
    check(close(got, want), f"{ks:<12} -> {name:<8} cmd=({got[0]:+.3f}, {got[1]:+.3f}, {got[2]:+.3f})")

print("\n[2] 數字鍵盤：NumLock 關閉時的 keysym 也要能用")
for ks, want, name in [("KP_Up", (V, 0, 0), "前進"), ("KP_Home", (V * SQ, V * SQ, 0), "左前"),
                       ("KP_Insert", (0, 0, W), "逆時針"), ("KP_Delete", (0, 0, -W), "順時針")]:
    got = press(ks)
    check(close(got, want), f"{ks:<12} -> {name}")

print("\n[3] 組合鍵")
got = press("KP_8", "KP_0")
check(close(got, (V, 0, W)), f"8+0 = 邊前進邊自轉 cmd=({got[0]:+.2f}, {got[1]:+.2f}, {got[2]:+.2f})")
got = press("KP_8", "KP_4")
check(close(got, (V * SQ, V * SQ, 0)), "8+4 = 左前斜走（和 7 等價，且不會比直走快）")
got = press("KP_8", "KP_2")
check(close(got, (0, 0, 0)), "8+2 = 互相抵銷")
got = press("KP_8", "KP_5")
check(close(got, (0, 0, 0)), "5 優先，任何情況都全停")

print("\n[4] 斜走不能比直走快")
for ks in ("KP_7", "KP_9", "KP_1", "KP_3"):
    vx, vy, _ = press(ks)
    check(abs((vx ** 2 + vy ** 2) ** 0.5 - V) < 1e-6, f"{ks} 的速度大小 = {(vx**2+vy**2)**0.5:.4f} = 上限")

print("\n[5] 放開按鍵後會停（靠 KEY_HOLD_S 逾時，吃掉自動重複的空窗）")
press("KP_8")
app.keys = {k: t - swerve_sim.KEY_HOLD_S - 0.01 for k, t in app.keys.items()}
got = app.read_cmd()
check(close(got, (0, 0, 0)), f"超過 {swerve_sim.KEY_HOLD_S}s 沒有新的 KeyPress -> 停車")

print("\n[6] 速度倍率")
app.speed_scale = 0.5
got = press("KP_8")
check(close(got, (V * 0.5, 0, 0)), f"倍率 0.5 時前進 = {got[0]:.3f}")
app.speed_scale = 1.0

print("\n[7] 功能鍵")
app.event_generate("<KeyPress-m>", when="now"); app.update()
check(app.miscal and abs(app.bot.physical_offset_deg[0] - (-45.0 + 15.0)) < 1e-9,
      "M 注入 FL +15 度校正誤差")
app.event_generate("<KeyPress-m>", when="now"); app.update()
check(not app.miscal and abs(app.bot.physical_offset_deg[0] - (-45.0)) < 1e-9,
      "再按一次 M 復原")
app.event_generate("<KeyPress-p>", when="now"); app.update()
check(app.paused, "P 暫停")
app.event_generate("<KeyPress-p>", when="now"); app.update()
app.bot.set_cmd_vel(0.8, 0.0, 0.5)
app.bot.step(0.5)
moved = abs(app.bot.true_pose[0]) + abs(app.bot.true_pose[1]) > 0.01
# when="now" 的 handler 是同步執行的，所以在 update() 之前就檢查；
# 一旦 update() 跑到 tick()，車子又會依殘餘輪速前進，那是正常行為
app.event_generate("<KeyPress-r>", when="now")
check(moved and app.bot.true_pose == [0.0, 0.0, 0.0] and not app.bot.true_trail,
      "R 重置回原點並清空軌跡")
app.bot.set_cmd_vel(0.0, 0.0, 0.0)
app.update()

print("\n[8] 端到端：實際灌 key event 走完 bind -> on_key_press -> read_cmd")
# 字母鍵 event_generate 合成得出來，用它證明事件接線本身是通的
# （KP_* 合成不出來是 Tk 在 Windows 的限制，不是程式的問題）
app.keys.clear()
app.speed_scale = 1.0
app.event_generate("<KeyPress-w>", when="now")
app.update()
got = app.read_cmd()
check(close(got, (V, 0, 0)), f"灌 <KeyPress-w> 之後 read_cmd 回 {got}")
app.keys.clear()
app.event_generate("<KeyPress-q>", when="now")
app.update()
got = app.read_cmd()
check(close(got, (0, 0, W)), f"灌 <KeyPress-q> 之後 read_cmd 回 {got}")

print("\n[9] 對應表健檢")
TRANSLATE = swerve_sim.TRANSLATE_KEYS
dup = set(TRANSLATE) & set(swerve_sim.ROTATE_KEYS)
check(not dup, f"平移鍵和旋轉鍵沒有重複定義（重複：{dup or '無'}）")
check(not (set(TRANSLATE) & swerve_sim.STOP_KEYS), "平移鍵沒有和停止鍵重疊")
move = set(TRANSLATE) | set(swerve_sim.ROTATE_KEYS) | swerve_sim.STOP_KEYS
speed = swerve_sim.SPEED_UP_KEYS | swerve_sim.SPEED_DOWN_KEYS
check(not (move & speed), f"移動鍵沒有和調速鍵重疊（重複：{move & speed or '無'}）")
for name, need in [
    ("KP_ 系列（X11 / NumLock 開）",
     {"KP_0", "KP_1", "KP_2", "KP_3", "KP_4", "KP_5", "KP_6", "KP_7", "KP_8", "KP_9",
      "KP_Decimal"}),
    ("NumLock 關的方向鍵系列",
     {"KP_Up", "KP_Down", "KP_Left", "KP_Right", "KP_Home", "KP_Prior",
      "KP_End", "KP_Next", "KP_Insert", "KP_Delete", "KP_Begin"}),
    ("純數字系列（Tk on Windows 實測會給這組）",
     {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "period"}),
]:
    check(need <= move, f"{name} 全部有對應（缺：{need - move or '無'}）")

print("\n[10] 實測 Tk 在這台機器上把數字鍵盤的 VK 翻成什麼 keysym")
# Windows 上 event.keycode 就是 VK code，這條路走的是 Tk 真正的鍵盤轉換，
# 不是我自己編的對照表。不管它吐哪一種，對應表都要接得住。
VK = {0x60: "0", 0x61: "1", 0x62: "2", 0x63: "3", 0x64: "4", 0x65: "5",
      0x66: "6", 0x67: "7", 0x68: "8", 0x69: "9", 0x6E: "小數點",
      0x6B: "加號", 0x6D: "減號"}
seen = {}
app.bind("<KeyPress>", lambda e: seen.__setitem__(e.keycode, e.keysym))
for vk in VK:
    app.event_generate("<KeyPress>", keycode=vk, when="now")
app.update()
app.bind("<KeyPress>", app.on_key_press)  # 綁回去

known = move | speed
uncovered = []
for vk, label in VK.items():
    ks = seen.get(vk)
    if ks is None:
        continue
    if ks not in known:
        uncovered.append((label, ks))
print("        " + "  ".join(f"{VK[v]}->{seen.get(v, '?')}" for v in sorted(VK)))
check(not uncovered,
      f"Tk 吐出來的 keysym 全部在對應表裡（沒接住的：{uncovered or '無'}）")

app.destroy()

print("\n" + "=" * 64)
if fails:
    print(f"失敗 {len(fails)} 項：")
    for f in fails:
        print("   - " + f)
    sys.exit(1)
print("全部通過")
