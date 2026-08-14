"""
Swerve 底盤 2D 模擬器（tkinter，無外部相依）
================================================================================

用數字鍵盤操控，即時看到：
  * 四個轉向模組的「機械角度」與「車體角度」，以及各自的可達行程扇形
  * 逆運動學挑的是正解還是反向解（輪速取負）
  * 車子的真實軌跡 vs 里程計推算軌跡

跑法：  python swerve_sim.py
打包：  python build_exe.py       （產生 dist/SwerveSim.exe）
"""

import math
import os
import time
import tkinter as tk

from swerve_model import (
    N, NAMES, SwerveRobot, MOUNT_OFFSET_DEG,
    STEER_ANGLE_MIN_DEG, STEER_ANGLE_MAX_DEG, STEER_ANGLE_MARGIN_DEG,
    STEER_KEEPOUT_DEG, STEER_SOFT_LIMIT_DEG, STRIKER_OFFSET_DEG,
    HOMING_PHASE_NAME,
    MAX_LINEAR_SPEED_MPS, MAX_ANGULAR_SPEED_RPS,
    CHASSIS_WHEELBASE_M, CHASSIS_TRACK_M, TRAIL_STEP_M,
)

TRAIL_MIN_PX = 3.0     # 軌跡上相鄰兩點在螢幕上至少要差這麼多 px 才值得畫
TRAIL_MAX_DRAW = 500   # 單條軌跡最多畫幾個點

# ------------------------------------------------------------------ 外觀
BG = "#14171c"
PANEL = "#1c2027"
EDGE = "#2e3540"
FG = "#d8dee9"
DIM = "#7a8494"
ACCENT = "#4fa8ff"
GREEN = "#5fd68a"
ORANGE = "#ffa64d"
RED = "#ff5f6b"
YELLOW = "#ffd24d"
PURPLE = "#c08cff"
SWITCH = "#3b6fd4"    # 微動開關（對應手繪圖的藍色）
SWPRESS = "#ff2f3c"   # 開關被壓下
STOPBLK = "#5b6472"   # 硬擋塊
STRIKER = "#8fd6ff"   # 隨模組轉動的撞塊

F_UI = ("Microsoft JhengHei UI", 10)
F_SM = ("Microsoft JhengHei UI", 9)
F_BIG = ("Microsoft JhengHei UI", 13, "bold")
F_MONO = ("Consolas", 10)
F_MONO_S = ("Consolas", 9)

W, H = 1320, 792
WORLD = (12, 66, 648, 606)      # x0, y0, x1, y1
CHASSIS = (660, 66, 1308, 606)
TABLE_Y = 616

# 數字鍵盤的 keysym 在不同平台 / NumLock 狀態下會長不一樣，三種都收：
#   KP_8 系列    X11 與部分 Windows 情境（NumLock 開）
#   KP_Up 系列   NumLock 關的時候變成方向 / 編輯鍵
#   "8" 系列     Tk on Windows 實測會直接給純數字，順便讓沒有數字鍵盤的筆電也能玩
TRANSLATE_KEYS = {
    "KP_8": (1, 0), "KP_Up": (1, 0), "8": (1, 0), "w": (1, 0), "Up": (1, 0),
    "KP_2": (-1, 0), "KP_Down": (-1, 0), "2": (-1, 0), "s": (-1, 0), "Down": (-1, 0),
    "KP_4": (0, 1), "KP_Left": (0, 1), "4": (0, 1), "a": (0, 1), "Left": (0, 1),
    "KP_6": (0, -1), "KP_Right": (0, -1), "6": (0, -1), "d": (0, -1), "Right": (0, -1),
    "KP_7": (1, 1), "KP_Home": (1, 1), "7": (1, 1),
    "KP_9": (1, -1), "KP_Prior": (1, -1), "9": (1, -1),
    "KP_1": (-1, 1), "KP_End": (-1, 1), "1": (-1, 1),
    "KP_3": (-1, -1), "KP_Next": (-1, -1), "3": (-1, -1),
}
ROTATE_KEYS = {
    "KP_0": 1.0, "KP_Insert": 1.0, "0": 1.0, "q": 1.0,             # 逆時針
    "KP_Decimal": -1.0, "KP_Delete": -1.0, "period": -1.0, "e": -1.0,  # 順時針
}
STOP_KEYS = {"KP_5", "KP_Begin", "KP_Clear", "5", "space"}
SPEED_UP_KEYS = {"KP_Add", "plus", "equal"}
SPEED_DOWN_KEYS = {"KP_Subtract", "minus"}

KEY_HOLD_S = 0.18  # 超過這麼久沒收到 KeyPress 就當作放開（吃掉自動重複的空窗）
TICK_MS = 25       # 畫面更新間隔（40 fps）。整張畫布每幀重建約要 25 ms，
                   # 排程週期設得比它短只會讓迴圈一直追不上，反而更抖

# 診斷用：設成檔名就會把每一個收到的 keysym 記進去，用來確認鍵盤送出什麼
KEYLOG = os.environ.get("SWERVE_KEYLOG", "")


SWEEP_SECONDS = 30.0


def sweep_cmd(t):
    """全方向掃描自檢的指令軌跡。

    目的是把「解會不會被逼到貼著微動開關」這件事測出來，所以要涵蓋：
    平移方向繞完整兩圈、平移疊加自轉、純自轉正反轉、以及會逼出 180 度換解的
    大角度跳變。回傳 (vx, vy, wz)，t 超過 SWEEP_SECONDS 回傳 None 表示做完了。
    """
    v = MAX_LINEAR_SPEED_MPS * 0.7
    w = MAX_ANGULAR_SPEED_RPS * 0.6
    if t < 12.0:                      # 平移方向連續轉兩圈
        a = math.radians(t / 12.0 * 720.0)
        return v * math.cos(a), v * math.sin(a), 0.0
    if t < 20.0:                      # 平移 + 自轉
        a = math.radians((t - 12.0) / 8.0 * 360.0)
        return v * math.cos(a), v * math.sin(a), w * math.sin(a * 2.0)
    if t < 23.0:                      # 純自轉（逆時針）
        return 0.0, 0.0, w
    if t < 26.0:                      # 純自轉（順時針）
        return 0.0, 0.0, -w
    if t < SWEEP_SECONDS:             # 方向大跳變，逼出 180 度換解
        step = int((t - 26.0) / 0.5)
        a = math.radians(step * 137.0)  # 137 度是無理數倍角，跳點不會重複
        return v * math.cos(a), v * math.sin(a), 0.0
    return None


def resolve_cmd(live, speed_scale):
    """一組「正被按住的 keysym」-> (vx, vy, wz)。

    抽成純函式是為了能單獨測試：Tk 在 Windows 上沒辦法用 event_generate 合成
    KP_* 按鍵（keysym 會變成 "??"），所以按鍵對應只能這樣驗。
    """
    if live & STOP_KEYS:
        return 0.0, 0.0, 0.0

    fx = fy = 0.0
    for k in live:
        if k in TRANSLATE_KEYS:
            dx, dy = TRANSLATE_KEYS[k]
            fx += dx
            fy += dy
    wz = sum(ROTATE_KEYS[k] for k in live if k in ROTATE_KEYS)

    mag = math.hypot(fx, fy)
    if mag > 1e-6:
        fx, fy = fx / mag, fy / mag  # 斜走不要比直走快
    v = MAX_LINEAR_SPEED_MPS * speed_scale
    w = MAX_ANGULAR_SPEED_RPS * speed_scale
    return fx * v, fy * v, max(-1.0, min(1.0, wz)) * w


def norm180(deg):
    while deg > 180.0:
        deg -= 360.0
    while deg <= -180.0:
        deg += 360.0
    return deg


class SwerveSim(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Swerve 底盤模擬器 — 安裝偏移角驗證")
        # 置中，並確保在小螢幕上不會有一半跑到畫面外
        sw, sh = self.winfo_screenwidth(), self.winfo_screenheight()
        self.geometry(f"{W}x{H}+{max(0, (sw - W) // 2)}+{max(0, (sh - H) // 2 - 20)}")
        self.configure(bg=BG)
        self.resizable(False, False)

        self.canvas = tk.Canvas(self, width=W, height=H, bg=BG, highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)

        self.bot = SwerveRobot()
        self.speed_scale = 0.6      # 速度倍率，用 +/- 調
        self.keys = {}              # keysym -> 最後一次 KeyPress 的時間
        self.show_help = True
        self.show_trail = True
        self.paused = False
        self.miscal = False         # 是否注入校正誤差
        self.world_scale = 160.0    # px/m
        self.fps = 0.0
        self.sweep_t = None         # 全方向掃描自檢的進度（None = 沒在跑）
        self.sweep_result = None    # (文字, 顏色)

        self.bind("<KeyPress>", self.on_key_press)
        self.bind("<KeyRelease>", self.on_key_release)
        self.canvas.bind("<MouseWheel>", self.on_wheel)
        self.focus_force()

        self._last = time.perf_counter()
        self._tick_id = self.after(TICK_MS, self.tick)

    def stop_loop(self):
        """停掉自動更新迴圈。

        測試裡一定要先呼叫這個再用 update()：update() 會一直處理事件直到清空，
        而 tick 每 TICK_MS 就重新排程一次，只要 draw() 比 TICK_MS 慢就永遠清不完，
        update() 會卡死。正常執行走 mainloop() 不受影響。
        """
        if self._tick_id is not None:
            self.after_cancel(self._tick_id)
            self._tick_id = None

    # ------------------------------------------------------------ 輸入
    def on_key_press(self, ev):
        ks = ev.keysym
        now = time.perf_counter()

        # 鍵盤對應不上時的診斷用：設 SWERVE_KEYLOG=<檔名> 就會把收到的 keysym 記下來
        if KEYLOG:
            with open(KEYLOG, "a", encoding="utf-8") as f:
                f.write(f"{ks}\n")

        if ks in ("r", "R"):
            self.bot.reset()
            self.sweep_t = None
            self.sweep_result = None
            return
        if ks in ("c", "C"):
            # 真的跑一次開機歸零（四個模組從不同起點，含開機就壓在開關上的）
            self.bot.start_homing()
            self.sweep_t = None
            self.sweep_result = None
            return
        if ks in ("x", "X"):
            if self.sweep_t is None:
                self.bot.reset()      # 統計歸零，掃描結果才有意義
                self.sweep_t = 0.0
                self.sweep_result = None
            else:
                self.sweep_t = None   # 中途取消
                self.bot.set_cmd_vel(0, 0, 0)
            return
        if ks in ("o", "O"):
            self.bot.kin.reset_odom()
            self.bot.odom_trail.clear()
            return
        if ks in ("t", "T"):
            self.show_trail = not self.show_trail
            return
        if ks in ("h", "H", "F1"):
            self.show_help = not self.show_help
            return
        if ks in ("p", "P", "Pause"):
            self.paused = not self.paused
            return
        if ks in ("m", "M"):
            # 注入校正誤差：FL 的物理偏移比設定值多 15 度
            self.miscal = not self.miscal
            self.bot.physical_offset_deg = list(MOUNT_OFFSET_DEG)
            if self.miscal:
                self.bot.physical_offset_deg[0] += 15.0
            return
        if ks in SPEED_UP_KEYS:
            self.speed_scale = min(1.0, self.speed_scale + 0.1)
            return
        if ks in SPEED_DOWN_KEYS:
            self.speed_scale = max(0.1, self.speed_scale - 0.1)
            return
        if ks == "Escape":
            self.destroy()
            return

        self.keys[ks] = now

    def on_key_release(self, ev):
        # Windows 的自動重複會夾雜 KeyRelease，所以不直接刪除，
        # 改由 KEY_HOLD_S 逾時判定，避免按住時速度一閃一閃
        pass

    def on_wheel(self, ev):
        f = 1.1 if ev.delta > 0 else 1 / 1.1
        self.world_scale = max(20.0, min(400.0, self.world_scale * f))

    def read_cmd(self):
        now = time.perf_counter()
        live = {k for k, t in self.keys.items() if now - t < KEY_HOLD_S}
        self.keys = {k: t for k, t in self.keys.items() if now - t < KEY_HOLD_S}
        return resolve_cmd(live, self.speed_scale)

    # ------------------------------------------------------------ 主迴圈
    def tick(self):
        now = time.perf_counter()
        dt = now - self._last
        self._last = now
        if dt > 0:
            self.fps = 0.9 * self.fps + 0.1 * (1.0 / dt)

        if not self.paused:
            if self.sweep_t is not None:
                cmd = sweep_cmd(self.sweep_t)
                if cmd is None:
                    self.finish_sweep()
                else:
                    self.bot.set_cmd_vel(*cmd)
                    self.sweep_t += dt
            else:
                self.bot.set_cmd_vel(*self.read_cmd())
            self.bot.step(dt)

        self.draw()

        # after() 是從「這個 callback 跑完」才開始算，所以要扣掉這一幀已經花掉的
        # 時間，否則實際週期會變成 TICK_MS + draw 時間（實測 25+22=47ms，只剩 21fps）
        spent_ms = (time.perf_counter() - now) * 1000.0
        self._tick_id = self.after(max(1, int(TICK_MS - spent_ms)), self.tick)

    def finish_sweep(self):
        self.sweep_t = None
        self.bot.set_cmd_vel(0, 0, 0)
        hits = self.bot.total_switch_hits()
        worst = self.bot.worst_clearance()
        oor = self.bot.out_of_range()
        if hits or oor:
            self.sweep_result = (f"掃描結果：壓到開關 {hits} 次 ← 實車會撞壞", SWPRESS)
        elif worst < STEER_ANGLE_MARGIN_DEG:
            self.sweep_result = (f"掃描結果：沒壓到，但最近只剩 {worst:.1f}°，太險", RED)
        else:
            self.sweep_result = (f"掃描結果：沒壓到開關，最小 clearance {worst:.1f}°", GREEN)

    # ------------------------------------------------------------ 繪圖
    def panel(self, box, title):
        x0, y0, x1, y1 = box
        self.canvas.create_rectangle(x0, y0, x1, y1, fill=PANEL, outline=EDGE)
        self.canvas.create_text(x0 + 12, y0 + 14, text=title, fill=DIM,
                                font=F_SM, anchor="w")

    def draw(self):
        c = self.canvas
        c.delete("all")
        self.draw_header()
        self.draw_world()
        self.draw_chassis()
        self.draw_table()
        if self.show_help:
            self.draw_help()

    # ---------------------------------------------------------- 標題列
    def draw_header(self):
        c = self.canvas
        b = self.bot
        vx, vy, wz = b.cmd
        c.create_text(14, 20, text="Swerve 底盤模擬器", fill=FG, font=F_BIG, anchor="w")
        c.create_text(14, 44, anchor="w", fill=DIM, font=F_SM,
                      text="驗證「繞中心 90° 陣列安裝」的偏移角轉換：機械角 ↔ 車體角")

        info = (f"cmd  vx={vx:+.2f}  vy={vy:+.2f}  wz={wz:+.2f}      "
                f"倍率 {self.speed_scale:.1f}      {self.fps:4.0f} fps")
        c.create_text(W - 14, 20, text=info, fill=FG, font=F_MONO, anchor="e")

        err = b.odom_error()
        col = GREEN if err < 0.02 else (YELLOW if err < 0.2 else RED)
        state = []
        if self.paused:
            state.append(("已暫停", YELLOW))
        if b.homing_fault:
            state.append(("歸零故障，車子不會動", SWPRESS))
        elif not b.ready:
            state.append(("歸零中，主控不給驅動輪出力", ACCENT))
        if self.miscal:
            state.append(("已注入 FL +15° 校正誤差", RED))
        state.append((f"里程計誤差 {err * 100:6.2f} cm", col))
        x = W - 14
        for txt, cc in reversed(state):
            c.create_text(x, 44, text=txt, fill=cc, font=F_MONO_S, anchor="e")
            x -= (len(txt) * 7 + 24)

    # ---------------------------------------------------------- 世界視圖
    def draw_world(self):
        c = self.canvas
        b = self.bot
        x0, y0, x1, y1 = WORLD
        self.panel(WORLD, "世界視圖　（綠=真實軌跡　橘=里程計推算　滾輪縮放）")
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2 + 10
        s = self.world_scale
        px, py = b.true_pose[0], b.true_pose[1]

        def w2s(wx, wy):
            # 世界座標 x 向前(畫面上)、y 向左(畫面左)，鏡頭跟著車子
            return cx - (wy - py) * s, cy - (wx - px) * s

        c.create_rectangle(x0 + 1, y0 + 22, x1 - 1, y1 - 1, outline="", fill="#171b21")

        # 網格
        step = 0.5
        rng = int(max(x1 - x0, y1 - y0) / s / step) + 2
        gx0 = math.floor(px / step) * step
        gy0 = math.floor(py / step) * step
        for i in range(-rng, rng + 1):
            ax, ay = w2s(gx0 + i * step, gy0 - rng * step)
            bx, by = w2s(gx0 + i * step, gy0 + rng * step)
            c.create_line(ax, ay, bx, by, fill="#232932")
            ax, ay = w2s(gx0 - rng * step, gy0 + i * step)
            bx, by = w2s(gx0 + rng * step, gy0 + i * step)
            c.create_line(ax, ay, bx, by, fill="#232932")

        c.create_rectangle(x0 + 1, y0 + 22, x1 - 1, y1 - 1, outline=EDGE)

        # 軌跡
        # 兩件事會讓這裡變成整個畫面的效能瓶頸，都要避開：
        #   1. smooth=True 會對折線做貝茲細分，上千個點時實測要 50 ms 以上，
        #      而點夠密的時候看起來跟直線折線完全一樣，所以不要用
        #   2. 點數會隨行走距離一直長，要依縮放抽點——螢幕上間距不到幾 px
        #      的點畫了也看不出差別
        #   3. dash 不能用在長折線上：同樣 580 個點，實線 3.2 ms、虛線 24 ms。
        #      綠 / 橘 兩個顏色已經夠分辨了，虛線留給車體輪廓那種幾個點的圖形
        if self.show_trail:
            for trail, col in ((b.true_trail, GREEN), (b.odom_trail, ORANGE)):
                if len(trail) < 2:
                    continue
                keep = max(1, int(TRAIL_MIN_PX / max(1e-6, TRAIL_STEP_M * s)))
                keep = max(keep, len(trail) // TRAIL_MAX_DRAW + 1)
                pts = []
                for k in range(0, len(trail), keep):
                    pts += list(w2s(*trail[k]))
                pts += list(w2s(*trail[-1]))  # 最後一點一定要畫到，不然會斷在車子後面
                if len(pts) >= 4:
                    c.create_line(*pts, fill=col, width=2)

        # 里程計推算的車體位置（虛線輪廓）
        o = b.kin.odom
        self.draw_robot(w2s, o['x'], o['y'], o['yaw'], s, ORANGE, ghost=True)
        # 真實車體
        self.draw_robot(w2s, b.true_pose[0], b.true_pose[1], b.true_pose[2], s, GREEN)

        # 比例尺
        c.create_line(x0 + 20, y1 - 20, x0 + 20 + s * 0.5, y1 - 20, fill=DIM, width=2)
        c.create_text(x0 + 20 + s * 0.25, y1 - 32, text="0.5 m", fill=DIM, font=F_SM)

        tv = b.true_vel
        c.create_text(x1 - 12, y0 + 38, anchor="ne", fill=DIM, font=F_MONO_S,
                      text=f"真實  x={b.true_pose[0]:+.3f}  y={b.true_pose[1]:+.3f}  "
                           f"yaw={math.degrees(b.true_pose[2]):+7.2f}°")
        c.create_text(x1 - 12, y0 + 54, anchor="ne", fill=ORANGE, font=F_MONO_S,
                      text=f"里程  x={o['x']:+.3f}  y={o['y']:+.3f}  "
                           f"yaw={math.degrees(o['yaw']):+7.2f}°")
        c.create_text(x1 - 12, y0 + 70, anchor="ne", fill=DIM, font=F_MONO_S,
                      text=f"實際速度  vx={tv[0]:+.2f}  vy={tv[1]:+.2f}  wz={tv[2]:+.2f}")

    def draw_robot(self, w2s, wx, wy, yaw, s, col, ghost=False):
        """在世界視圖裡畫一台車（含四個輪子的實際朝向）"""
        c = self.canvas
        b = self.bot
        hx, hy = CHASSIS_WHEELBASE_M * 0.62, CHASSIS_TRACK_M * 0.62
        cs, sn = math.cos(yaw), math.sin(yaw)

        def body2world(bx, by):
            return wx + bx * cs - by * sn, wy + bx * sn + by * cs

        corners = [(+hx, +hy), (+hx, -hy), (-hx, -hy), (-hx, +hy)]
        pts = []
        for bx, by in corners:
            pts += list(w2s(*body2world(bx, by)))
        c.create_polygon(*pts, outline=col, fill="", width=2,
                         dash=(4, 3) if ghost else None)

        # 車頭指示
        nx, ny = w2s(*body2world(hx * 1.35, 0))
        mx, my = w2s(*body2world(hx, 0))
        c.create_line(mx, my, nx, ny, fill=col, width=2, arrow="last")

        if ghost:
            return

        # 四個輪子，畫在真實朝向上
        wl = 0.09
        for i in range(N):
            mxb, myb = b.kin.module_x[i], b.kin.module_y[i]
            g = math.radians(b.module_global_deg(i))
            ax, ay = body2world(mxb - math.cos(g) * wl, myb - math.sin(g) * wl)
            bx2, by2 = body2world(mxb + math.cos(g) * wl, myb + math.sin(g) * wl)
            p1 = w2s(ax, ay)
            p2 = w2s(bx2, by2)
            spd = b.wheel_act_mps[i]
            wc = ACCENT if abs(spd) < 1e-3 else (GREEN if spd > 0 else RED)
            c.create_line(*p1, *p2, fill=wc, width=4, capstyle="round")

    # ---------------------------------------------------------- 底盤視圖
    @staticmethod
    def polar(sx, sy, g_deg, r):
        """車體角度 g（度，逆時針為正）+ 半徑 -> 螢幕座標。
        車體 +x（車頭）在螢幕上朝正上方，+y（左）朝螢幕左方。"""
        g = math.radians(g_deg)
        return sx - math.sin(g) * r, sy - math.cos(g) * r

    def rot_block(self, sx, sy, g_deg, r_center, w, h):
        """在半徑 r_center、車體角度 g 的位置放一個小方塊，方塊會跟著角度轉。
        w = 切線方向寬度，h = 徑向厚度。回傳 polygon 的座標串。"""
        g = math.radians(g_deg)
        rx, ry = -math.sin(g), -math.cos(g)     # 徑向單位向量（螢幕座標）
        tx, ty = math.cos(g), -math.sin(g)      # 切線方向
        ox, oy = sx + rx * r_center, sy + ry * r_center
        pts = []
        for sw, sh in ((-1, -1), (+1, -1), (+1, +1), (-1, +1)):
            pts += [ox + tx * (w / 2) * sw + rx * (h / 2) * sh,
                    oy + ty * (w / 2) * sw + ry * (h / 2) * sh]
        return pts

    def draw_chassis(self):
        c = self.canvas
        b = self.bot
        x0, y0, x1, y1 = CHASSIS
        self.panel(CHASSIS, "底盤視圖（車體座標）　灰扇形=撞塊可走的 270°　"
                            "紅斜線=到不了的 90° 死角　彩色弧=離兩顆開關各剩幾度")
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2 + 6
        s = 700.0   # px/m
        rad = 64.0  # 模組外圈半徑 (px)
        R_STOP = rad * 1.22     # 硬擋塊
        R_SW = rad * 1.06       # 微動開關
        R_BAND = rad * 0.99     # keep-out / 安全邊界色帶
        R_TRK = rad * 0.86      # 撞塊軌道
        R_WARC = rad * 0.56     # 輪子可指向範圍
        R_WHEEL = rad * 0.48    # 輪子半長

        def b2s(bx, by):
            return cx - by * s, cy - bx * s

        # 車體外框：直接用四個模組的位置當四個角
        hx, hy = CHASSIS_WHEELBASE_M * 0.5, CHASSIS_TRACK_M * 0.5
        p = [b2s(+hx, +hy), b2s(+hx, -hy), b2s(-hx, -hy), b2s(-hx, +hy)]
        c.create_polygon(*[v for pt in p for v in pt], outline=EDGE, fill="#171b21", width=2)

        # 車頭方向（四個模組圓之間的空隙很窄，文字一律往中央擺才不會被蓋住）
        c.create_line(cx, cy + 6, cx, cy - 58, fill=ACCENT, width=3, arrow="last")
        c.create_text(cx, cy + 20, text="車頭 (車體 0°)", fill=ACCENT, font=F_SM)
        c.create_line(cx - 6, cy, cx - 58, cy, fill=DIM, width=1, arrow="last")
        c.create_text(cx - 60, cy, text="+y (左)", fill=DIM, font=F_SM, anchor="e")

        for i in range(N):
            mx, my = b.kin.module_x[i], b.kin.module_y[i]
            sx, sy = b2s(mx, my)
            off = MOUNT_OFFSET_DEG[i]          # 機械角 -> 輪子朝向
            soff = off + STRIKER_OFFSET_DEG[i]  # 機械角 -> 撞塊 / 微動開關位置
            mod = b.steer[i]

            mech = mod.angle_deg
            tgt = b.target_deg[i]

            def clr_col(v):
                return (SWPRESS if v <= 0.05 else RED if v < STEER_ANGLE_MARGIN_DEG
                        else YELLOW if v < STEER_KEEPOUT_DEG else GREEN)

            def arc(r, a_from, extent, col, width, style="arc"):
                c.create_arc(sx - r, sy - r, sx + r, sy + r,
                             start=a_from + 90.0, extent=extent,
                             style=style, outline=col, width=width)

            # 底盤：模組佔位圓
            c.create_oval(sx - rad, sy - rad, sx + rad, sy + rad,
                          fill="#1d232c", outline="#2f3743")

            # ---- 限位機構：一根撞塊 + 兩顆開關 -----------------------------
            # 撞塊在兩顆開關之間繞「另外那一邊」走 270 度，中間 90 度的缺口
            # 永遠不會經過，所以不會誤觸另一顆開關。
            a0 = soff + STEER_ANGLE_MIN_DEG   # min 開關（機械 0°）
            a1 = soff + STEER_ANGLE_MAX_DEG   # max 開關（機械 270°）
            span = STEER_ANGLE_MAX_DEG - STEER_ANGLE_MIN_DEG

            # 到不了的 90 度死角（朝向車體中心）
            c.create_arc(sx - rad, sy - rad, sx + rad, sy + rad,
                         start=a1 + 90.0, extent=360.0 - span,
                         style="pieslice", outline="#7a2f3a", fill="#2a1218")
            for f in range(1, 6):
                a = a1 + (360.0 - span) * f / 6.0
                c.create_line(*self.polar(sx, sy, a, rad * 0.16),
                              *self.polar(sx, sy, a, rad * 0.97),
                              fill="#7a2f3a", width=1)

            # 撞塊可以走的 270 度
            c.create_arc(sx - rad, sy - rad, sx + rad, sy + rad,
                         start=a0 + 90.0, extent=span,
                         style="pieslice", outline="#4a5464", fill="#252d3a")

            # 撞塊軌道；兩段彩色弧 = 離兩顆開關各還剩幾度，加起來永遠是 270
            arc(R_TRK, a0, span, "#2b3341", 3)
            left_min = mech - STEER_ANGLE_MIN_DEG     # 離 0° 開關還剩幾度
            left_max = STEER_ANGLE_MAX_DEG - mech     # 離 270° 開關還剩幾度
            arc(R_TRK, a0, left_min, clr_col(left_min), 7)
            arc(R_TRK, soff + mech, left_max, clr_col(left_max), 7)

            # 兩端的 keep-out 帶與安全邊界
            for base_a, sgn in ((a0, +1.0), (a1, -1.0)):
                m0 = base_a + sgn * STEER_ANGLE_MARGIN_DEG
                arc(R_BAND, min(base_a, m0), STEER_ANGLE_MARGIN_DEG, "#c0303f", 8)
                arc(R_BAND, min(m0, m0 + sgn * STEER_KEEPOUT_DEG),
                    STEER_KEEPOUT_DEG, "#8a6a1e", 8)

            # 硬擋塊 + 微動開關（固定在車架上，不會轉）
            for a_sw, lbl, pressed in ((a0, "0°", mod.lim_min),
                                       (a1, "270°", mod.lim_max)):
                c.create_polygon(*self.rot_block(sx, sy, a_sw, R_STOP, 22, 11),
                                 fill=STOPBLK, outline="#8b95a6")
                c.create_polygon(*self.rot_block(sx, sy, a_sw, R_SW, 16, 11),
                                 fill=(SWPRESS if pressed else SWITCH), outline="#0f1216")
                c.create_line(*self.polar(sx, sy, a_sw, R_SW - 6),
                              *self.polar(sx, sy, a_sw, R_TRK),
                              fill=(SWPRESS if pressed else SWITCH), width=2)
                lx, ly = self.polar(sx, sy, a_sw, R_STOP + 13)
                c.create_text(lx, ly, text=lbl, font=("Consolas", 8),
                              fill=(SWPRESS if pressed else "#7f93bf"))

            # 目標位置（空心）與撞塊本體
            c.create_polygon(*self.rot_block(sx, sy, soff + tgt, R_TRK, 11, 9),
                             fill="", outline=DIM, width=2)
            c.create_polygon(*self.rot_block(sx, sy, soff + mech, R_TRK, 18, 13),
                             fill=clr_col(mod.limit_clearance_deg),
                             outline="#e8eef7", width=2)

            # ---- 輪子 -------------------------------------------------------
            # 紫弧 = 輪子能指向的角度範圍，輪子的粗端永遠落在這條弧上
            arc(R_WARC, STEER_ANGLE_MIN_DEG + off,
                STEER_ANGLE_MAX_DEG - STEER_ANGLE_MIN_DEG, "#7a56ab", 5)

            g = b.module_global_deg(i)
            spd = b.wheel_act_mps[i]
            wc = ACCENT if abs(spd) < 1e-3 else (GREEN if spd > 0 else RED)
            c.create_line(*self.polar(sx, sy, g, -R_WHEEL),
                          *self.polar(sx, sy, g, R_WHEEL * 0.5),
                          fill=wc, width=4, capstyle="round")
            c.create_line(*self.polar(sx, sy, g, R_WHEEL * 0.5),
                          *self.polar(sx, sy, g, R_WHEEL),
                          fill=wc, width=8, capstyle="round")

            # 實際行進方向（輪速為負時指向輪子的另一端）
            if abs(spd) > 1e-3:
                al = R_WARC * (0.5 + 0.45 * min(1.0, abs(spd) / 2.0))
                if spd < 0:
                    al = -al
                c.create_line(sx, sy, *self.polar(sx, sy, g, al),
                              fill=YELLOW, width=3, arrow="last")

            # 真的出事才圈起來：頂到硬擋塊，或是「已經歸零、正常運轉」卻跑出行程。
            # 歸零過程中撞塊會進到開關之後的 overtravel 區，那是正常的，不算。
            running = mod.homed and not mod.fault
            if mod.jammed or (running and not (STEER_ANGLE_MIN_DEG - 1e-6 <= mech
                                               <= STEER_ANGLE_MAX_DEG + 1e-6)):
                c.create_oval(sx - rad - 16, sy - rad - 16, sx + rad + 16, sy + rad + 16,
                              outline=SWPRESS, width=3)

            c.create_oval(sx - 4, sy - 4, sx + 4, sy + 4, fill=EDGE, outline=DIM)
            clr = mod.limit_clearance_deg
            sc = clr_col(clr)

            # --- 模組標籤：前排的字放上面、後排放下面，中間留空 ---
            front = mx > 0
            base = sy - R_STOP if front else sy + R_STOP
            d = -1 if front else 1
            c.create_text(sx, base + d * 64, text=NAMES[i], fill=FG, font=F_BIG)
            c.create_text(sx, base + d * 46, fill=DIM, font=F_MONO_S,
                          text=f"offset {off:+.0f}°")
            c.create_text(sx, base + d * 31, font=F_MONO_S, fill=FG,
                          text=f"機械 {mod.angle_deg:6.1f}°")
            c.create_text(sx, base + d * 17, font=F_MONO_S, fill=PURPLE,
                          text=f"車體 {norm180(g):+6.1f}°")
            if mod.homed and not mod.fault:
                c.create_text(sx, base + d * 3, font=F_MONO_S, fill=sc,
                              text=f"離開關 {clr:5.1f}°")
            else:
                # 歸零中：顯示現在跑到哪一階段
                c.create_text(sx, base + d * 3, font=F_MONO_S,
                              fill=SWPRESS if mod.fault else ACCENT,
                              text=("歸零故障" if mod.fault
                                    else HOMING_PHASE_NAME[mod.phase]))

            c.create_oval(sx - 4, sy - 4, sx + 4, sy + 4, fill=EDGE, outline=DIM)

    # ---------------------------------------------------------- 數值表
    def draw_table(self):
        c = self.canvas
        b = self.bot
        y = TABLE_Y
        c.create_rectangle(12, y, W - 12, H - 12, fill=PANEL, outline=EDGE)

        cols = [(28, "模組"), (92, "機械角"), (178, "目標"), (258, "誤差"),
                (338, "車體角"), (424, "離開關"), (506, "輪速"), (580, "解"),
                (644, "開關"), (712, "PWM"), (790, "軟限位"), (866, "到位")]
        for cxp, name in cols:
            c.create_text(cxp, y + 18, text=name, fill=DIM, font=F_SM, anchor="w")
        c.create_line(20, y + 30, 940, y + 30, fill=EDGE)

        for i in range(N):
            ry = y + 50 + i * 22
            m = b.steer[i]
            err = b.target_deg[i] - m.angle_deg
            spd = b.wheel_act_mps[i]
            raw = b.wheel_raw_mps[i]
            clr = m.limit_clearance_deg
            ccol = (SWPRESS if clr <= 0.05 else RED if clr < STEER_ANGLE_MARGIN_DEG
                    else YELLOW if clr < STEER_KEEPOUT_DEG else GREEN)
            flip = "反向" if raw < -1e-6 else ("正解" if raw > 1e-6 else "—")
            fcol = RED if raw < -1e-6 else (GREEN if raw > 1e-6 else DIM)
            lim = ("MIN " if m.lim_min else "") + ("MAX" if m.lim_max else "")
            vals = [
                (28, NAMES[i], FG),
                (92, f"{m.angle_deg:7.2f}°", FG),
                (178, f"{b.target_deg[i]:7.2f}°", DIM),
                (258, f"{err:+7.2f}°", YELLOW if abs(err) > 2 else DIM),
                (338, f"{norm180(b.module_global_deg(i)):+7.2f}°", PURPLE),
                (424, f"{clr:6.1f}°", ccol),
                (506, f"{spd:+6.3f}", GREEN if spd > 0 else (RED if spd < 0 else DIM)),
                (580, flip, fcol),
                (644, lim or "—", SWPRESS if lim else DIM),
                (712, f"{m.pwm:+6.0f}", DIM),
                (790, "減速中" if m.soft_limited else "—", YELLOW if m.soft_limited else DIM),
                (866, "✓" if m.at_target else "…", GREEN if m.at_target else DIM),
            ]
            for cxp, txt, col in vals:
                c.create_text(cxp, ry, text=txt, fill=col, font=F_MONO, anchor="w")

        self.draw_safety_panel(960, y)

    def draw_safety_panel(self, px, y):
        """右側：限位安全摘要 + 正運動學閉環自檢"""
        c = self.canvas
        b = self.bot
        hits = b.total_switch_hits()
        worst = b.worst_clearance()

        c.create_line(px - 16, y + 8, px - 16, H - 20, fill=EDGE)
        c.create_text(px, y + 18, anchor="w", fill=DIM, font=F_SM, text="限位安全")

        col = GREEN if hits == 0 else SWPRESS
        c.create_text(px, y + 44, anchor="w", font=F_MONO, fill=col,
                      text=f"運轉中壓到開關  {hits} 次"
                           + ("   ← 實車會撞壞" if hits else ""))
        jam = b.total_hard_stop_hits()
        hom = b.total_homing_switch_hits()
        c.create_text(px + 300, y + 44, anchor="w", font=F_MONO_S,
                      fill=SWPRESS if jam else DIM,
                      text=f"頂到硬擋塊 {jam}　歸零碰開關 {hom}")
        if worst > 900.0:   # 還沒開始正常運轉，這個數字沒有意義
            c.create_text(px, y + 64, anchor="w", font=F_MONO, fill=DIM,
                          text="全程最小 clearance    --   （歸零完才開始統計）")
        else:
            wcol = (SWPRESS if worst <= 0.05 else RED if worst < STEER_ANGLE_MARGIN_DEG
                    else YELLOW if worst < STEER_KEEPOUT_DEG else GREEN)
            c.create_text(px, y + 64, anchor="w", font=F_MONO, fill=wcol,
                          text=f"全程最小 clearance  {worst:5.1f}°")
        c.create_text(px, y + 84, anchor="w", font=F_MONO_S, fill=DIM,
                      text=f"keep-out {STEER_KEEPOUT_DEG:.0f}°  "
                           f"安全邊界 {STEER_ANGLE_MARGIN_DEG:.0f}°  "
                           f"軟限位 {STEER_SOFT_LIMIT_DEG:.0f}°")

        # 掃描自檢
        if self.sweep_t is not None:
            pct = min(100.0, self.sweep_t / SWEEP_SECONDS * 100.0)
            c.create_text(px, y + 110, anchor="w", font=F_MONO, fill=ACCENT,
                          text=f"全方向掃描中… {pct:3.0f}%")
            bw = 300
            c.create_rectangle(px, y + 122, px + bw, y + 130, outline=EDGE)
            c.create_rectangle(px, y + 122, px + bw * pct / 100.0, y + 130,
                               outline="", fill=ACCENT)
        elif self.sweep_result:
            c.create_text(px, y + 110, anchor="w", font=F_MONO,
                          fill=self.sweep_result[1], text=self.sweep_result[0])
        else:
            c.create_text(px, y + 110, anchor="w", font=F_MONO_S, fill=DIM,
                          text="按 X 跑全方向掃描，自動確認會不會壓到開關")

        # 正運動學閉環自檢
        rvx, rvy, rwz = b.kin.forward(b.reported_deg, b.wheel_act_mps)
        tv = b.true_vel
        e = max(abs(rvx - tv[0]), abs(rvy - tv[1]), abs(rwz - tv[2]))
        c.create_text(px, H - 30, anchor="w", font=F_MONO_S,
                      fill=GREEN if e < 1e-6 else RED,
                      text=f"正運動學閉環殘差 {e:.2e}  "
                           f"{'一致' if e < 1e-6 else '不一致（偏移角設錯）'}")

    # ---------------------------------------------------------- 說明
    def draw_help(self):
        c = self.canvas
        lines = [
            ("數字鍵盤", ACCENT),
            ("  8 2 4 6    前 後 左 右", FG),
            ("  7 9 1 3    四個斜向", FG),
            ("  0 / .      逆時針 / 順時針自轉", FG),
            ("  5          全停", FG),
            ("  + -        速度倍率", FG),
            ("", FG),
            ("其他", ACCENT),
            ("  C 跑一次開機歸零", YELLOW),
            ("  X 全方向掃描自檢", YELLOW),
            ("  R 重置    O 里程計歸零", FG),
            ("  T 軌跡    P 暫停", FG),
            ("  M 注入 FL +15° 校正誤差", FG),
            ("  H 開關這個說明   ESC 離開", DIM),
            ("", FG),
            ("底盤視圖圖例", ACCENT),
            ("  亮塊=撞塊（輪叉上的銷，隨模組轉）", FG),
            ("  藍塊=微動開關  灰塊=硬擋塊", SWITCH),
            ("  灰扇形=撞塊可走的 270°", FG),
            ("  紅斜線=到不了的 90° 死角", RED),
            ("  彩色弧=離兩顆開關各剩幾度", GREEN),
            ("  紫弧=輪子可指向的範圍", PURPLE),
            ("  黃帶=程式主動避開  紅帶=禁區", YELLOW),
        ]
        bx, by = WORLD[0] + 14, WORLD[1] + 34
        bw, bh = 232, len(lines) * 17 + 16
        c.create_rectangle(bx, by, bx + bw, by + bh, fill="#11151a", outline=EDGE)
        for k, (txt, col) in enumerate(lines):
            c.create_text(bx + 10, by + 14 + k * 17, text=txt, fill=col,
                          font=F_SM, anchor="w")


if __name__ == "__main__":
    try:  # Windows 高 DPI 下不要糊掉
        from ctypes import windll
        windll.shcore.SetProcessDpiAwareness(1)
    except Exception:
        pass
    SwerveSim().mainloop()
