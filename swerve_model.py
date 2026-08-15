"""
Swerve 底盤模擬：韌體邏輯的 1:1 Python 移植
================================================================================

這支檔案不含任何 GUI，純粹把兩塊板子上的 C++ 邏輯照抄成 Python，用來驗證
「安裝偏移角 + 270 度行程 + 反向解」這套座標轉換到底通不通：

    SwerveKinematics   <-  Esp32_wheel/lib/SwerveKinematics/SwerveKinematics.cpp
    SteerBoardModule   <-  esp32_rad/lib/SteerModule/SteerModule.cpp（位置環部分）
    SwerveRobot        <-  Esp32_wheel/src/bot.cpp 的 loop_bot_control()

移植原則是「連 bug 都要一起抄」——公式、判斷順序、飽和處理都跟 C++ 對齊，
不要在這裡「順手改好一點」，否則模擬過了實車還是會壞。

另外多做一件韌體沒有的事：同時維護「設定值偏移」與「物理真實偏移」兩套角度。
兩者相同時里程計會完全貼合真實軌跡；把設定值調錯（模擬校正沒做好）就會看到
里程計慢慢飄掉，可以用來判斷偏移角填錯時的症狀長什麼樣。
"""

import math

N = 4
NAMES = ("FL", "FR", "RL", "RR")

# ------------------------------------------------------------------ 參數
# 對應 Esp32_wheel/include/swerve_config.h
CHASSIS_WHEELBASE_M = 0.200
CHASSIS_TRACK_M = 0.200
# HB-116 輪轂馬達，8S 電池 + VESC 佔空比 71%，按電量末端設上限（見 swerve_config.h）
WHEEL_DIAMETER_M = 0.116
MAX_WHEEL_SPEED_MPS = 1.1
MAX_LINEAR_SPEED_MPS = 1.0
MAX_ANGULAR_SPEED_RPS = 2.0

STEER_ANGLE_MIN_DEG = 0.0
STEER_ANGLE_MAX_DEG = 270.0
STEER_ANGLE_MARGIN_DEG = 3.0

# 主動避開限位開關：挑解時把窗口收緊成 [3+30, 267-30]，硬性排除靠近開關的解
STEER_KEEPOUT_DEG = 30.0

# 換解遲滯：要換到另一組等價解（θ vs θ+180），它得比目前這組近上這麼多度。
# 目前角度離兩個解各 90 度時兩者成本相等，指令抖一點就會讓贏家換人，而換人
# 等於轉 180 度 + 驅動輪反轉。詳見 swerve_config.h。
STEER_FLIP_HYSTERESIS_DEG = 30.0

# ---------------------------------------------------------- 轉向到位閘門
# 輪子還沒轉到位就給油會橫著刮地，車體被側向力推歪。餘弦補償在小誤差時
# 幾乎不打折（差 10 度還給 98.5%），所以再加一道硬閘門。兩個門檻不同 = 遲滯。
DRIVE_ALIGN_GATE_DEG = 8.0
DRIVE_ALIGN_RELEASE_DEG = 4.0
DRIVE_ALIGN_GATE_LINKED = True   # True = 任一顆沒到位就四顆全切

# ------------------------------------------------------ 加減速斜率限制
# /cmd_vel 是階躍的，直接餵給跑速度閉環的 VESC 就是全電流起步 / 全力煞車，
# 車體會往後坐再往前點頭。限在**底盤速度**這一層，四顆才會等比例地一起長，
# 運動方向全程不變形。急停 / 逾時 / 轉向板故障不走斜率，直接歸零。
DRIVE_ACCEL_MPS2 = 1.0
DRIVE_DECEL_MPS2 = 2.0
DRIVE_ANG_ACCEL_RPS2 = 3.0
DRIVE_ANG_DECEL_RPS2 = 6.0
# 大角度變向要先停車再轉。輪胎橫著刮地的原因不是驅動輪在出力，是「車體還在
# 往原方向跑，轉向卻已經開始甩」—— 動量不會因為斷油就消失。所以大變向時先
# 凍結轉向目標，等實際輪速降到這個值以下才放行。0 = 關閉。詳見 swerve_config.h。
DRIVE_REPOINT_SPEED_MPS = 0.05
# 變向要大到幾度才值得把車停下來。和 DRIVE_ALIGN_GATE_DEG 是兩個不同的決策：
# 那個是「誤差多大就不該再給油」，這個是「變向多大才值得整台車停下來重來」。
# 停車的代價約一秒（減速+轉向+重新加速），共用同一個小門檻會讓手動操作
# 一路停停走走。靠 θ/θ+180 兩個解，需要的轉角最多 90 度，所以範圍是 0~90。
DRIVE_REPOINT_ANGLE_DEG = 60.0

# 機械角度 0 度（撞塊壓住 0 度開關）時，輪子在車體座標下指向的方向。
# FL 朝車尾、FR 朝左、RL 朝右、RR 朝車頭 —— 四個模組繞車體中心 90 度陣列的
# 同一種裝法，所以這四個值也是 90 度一階。由開關位置與撞塊偏移推出，見下。
MOUNT_OFFSET_DEG = (-180.0, 90.0, -90.0, 0.0)

# ---------------------------------------------------------------- 限位機構幾何
# 這一段完全不進入任何運算。機械角度的刻度（0 = 碰到 min 開關、270 = 碰到 max
# 開關）本來就已經把機構細節吸收掉了，這裡只是為了把限位機構畫在正確的位置。
#
# 實車機構（見 CAD）：
#   * 輪叉側面「一根」圓柱銷當撞塊，跟著模組一起轉
#   * 車架上垂下兩片微動開關撥桿，兩顆開關相差 90 度
#   * 撞塊在兩顆開關之間繞「另外那一邊」走 270 度，中間那 90 度的缺口
#     它永遠不會經過，所以不會誤觸另一顆開關
#
#   撞塊位置 = 機械角 + MOUNT_OFFSET + STRIKER_OFFSET
#   min 開關 = 撞塊在機械 0 度時的位置
#   max 開關 = 撞塊在機械 270 度時的位置
#
# 這三個量只有兩個自由度，挑兩個當已知第三個就被決定了。這裡的已知是：
#   (a) 開關位置 —— 實車俯視圖量到的，是外部事實
#   (b) STRIKER_OFFSET = +90 —— 撞塊銷相對輪子朝向的角度差，CAD 對圖確認
#       （銷裝在輪叉側面，所以差 90 度），四個模組同值
# 被決定的是 MOUNT_OFFSET（上面那組），四組都對得上量到的開關位置：
#   FL(offset -180)：min = 0+(-180)+90 = -90，max = 270+(-180)+90 = 180  ✓
#   FR(offset  +90)：min = 180，max = 90    ✓
#   RL(offset  -90)：min = 0，  max = -90   ✓
#   RR(offset    0)：min = 90， max = 0     ✓
#
# 附帶一個可以拿來佐證的性質：每個模組那 90 度的死角剛好朝向車體中心，正是
# 開關裝在內側的必然結果。死角是開關位置決定的，換句話說這個佐證只驗 (a)，
# 改 STRIKER_OFFSET 不會動到它。
STRIKER_OFFSET_DEG = (90.0, 90.0, 90.0, 90.0)


def striker_body_deg(i, mech_deg):
    """模組 i 在機械角度 mech_deg 時，限位撞塊在車體座標的角度"""
    return mech_deg + MOUNT_OFFSET_DEG[i] + STRIKER_OFFSET_DEG[i]


def switch_body_deg(i):
    """模組 i 的兩顆微動開關在車體座標的角度 (min 開關, max 開關)"""
    return (striker_body_deg(i, STEER_ANGLE_MIN_DEG),
            striker_body_deg(i, STEER_ANGLE_MAX_DEG))

# 對應 esp32_rad/include/steer_config.h
STEER_PID_KP = 110.0
STEER_PID_KI = 0.0
STEER_PID_KD = 6.0
STEER_MAX_PWM = 800
STEER_MIN_MOVE_PWM = 45       # 2026-08-16 地板承重四顆同步實測，最差 38 加兩成餘裕
STEER_TOLERANCE_DEG = 1.5
STEER_PWM_MAX = 1023
STEER_SOFT_LIMIT_DEG = 10.0   # 軟限位減速區
STEER_HOMING_PARK_DEG = 135.0 # 校正完停在行程正中間

# MD36 P71 24V + 霍爾編碼盤 13 ppr，外加迴轉盤齒圈那一級
STEER_ENCODER_PPR = 13.0
STEER_MOTOR_GEARBOX = 71.0
STEER_RING_STAGE = 4.5        # 實車歸零量出來的（見 steer_config.h 的說明）
STEER_GEAR_RATIO = STEER_MOTOR_GEARBOX * STEER_RING_STAGE
# 韌體開機時用的「估算」刻度。歸零碰到第二顆開關後會用實測值覆寫掉。
STEER_TICKS_PER_DEG_NOMINAL = STEER_ENCODER_PPR * 4.0 * STEER_GEAR_RATIO / 360.0

STEER_HOMING_PWM = 320
STEER_HOMING_SLOW_PWM = 170      # 二次逼近的慢速 PWM，0 = 不做二次逼近
STEER_HOMING_REBOUND_DEG = 4.0   # 退開幾度再慢慢碰第二次
STEER_HOMING_TIMEOUT_MS = 20000
STEER_MEASURE_SPAN = True
STEER_SPAN_TOL_FRAC = 0.25       # 實測刻度和理論值差超過這個比例判故障
LIMIT_DEBOUNCE_MS = 5.0

# ------------------------------------------- 微動開關的傳輸鏈（主控板 -> CAN -> 轉向板）
# 開關實體接在主控板（Esp32_wheel/lib/LimitBank），轉向板靠 CAN 0x080 才知道
# 開關被壓下。這條路徑的延遲會一比一變成歸零零點的誤差，所以要模擬出來。
#
#   主控 200 Hz 掃描 -> 5 ms 去彈跳 -> 有變化立刻送 -> CAN -> 轉向板 200 Hz 消化
#
# 對應 swerve_config.h 的 LIMIT_HEARTBEAT_MS 與 bot.cpp 的 loop_bot_limits()
LIMIT_POLL_HZ = 200.0        # 主控掃描頻率（control_task 每一拍）
LIMIT_CAN_LATENCY_MS = 0.5   # 訊框時間 + 仲裁排隊，500 kbit/s 下約 0.3~0.7 ms
LIMIT_HEARTBEAT_MS = 20.0    # 沒有變化時的重送週期
LIMIT_LATCH_REPEATS = 3      # 一個邊沿連送幾幀才清掉黏著旗標

# 對應 SteerModule.h 的 STEER_LIMIT_AGE_COMPENSATE。
# 關掉就退回「收到就記，不補償」，test_homing.py 第 5 項會把兩者對照出來。
STEER_LIMIT_AGE_COMPENSATE = True

# ------------------------------------------------------------------- 急停
# 對應 swerve_config.h 的 ESTOP_*。急停開關接在主控板的 GPIO 上，觸發後用 CAN
# 命令 VESC 煞車。這裡模擬的重點是**閂鎖與重新致能的邏輯**（那是最容易寫錯、
# 而且寫錯就會讓車子自己跑掉的部分），煞車物理只用一個比較短的時間常數帶過。
#
# 只影響四顆驅動輪。轉向板是直流馬達、有自己獨立的急停迴路，所以急停期間
# 轉向板照常收角度指令，只是速度被壓成零，輪子因此凍結在原地。
ESTOP_LATCH = True                # 放開開關不會自動恢復
ESTOP_REARM_REQUIRE_ZERO = False  # 解除前要不要先等 /cmd_vel 歸零（見下）
ESTOP_REARM_CMD_ZERO_MS = 1000.0  # 上面為 True 時，放開後還要零速維持這麼久
ESTOP_TRIP_DEBOUNCE_MS = 3.0      # 觸發要快
ESTOP_CLEAR_DEBOUNCE_MS = 50.0    # 解除要慢
ESTOP_BRAKE_UNTIL_MPS = 0.05
DRIVE_BRAKE_TAU_S = 0.02          # 煞車電流下的輪速衰減，比 DRIVE_TAU_S 快很多

# ------------------------------------------------- 模擬用的「物理真實」參數
# 這些韌體不知道，是拿來製造「韌體以為的」和「真的是」之間的落差用的。
# 想試更糟的硬體就把數字加大，看歸零還撐不撐得住。
HW_TICKS_PER_DEG = STEER_TICKS_PER_DEG_NOMINAL * 1.04  # 實際刻度比估算值大 4%
HW_BACKLASH_DEG = 1.2            # 輸出側齒隙
HW_HARD_STOP_OVERTRAVEL_DEG = 2.5  # 開關觸發之後還有多少行程才頂到硬擋塊

# 執行模式 / 歸零階段，對應 SwerveCan.h 與 SteerModule.h 的 enum
SM_IDLE, SM_RUN, SM_HOMING, SM_ESTOP = 0, 1, 2, 3
(HOMING_IDLE, HOMING_BACKOFF, HOMING_SEEK_MIN, HOMING_REBOUND_MIN, HOMING_SLOW_MIN,
 HOMING_SEEK_MAX, HOMING_REBOUND_MAX, HOMING_SLOW_MAX, HOMING_RETURN,
 HOMING_DONE, HOMING_FAULT) = range(11)
HOMING_PHASE_NAME = ("閒置", "退離開關", "快速找 0°", "退開", "慢速碰 0°",
                     "快速找 270°", "退開", "慢速碰 270°", "回停放角", "完成", "故障")

# 這兩個 C++ 裡沒有，是模擬用的馬達物理模型。
# 轉向模組轉速 = MD36 減速箱輸出 125 rpm(24V 空載) / 齒圈那一級
#              = 125 / 4.5 = 27.8 rpm = 167 度/秒
STEER_MAX_RATE_DPS = 125.0 / STEER_RING_STAGE * 6.0  # rpm -> deg/s 就是 x6
STEER_TAU_S = 0.05          # 轉向機構的一階慣性時間常數
DRIVE_TAU_S = 0.08          # 驅動輪速度跟隨的一階慣性時間常數

# 軌跡每隔這麼遠才記一個點。畫圖時會再依縮放抽點，所以這裡只要夠細就好。
TRAIL_STEP_M = 0.01
TRAIL_MAX_POINTS = 4000

LOOP_CONTROL_HZ = 50   # 主控 loop_bot_control()：逆運動學 + 送指令
LOOP_ODOM_HZ = 100     # 主控 loop_bot_odom()：里程計積分（和控制迴圈拆開）
LOOP_STEER_HZ = 200    # 轉向板位置環
LOOP_REPORT_HZ = 100   # 轉向板 -> 主控 的角度回報（要 >= LOOP_ODOM_HZ 才有意義）


def constrain(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


# ============================================================================
#  SwerveKinematics：主控板運動學（含這次加的安裝偏移轉換）
# ============================================================================
class SwerveKinematics:
    """1:1 對應 Esp32_wheel/lib/SwerveKinematics/SwerveKinematics.cpp

    內部同時存在兩套角度：
        車體角度 (global) 0 度 = 車頭正前方，逆時針為正
        機械角度 (local)  0 度 = 0 度限位開關的位置，行程 [min, max]
    差一個安裝偏移角：global = local + offset
    """

    def __init__(self):
        self.module_x = [0.0] * N
        self.module_y = [0.0] * N
        self.radius_sum_sq = 1.0
        self.max_wheel_speed = 2.0
        self.steer_min_deg = 0.0
        self.steer_max_deg = 270.0
        self.mount_offset_deg = [0.0] * N
        self.keepout_deg = 0.0
        self.flip_hysteresis_deg = 0.0
        self.last_flip = [0] * N
        self.last_angle_deg = [0.0] * N
        self.seeded = False
        self.odom = dict(x=0.0, y=0.0, yaw=0.0, vx=0.0, vy=0.0, wz=0.0)

    # ---------------------------------------------------------------- 設定
    def set_chassis_param(self, wheelbase_m, track_m, max_wheel_speed_mps):
        hx, hy = wheelbase_m * 0.5, track_m * 0.5
        self.module_x = [+hx, +hx, -hx, -hx]   # 0=FL 1=FR 2=RL 3=RR
        self.module_y = [+hy, -hy, +hy, -hy]   # y 向左為正
        self.radius_sum_sq = sum(x * x + y * y
                                 for x, y in zip(self.module_x, self.module_y))
        if self.radius_sum_sq < 1e-6:
            self.radius_sum_sq = 1e-6
        self.max_wheel_speed = max(max_wheel_speed_mps, 0.01)

    def set_steer_range(self, min_deg, max_deg, margin_deg):
        self.steer_min_deg = min_deg + margin_deg
        self.steer_max_deg = max_deg - margin_deg
        if self.steer_max_deg < self.steer_min_deg:
            self.steer_max_deg = self.steer_min_deg

    def set_mount_offsets(self, offset_deg):
        self.mount_offset_deg = list(offset_deg)

    def set_limit_keepout(self, keepout_deg):
        """挑解時離微動開關至少要留幾度。必須在 set_steer_range() 之後呼叫。"""
        self.keepout_deg = max(0.0, keepout_deg)
        # 收緊後的窗口至少要有 180 度，否則會有方向兩個解都被擋掉
        max_keepout = (self.steer_max_deg - self.steer_min_deg - 180.0) * 0.5
        self.keepout_deg = min(self.keepout_deg, max(0.0, max_keepout))

    def set_flip_hysteresis(self, deg):
        """換到另一組等價解要「近上這麼多度」才算數（0 = 關閉）。

        目前角度離兩個解各 90 度時兩者成本相等，指令抖一點就會讓贏家換人，
        而換人 = 轉 180 度 + 驅動輪反轉。理由詳見 swerve_config.h。
        """
        self.flip_hysteresis_deg = max(0.0, deg)

    def limit_clearance_deg(self, local_deg):
        """某個機械角度離最近那顆限位開關還有幾度（用可用行程算）"""
        return min(local_deg - self.steer_min_deg, self.steer_max_deg - local_deg)

    def local_to_global_deg(self, i, local_deg):
        return local_deg + self.mount_offset_deg[i]

    def global_to_local_deg(self, i, global_deg):
        return global_deg - self.mount_offset_deg[i]

    # ------------------------------------------------------------ 角度求解
    def resolve_angle(self, i, desired_local_deg, current_local_deg):
        """在行程限制下挑一個能指向 desired_local_deg 的機械角度。

        候選有兩組：desired（輪速為正）與 desired+180（輪速取負），
        每組再考慮 ±360 的等價角。回傳 (angle_deg, sign)，找不到解回傳 None。

        兩輪掃描：pass 0 只考慮離微動開關至少 keepout 遠的候選，找不到才放寬。
        行程 264 度 >= 180 + 2*keepout 時 pass 0 必定有解，所以正常運轉輪子
        永遠不會靠近開關。用硬性篩選而不是軟性罰分，是因為換解固定要轉 180 度，
        任何小於 180 的懲罰在連續轉向時都推不動它。
        """
        for strict in (self.keepout_deg > 0.0, False):
            best = None
            for flip in (0, 1):
                base = desired_local_deg + (180.0 if flip else 0.0)
                sign = -1.0 if flip else 1.0
                for k in range(-2, 3):
                    a = base + 360.0 * k
                    if a < self.steer_min_deg or a > self.steer_max_deg:
                        continue
                    if strict and self.limit_clearance_deg(a) < self.keepout_deg:
                        continue
                    cost = abs(a - current_local_deg)
                    # 換解遲滯：加在「另一組」的成本上而不是減在原本那組，
                    # 這樣兩組都找不到解時的行為完全不變
                    if self.flip_hysteresis_deg > 0.0 and flip != self.last_flip[i]:
                        cost += self.flip_hysteresis_deg
                    if best is None or cost < best[0]:
                        best = (cost, a, sign, flip)
            if best is not None:
                self.last_flip[i] = best[3]
                return best[1], best[2]
            if not strict:
                break
        return None

    def seed_angles(self, current_angle_deg):
        self.last_angle_deg = [constrain(a, self.steer_min_deg, self.steer_max_deg)
                               for a in current_angle_deg]
        self.seeded = True

    # ------------------------------------------------------------ 逆運動學
    def inverse(self, vx, vy, wz, current_angle_deg):
        """底盤速度 -> 四個模組的 (機械角度, 輪速)。current 與回傳都是機械角度。"""
        if not self.seeded:
            self.seed_angles(current_angle_deg)

        mag, direction = [], []
        for i in range(N):
            # 模組 i 的速度向量 = 平移分量 + 旋轉分量 (wz x r)，在車體座標下算
            vxi = vx - wz * self.module_y[i]
            vyi = vy + wz * self.module_x[i]
            mag.append(math.hypot(vxi, vyi))
            direction.append(math.atan2(vyi, vxi))

        # 去飽和：任一輪超過上限就整體等比例縮小，維持運動方向不變形
        peak = max(mag)
        if peak > self.max_wheel_speed:
            scale = self.max_wheel_speed / peak
            mag = [m * scale for m in mag]

        out = []
        for i in range(N):
            # 速度為零時 atan2 沒有意義，維持上一次的角度，不要讓輪子亂轉
            if mag[i] < 1e-4:
                out.append((self.last_angle_deg[i], 0.0))
                continue

            # 車體座標 -> 機械座標，之後的行程比較全部在機械座標下做
            desired_local = self.global_to_local_deg(i, math.degrees(direction[i]))
            r = self.resolve_angle(i, desired_local, current_angle_deg[i])
            if r is None:
                out.append((self.last_angle_deg[i], 0.0))
                continue

            angle_deg, sign = r
            out.append((angle_deg, mag[i] * sign))
            self.last_angle_deg[i] = angle_deg
        return out

    # -------------------------------------------------- 速度投影 / 模組速度
    def module_speeds(self, vx, vy, wz):
        """一個底盤速度會讓四個模組各自以多快移動（純量，恆為正）。

        只看大小不看方向，用來回答「這個指令算不算還在動」。比直接看
        |vx|、|vy| 好的地方是自轉（只有 wz）也算得進來。
        """
        out = []
        for i in range(N):
            vxi = vx - wz * self.module_y[i]
            vyi = vy + wz * self.module_x[i]
            out.append(math.hypot(vxi, vyi))
        return out

    def project_wheel_speeds(self, vx, vy, wz, angle_local_deg):
        """把底盤速度投影到「四個模組已經被指定的方向」上，得到各輪線速度。

        用途是把「角度怎麼決定」和「速度多大」徹底分開 —— 角度解取決於
        vx:vy:wz 的比例，而斜率限制是各軸獨立的，過渡期間那個比例和上層指令
        不一樣。把斜率限制後的速度餵回 inverse() 會算出不同的角度，輪子就會在
        兩個解之間跳（實車症狀：轉向中再動搖桿會抽搐、斜走走不出來）。

        所以角度一律用指令算，速度用這個函式投影。穩態時兩者平行，投影值就等於
        速度大小。反向解（θ+180）不用特別處理，投影自然是負值。
        """
        out = []
        peak = 0.0
        for i in range(N):
            vxi = vx - wz * self.module_y[i]
            vyi = vy + wz * self.module_x[i]
            a = math.radians(self.local_to_global_deg(i, angle_local_deg[i]))
            s = vxi * math.cos(a) + vyi * math.sin(a)
            out.append(s)
            peak = max(peak, abs(s))
        # 去飽和，和 inverse() 同一套
        if peak > self.max_wheel_speed:
            scale = self.max_wheel_speed / peak
            out = [s * scale for s in out]
        return out

    # ------------------------------------------------------------ 正運動學
    def forward(self, angle_deg, speed_mps):
        """四個模組的機械角度 + 輪速 -> 底盤速度（最小平方解）。"""
        sum_vx = sum_vy = sum_moment = 0.0
        for i in range(N):
            # 先把機械角度加上安裝偏移換成車體角度再投影
            a = math.radians(self.local_to_global_deg(i, angle_deg[i]))
            vxi = speed_mps[i] * math.cos(a)
            vyi = speed_mps[i] * math.sin(a)
            sum_vx += vxi
            sum_vy += vyi
            sum_moment += self.module_x[i] * vyi - self.module_y[i] * vxi
        return sum_vx / N, sum_vy / N, sum_moment / self.radius_sum_sq

    @staticmethod
    def normalize_angle(rad):
        while rad > math.pi:
            rad -= 2.0 * math.pi
        while rad <= -math.pi:
            rad += 2.0 * math.pi
        return rad

    def update_odom(self, angle_deg, speed_mps, dt_s):
        if dt_s <= 0.0 or dt_s > 0.5:
            return
        vx, vy, wz = self.forward(angle_deg, speed_mps)
        self.odom['vx'], self.odom['vy'], self.odom['wz'] = vx, vy, wz
        # 用區間中點的航向積分，比用起始航向準
        yaw_mid = self.odom['yaw'] + wz * dt_s * 0.5
        self.odom['x'] += (vx * math.cos(yaw_mid) - vy * math.sin(yaw_mid)) * dt_s
        self.odom['y'] += (vx * math.sin(yaw_mid) + vy * math.cos(yaw_mid)) * dt_s
        self.odom['yaw'] = self.normalize_angle(self.odom['yaw'] + wz * dt_s)

    def reset_odom(self):
        self.odom = dict(x=0.0, y=0.0, yaw=0.0, vx=0.0, vy=0.0, wz=0.0)


# ============================================================================
#  SteerBoardModule：轉向板單一模組的位置環 + 馬達物理
# ============================================================================
class LimitLink:
    """微動開關從主控板送到轉向板的整條路徑。

    對應 Esp32_wheel/lib/LimitBank（去彈跳 + 黏著旗標 + age）加上 CAN 傳輸，
    以及 esp32_rad 那邊 200 Hz 才消化一次的取樣量化。

    一個實體管一個模組的兩顆開關。實車上四個模組是同一個訊框送的，拆開來模擬
    會讓「別的模組觸發時順便把我這顆的狀態早一點送到」這件事消失，所以模擬出來
    的延遲會**略微偏悲觀**——對安全測試而言方向是對的。

    韌體裡 age 是「邊沿到現在幾 ms」，轉向板收到後回推成本地時刻；這裡兩邊共用
    同一個時鐘，所以直接把邊沿時刻放進訊框，效果一樣而且少一層換算誤差。
    """

    def __init__(self):
        self.t_ms = 0.0

        # ---- 主控板側（LimitBank）----
        self._raw = [False, False]        # 未去彈跳
        self._deb = [False, False]        # 去彈跳後
        self._change_ms = [0.0, 0.0]
        self._edge_ms = [0.0, 0.0]        # 去彈跳成立、且是「被壓下」的時刻
        self._latch = [False, False]
        self._latch_sends = [0, 0]
        self._dirty = True
        self._next_poll_ms = 0.0
        self._last_send_ms = -1e9

        # ---- 在途訊框：(送達時刻, state, latched, edge_ms) ----
        self._inflight = []

        # ---- 轉向板側（SteerModule 收到的東西）----
        self.now = [False, False]         # lim_min_ / lim_max_
        self.hit = [False, False]         # hit_min_ / hit_max_
        self.hit_at_ms = [0.0, 0.0]       # hit_min_at_ms_ / hit_max_at_ms_

    def clear_hits(self):
        """對應 SteerModule::enter_phase()：換階段就丟掉還沒消化的邊沿"""
        self.hit = [False, False]

    def step(self, dt_s, phys_min, phys_max):
        """推進一拍。phys_* 是撞塊「真的」有沒有壓到開關（未去彈跳的原始訊號）。"""
        self.t_ms += dt_s * 1000.0
        self._poll(phys_min, phys_max)
        self._deliver()

    # -------------------------------------------------- 主控板：掃描 + 送出
    def _poll(self, phys_min, phys_max):
        # 200 Hz 掃描，這個量化本身就是延遲的一部分
        if self.t_ms < self._next_poll_ms:
            return
        self._next_poll_ms = self.t_ms + 1000.0 / LIMIT_POLL_HZ

        for i, raw in enumerate((phys_min, phys_max)):
            if raw != self._raw[i]:
                self._raw[i] = raw
                self._change_ms[i] = self.t_ms
            elif raw != self._deb[i] and self.t_ms - self._change_ms[i] >= LIMIT_DEBOUNCE_MS:
                self._deb[i] = raw
                if raw:
                    # 記原始訊號變化的時刻，不是去彈跳成立的時刻——撞塊是在
                    # 前者碰到開關的，5 ms 去彈跳是我們自己加的延遲，這段時間
                    # 模組還在轉，記後者就補不回來
                    self._edge_ms[i] = self._change_ms[i]
                    self._latch[i] = True
                    self._latch_sends[i] = 0
                self._dirty = True

        if self._dirty or (self.t_ms - self._last_send_ms) >= LIMIT_HEARTBEAT_MS:
            self._send()

    def _send(self):
        """對應 bot.cpp 的 loop_bot_limits()：打包 + 清 latch"""
        edge = [self._edge_ms[i] if self._latch[i] else None for i in range(2)]
        self._inflight.append((self.t_ms + LIMIT_CAN_LATENCY_MS,
                               list(self._deb), list(self._latch), edge))
        self._last_send_ms = self.t_ms

        for i in range(2):
            if not self._latch[i]:
                continue
            self._latch_sends[i] += 1
            if self._latch_sends[i] >= LIMIT_LATCH_REPEATS:
                self._latch[i] = False
                self._latch_sends[i] = 0

        self._dirty = any(self._latch)

    # -------------------------------------------------- 轉向板：消化
    def _deliver(self):
        arrived = [f for f in self._inflight if f[0] <= self.t_ms]
        if not arrived:
            return
        self._inflight = [f for f in self._inflight if f[0] > self.t_ms]

        for _, state, latched, edge in arrived:
            self.now = list(state)
            for i in range(2):
                # 對應 set_limit_state()：只在「還沒消化」時記時刻，
                # 否則重送的那幾幀會把時間戳一路往前推，補償量就變小了
                if latched[i] and not self.hit[i]:
                    self.hit[i] = True
                    self.hit_at_ms[i] = edge[i] if edge[i] is not None else self.t_ms


# ============================================================================
class SteerBoardModule:
    """對應 esp32_rad 的 SteerModule：位置環 + 歸零狀態機 + 馬達/機構物理。

    這個類別刻意分成「韌體看得到的」與「物理真實的」兩套狀態，不要混用：

        韌體看得到                    物理真實
        ----------------------------  ----------------------------
        angle_deg   由編碼器推算的角度  true_deg    模組真正的角度
        ticks_per_deg  刻度（可能錯）   HW_TICKS_PER_DEG  真實刻度
        lim_min/max 去彈跳後的開關狀態  撞塊有沒有真的壓到開關
        zero_tick   歸零時記下的計數

    歸零的目的就是把 angle_deg 對齊 true_deg。兩者的差就是歸零誤差，
    它會原封不動變成里程計誤差和「直線走不直」，所以這是最值得模擬的東西。

    機構模型包含三個實車一定會有、而且會吃掉精度的效應：
      1. 齒隙  —— 編碼器在馬達軸、90:1 減速，換方向時輸出會空走一段
      2. 刻度誤差 —— 韌體一開始用估算的 ticks/度，和實際有落差
      3. 開關偵測延遲 —— 5 ms 去彈跳，逼近越快、多跑越多才被讀到
    """

    def __init__(self, init_true_deg=STEER_HOMING_PARK_DEG, homed=True):
        # ---- 物理真實 ----
        self.true_deg = init_true_deg    # 模組真正的角度（輸出側）
        self.motor_deg = init_true_deg   # 馬達側位置，和輸出差一個齒隙
        self.rate_dps = 0.0

        # ---- 韌體狀態 ----
        # homed=True 代表「假裝已經完美校正過」，給不想跑歸零的測試用
        self.ticks_per_deg = HW_TICKS_PER_DEG if homed else STEER_TICKS_PER_DEG_NOMINAL
        self.zero_tick = self._enc_ticks() - init_true_deg * self.ticks_per_deg
        self.angle_deg = init_true_deg
        self.target_deg = init_true_deg
        self.homed = homed
        self.fault = False
        self.mode = SM_RUN if homed else SM_IDLE
        self.phase = HOMING_DONE if homed else HOMING_IDLE
        self.measured_span_deg = 0.0

        self.integral = 0.0
        self.last_error = 0.0
        self.derror_filt = 0.0
        self.at_target = False
        self.pwm = 0.0
        self.soft_limited = False

        # 微動開關：實體在主控板，經 LimitLink（去彈跳 + CAN + 取樣量化）才到這裡
        self.link = LimitLink()
        self.lim_min = False   # 對應韌體的 lim_min_（現在的狀態）
        self.lim_max = False
        self.rate_dps_est = 0.0  # 對應韌體的 rate_dps_，延遲補償用
        self._rate_last_tick = None

        self._t_ms = 0.0
        self._phase_start_ms = 0.0
        self._rebound_ref_tick = 0

        # 安全統計。還沒歸零完成之前這個值沒有意義（歸零本來就要去頂開關），
        # 所以先給一個大數，等進入 SM_RUN 才開始真的統計。
        self.min_clearance_deg = (min(init_true_deg - STEER_ANGLE_MIN_DEG,
                                      STEER_ANGLE_MAX_DEG - init_true_deg)
                                  if homed else 999.0)
        self.switch_hits = 0         # 正常運轉時壓到開關的次數，必須永遠是 0
        self.homing_switch_hits = 0  # 歸零時碰到開關的次數，本來就該 > 0
        self.hard_stop_hits = 0      # 撞到硬擋塊的次數（比壓到開關嚴重得多）
        self._was_pressed = False
        self._was_jammed = False

    # ------------------------------------------------------------ 物理
    def _enc_ticks(self):
        """編碼器讀數。注意是馬達側位置乘上『真實』刻度，韌體不知道這個值。

        PCNT 是硬體計數器，只吐整數（韌體那邊 enc_.getTicks() 回傳 int32），
        所以這裡要量化。少了它模擬出來的歸零精度會比實車樂觀約 1 tick
        （1/48 度 ≈ 0.02 度），test_homing 報到小數第三位就變成假精度。
        """
        return float(math.floor(self.motor_deg * HW_TICKS_PER_DEG))

    @property
    def limit_clearance_deg(self):
        """真實角度離最近那顆微動開關還有幾度"""
        return min(self.true_deg - STEER_ANGLE_MIN_DEG,
                   STEER_ANGLE_MAX_DEG - self.true_deg)

    @property
    def jammed(self):
        """撞塊有沒有頂到硬擋塊。歸零時進到開關之後的 overtravel 區是正常的，
        頂到硬擋塊才是真的出事。"""
        return (self.true_deg <= STEER_ANGLE_MIN_DEG - HW_HARD_STOP_OVERTRAVEL_DEG + 1e-6
                or self.true_deg >= STEER_ANGLE_MAX_DEG + HW_HARD_STOP_OVERTRAVEL_DEG - 1e-6)

    @property
    def home_error_deg(self):
        """韌體以為的角度 - 真實角度。歸零做得好不好就看這個。"""
        return self.angle_deg - self.true_deg

    def _update_angle(self):
        """對應 SteerModule::update_angle()：從編碼器推算角度"""
        self.angle_deg = (STEER_ANGLE_MIN_DEG
                          + (self._enc_ticks() - self.zero_tick) / self.ticks_per_deg)

    def _read_limits(self, dt_s):
        """開關訊號從主控板送過來，對應 SteerModule::set_limit_state()。

        真實開關在 true_deg 碰到 0 / 270 時導通，但這塊板看不到那條線，要等
        主控去彈跳（5 ms）、送 CAN、自己再取樣一次才知道。整段延遲換算成角度
        就是零點誤差，逼近越快誤差越大——這正是要做慢速二次逼近的原因，
        也是訊框要帶 age 讓這裡回推的原因。
        """
        self._t_ms += dt_s * 1000.0
        self.link.step(dt_s,
                       self.true_deg <= STEER_ANGLE_MIN_DEG,
                       self.true_deg >= STEER_ANGLE_MAX_DEG)
        self.lim_min, self.lim_max = self.link.now

    def _update_rate(self, dt_s):
        """對應 SteerModule::update_rate()：用編碼器原始計數估角速度。

        不能用 angle_deg 差分——歸零時 zero_tick 會跳，差分會看到假的巨大角速度。
        """
        raw = self._enc_ticks()
        if self._rate_last_tick is not None and dt_s > 1e-4:
            r = (raw - self._rate_last_tick) / self.ticks_per_deg / dt_s
            self.rate_dps_est = 0.7 * self.rate_dps_est + 0.3 * r
        self._rate_last_tick = raw

    def _contact_tick(self, hit_at_ms):
        """對應 SteerModule::contact_tick()：把訊號延遲換算成角度扣掉。

        開關訊號是 age 毫秒前發生的，這段時間模組仍以 rate_dps_est 在轉，
        所以現在的讀數已經多跑了 rate*age 度。扣掉才是真正的接觸點。
        """
        raw = self._enc_ticks()
        if not STEER_LIMIT_AGE_COMPENSATE:
            return raw
        age_ms = self._t_ms - hit_at_ms
        if age_ms < 0.0 or age_ms > 500.0:
            return raw
        back_deg = self.rate_dps_est * (age_ms * 1e-3)
        # 對應韌體的 (int32_t)lroundf(back_deg * ticks_per_deg_)：
        # 補償量也是整數 tick，四捨五入且遠離零（lroundf 的語意）
        v = back_deg * self.ticks_per_deg
        return raw - (math.floor(v + 0.5) if v >= 0.0 else math.ceil(v - 0.5))

    def _drive(self, pwm):
        """對應 SteerModule::drive()：壓到開關就不准再往那個方向出力"""
        if self.lim_min and pwm < 0:
            pwm = 0.0
        if self.lim_max and pwm > 0:
            pwm = 0.0
        self.pwm = constrain(pwm, -float(STEER_PWM_MAX), float(STEER_PWM_MAX))

    def _physics(self, dt_s):
        """PWM -> 馬達轉動 -> 經過齒隙帶動輸出 -> 撞到硬擋塊就停"""
        target_rate = (self.pwm / STEER_PWM_MAX) * STEER_MAX_RATE_DPS
        alpha = dt_s / (STEER_TAU_S + dt_s)
        self.rate_dps += (target_rate - self.rate_dps) * alpha
        self.motor_deg += self.rate_dps * dt_s

        # 齒隙：輸出被馬達側的齒面推著走，中間有一段空隙推不動
        half = HW_BACKLASH_DEG * 0.5
        if self.motor_deg - self.true_deg > half:
            self.true_deg = self.motor_deg - half
        elif self.true_deg - self.motor_deg > half:
            self.true_deg = self.motor_deg + half

        # 硬擋塊：開關之外還有 overtravel，超過就真的頂死了
        lo = STEER_ANGLE_MIN_DEG - HW_HARD_STOP_OVERTRAVEL_DEG
        hi = STEER_ANGLE_MAX_DEG + HW_HARD_STOP_OVERTRAVEL_DEG
        jammed = False
        if self.true_deg < lo:
            self.true_deg, jammed = lo, True
        elif self.true_deg > hi:
            self.true_deg, jammed = hi, True
        if jammed:
            self.rate_dps = 0.0
            self.motor_deg = constrain(self.motor_deg,
                                       self.true_deg - half, self.true_deg + half)
        if jammed and not self._was_jammed:
            self.hard_stop_hits += 1
        self._was_jammed = jammed

        # 統計：只算「正常運轉」期間。歸零本來就一定要去碰開關，
        # 把那幾次算進來會讓安全指標永遠是紅的，反而看不出真正的問題。
        pressed = self.lim_min or self.lim_max
        newly_pressed = pressed and not self._was_pressed
        self._was_pressed = pressed

        if self.mode == SM_RUN and self.homed and not self.fault:
            c = self.limit_clearance_deg
            if c < self.min_clearance_deg:
                self.min_clearance_deg = c
            if newly_pressed:
                self.switch_hits += 1
        elif self.mode == SM_HOMING and newly_pressed:
            self.homing_switch_hits += 1

    # ------------------------------------------------------------ 對外介面
    def set_target_deg(self, deg):
        self.target_deg = constrain(deg, STEER_ANGLE_MIN_DEG, STEER_ANGLE_MAX_DEG)

    def set_mode(self, m):
        if m == self.mode:
            return
        self.mode = m
        self.integral = 0.0
        self.derror_filt = 0.0
        self.last_error = 0.0
        if m == SM_HOMING:
            self.start_homing()

    def start_homing(self):
        self.homed = False
        self.fault = False
        self.mode = SM_HOMING
        self.phase = HOMING_BACKOFF
        self._phase_start_ms = self._t_ms
        self.link.clear_hits()
        self.integral = 0.0
        self.ticks_per_deg = STEER_TICKS_PER_DEG_NOMINAL  # 先用估算值

    def update(self, dt_s):
        self._read_limits(dt_s)
        self._update_angle()
        self._update_rate(dt_s)

        if self.mode == SM_HOMING:
            self._run_homing(dt_s)
        elif self.mode == SM_RUN and self.homed and not self.fault:
            self._run_position_pid(dt_s)
        else:
            self.at_target = False
            self._drive(0.0)

        self._physics(dt_s)

    # ------------------------------------------------------------ 歸零狀態機
    def _run_homing(self, dt_s):
        """1:1 對應 SteerModule::run_homing()

        兩顆開關各做「快速撞 -> 退開 -> 慢速再碰」，慢速那次才記錄。
        碰兩顆的目的：min 給零點，max 給刻度（用實測行程反推 ticks/度）。
        """
        timed_out = (self._t_ms - self._phase_start_ms) > STEER_HOMING_TIMEOUT_MS
        hit_min, hit_max = self.link.hit

        def go(next_phase):
            """對應 SteerModule::enter_phase()"""
            self.phase = next_phase
            self._phase_start_ms = self._t_ms
            # 上一階段沒消化的開關邊沿不要留到下一階段
            self.link.clear_hits()

        def fail():
            self._drive(0.0)
            go(HOMING_FAULT)
            self.fault = True

        p = self.phase
        if p == HOMING_BACKOFF:
            # 開機時可能剛好壓在某個開關上，先離開。
            # 這裡問的是「現在還壓著嗎」，所以用現在狀態而不是黏著旗標
            if self.lim_min:
                self._drive(+STEER_HOMING_PWM)
            elif self.lim_max:
                self._drive(-STEER_HOMING_PWM)
            else:
                self._drive(0.0)
                go(HOMING_SEEK_MIN)
                return
            if timed_out:
                fail()

        elif p == HOMING_SEEK_MIN:
            # 快速逼近時開關可能只導通一瞬間就彈開，所以連黏著旗標一起看
            if self.lim_min or hit_min:
                # 快速這次只把位置找到，不記錄，所以也不需要補償
                self._drive(0.0)
                self._rebound_ref_tick = self._enc_ticks()
                go(HOMING_REBOUND_MIN if STEER_HOMING_SLOW_PWM > 0 else HOMING_SLOW_MIN)
            elif self.lim_max or hit_max:
                fail()  # 往 0 度找卻壓到 270 度開關 = 方向反了
            else:
                self._drive(-STEER_HOMING_PWM)
                if timed_out:
                    fail()

        elif p == HOMING_REBOUND_MIN:
            moved = (self._enc_ticks() - self._rebound_ref_tick) / self.ticks_per_deg
            if not self.lim_min and moved >= STEER_HOMING_REBOUND_DEG:
                self._drive(0.0)
                go(HOMING_SLOW_MIN)
            else:
                self._drive(+STEER_HOMING_PWM)
                if timed_out:
                    fail()

        elif p == HOMING_SLOW_MIN:
            # 這一次才是真正記零點的，所以要把訊號延遲扣掉
            if hit_min or self.lim_min:
                self._drive(0.0)
                # 有黏著旗標才知道邊沿何時發生；只有現在狀態就退回不補償
                self.zero_tick = (self._contact_tick(self.link.hit_at_ms[0])
                                  if hit_min else self._enc_ticks())
                self._update_angle()
                if STEER_MEASURE_SPAN:
                    go(HOMING_SEEK_MAX)
                else:
                    self.target_deg = STEER_HOMING_PARK_DEG
                    go(HOMING_RETURN)
            else:
                self._drive(-(STEER_HOMING_SLOW_PWM or STEER_HOMING_PWM))
                if timed_out:
                    fail()

        elif p == HOMING_SEEK_MAX:
            if self.lim_max or hit_max:
                self._drive(0.0)
                self._rebound_ref_tick = self._enc_ticks()
                go(HOMING_REBOUND_MAX if STEER_HOMING_SLOW_PWM > 0 else HOMING_SLOW_MAX)
            elif self.lim_min and (self._t_ms - self._phase_start_ms) > 1500:
                fail()  # 離不開 0 度開關 = 馬達沒動或方向反了
            else:
                self._drive(+STEER_HOMING_PWM)
                if timed_out:
                    fail()

        elif p == HOMING_REBOUND_MAX:
            moved = (self._rebound_ref_tick - self._enc_ticks()) / self.ticks_per_deg
            if not self.lim_max and moved >= STEER_HOMING_REBOUND_DEG:
                self._drive(0.0)
                go(HOMING_SLOW_MAX)
            else:
                self._drive(-STEER_HOMING_PWM)
                if timed_out:
                    fail()

        elif p == HOMING_SLOW_MAX:
            # 這一次決定刻度，同樣要扣掉延遲，否則量到的行程會偏長
            if hit_max or self.lim_max:
                self._drive(0.0)
                span_deg = STEER_ANGLE_MAX_DEG - STEER_ANGLE_MIN_DEG
                contact = (self._contact_tick(self.link.hit_at_ms[1])
                           if hit_max else self._enc_ticks())
                span_ticks = contact - self.zero_tick
                measured = span_ticks / span_deg if span_deg > 1.0 else 0.0
                sane = (span_ticks > 50 and span_deg > 1.0 and
                        (STEER_SPAN_TOL_FRAC <= 0.0 or
                         self.ticks_per_deg * (1 - STEER_SPAN_TOL_FRAC) < measured
                         < self.ticks_per_deg * (1 + STEER_SPAN_TOL_FRAC)))
                if sane:
                    self.ticks_per_deg = measured
                    self.measured_span_deg = span_deg
                    self._update_angle()
                    self.target_deg = STEER_HOMING_PARK_DEG
                    go(HOMING_RETURN)
                else:
                    fail()
            else:
                self._drive(+(STEER_HOMING_SLOW_PWM or STEER_HOMING_PWM))
                if timed_out:
                    fail()

        elif p == HOMING_RETURN:
            self.homed = True
            self.target_deg = STEER_HOMING_PARK_DEG
            self._run_position_pid(dt_s)
            if self.at_target:
                go(HOMING_DONE)
            elif timed_out:
                self.homed = False
                fail()

        elif p == HOMING_DONE:
            self._run_position_pid(dt_s)

        else:  # HOMING_FAULT / HOMING_IDLE
            self.at_target = False
            self._drive(0.0)

    # ------------------------------------------------------------ 位置環
    def _run_position_pid(self, dt_s):
        """1:1 對應 SteerModule::run_position_pid()。注意用的是 angle_deg
        （韌體以為的角度），不是 true_deg——韌體本來就只看得到編碼器。"""
        error = self.target_deg - self.angle_deg
        self.at_target = abs(error) <= STEER_TOLERANCE_DEG

        if self.at_target:
            self.integral *= 0.9
            self.last_error = error
            self._drive(0.0)
            self.soft_limited = False
            return

        self.integral += error * dt_s
        if STEER_PID_KI > 1e-6:
            imax = STEER_MAX_PWM / STEER_PID_KI
            self.integral = constrain(self.integral, -imax, imax)
        else:
            self.integral = 0.0

        derror = (error - self.last_error) / dt_s
        self.derror_filt = 0.7 * self.derror_filt + 0.3 * derror
        self.last_error = error

        out = (STEER_PID_KP * error + STEER_PID_KI * self.integral
               + STEER_PID_KD * self.derror_filt)
        if out > 0.5:
            out += STEER_MIN_MOVE_PWM
        elif out < -0.5:
            out -= STEER_MIN_MOVE_PWM
        out = constrain(out, -float(STEER_MAX_PWM), float(STEER_MAX_PWM))

        # 軟限位：越接近行程末端，越壓低往那個末端去的出力
        self.soft_limited = False
        if STEER_SOFT_LIMIT_DEG > 0.0:
            if out < 0.0:
                room = self.angle_deg - STEER_ANGLE_MIN_DEG
                if room < STEER_SOFT_LIMIT_DEG:
                    out *= (room / STEER_SOFT_LIMIT_DEG) if room > 0.0 else 0.0
                    self.soft_limited = True
            elif out > 0.0:
                room = STEER_ANGLE_MAX_DEG - self.angle_deg
                if room < STEER_SOFT_LIMIT_DEG:
                    out *= (room / STEER_SOFT_LIMIT_DEG) if room > 0.0 else 0.0
                    self.soft_limited = True

        # 對應韌體的 drive((int)out)：ledcWrite 吃的是整數，往零截斷。
        # 軟限位把出力壓到 0~1 之間時，實車是完全不動的，這裡不取整會多爬一點。
        self._drive(float(int(out)))


# ============================================================================
#  SwerveRobot：把兩塊板 + 車體物理兜起來
# ============================================================================
class SwerveRobot:
    """對應 bot.cpp 的 loop_bot_control()，外加車體剛體積分。

    真實姿態 (true_*) 用「物理偏移」算，里程計用「設定偏移」算。
    兩者一致時應該完全重疊；把設定偏移調錯就會看到里程計飄掉。
    """

    def __init__(self, control_hz=LOOP_CONTROL_HZ, report_hz=LOOP_REPORT_HZ,
                 steer_hz=LOOP_STEER_HZ, odom_hz=LOOP_ODOM_HZ):
        # 把四個迴圈頻率開出來，測試時可以拉到跟物理積分同步，
        # 藉此把「座標轉換對不對」和「取樣延遲造成的誤差」分開看
        self.control_hz = control_hz
        self.report_hz = report_hz
        self.steer_hz = steer_hz
        self.odom_hz = odom_hz

        self.kin = SwerveKinematics()
        self.kin.set_chassis_param(CHASSIS_WHEELBASE_M, CHASSIS_TRACK_M, MAX_WHEEL_SPEED_MPS)
        self.kin.set_steer_range(STEER_ANGLE_MIN_DEG, STEER_ANGLE_MAX_DEG, STEER_ANGLE_MARGIN_DEG)
        self.kin.set_mount_offsets(MOUNT_OFFSET_DEG)
        self.kin.set_limit_keepout(STEER_KEEPOUT_DEG)
        self.kin.set_flip_hysteresis(STEER_FLIP_HYSTERESIS_DEG)

        # 物理真實偏移，預設等於設定值；改動它 = 模擬校正沒做準
        self.physical_offset_deg = list(MOUNT_OFFSET_DEG)

        # 開機歸零完成後停在 park 角度（行程正中間，離兩顆開關都最遠）
        self.steer = [SteerBoardModule(STEER_HOMING_PARK_DEG) for _ in range(N)]

        # 主控看到的「轉向板回報角度」，50 Hz 才更新一次
        self.reported_deg = [m.angle_deg for m in self.steer]

        self.cmd = [0.0, 0.0, 0.0]          # vx, vy, wz（上層要的）
        self.ramp = [0.0, 0.0, 0.0]         # 斜率限制後的，實際拿去算輪速的
        self.align_gate_closed = [True] * N # 到位閘門的遲滯狀態，True = 這顆還沒到位
        self.gate_stop = False              # 這一拍有沒有把底盤速度壓成零
        self.repoint_wait = False           # 要大變向、但車還在跑，正在等它停
        self.target_deg = [m.angle_deg for m in self.steer]
        self.wheel_cmd_mps = [0.0] * N      # 主控算出來、經餘弦補償後送 VESC 的值
        self.wheel_act_mps = [0.0] * N      # VESC 實際輪速
        self.wheel_raw_mps = [0.0] * N      # 補償前的逆運動學輪速（看 flip 用）

        self.true_pose = [0.0, 0.0, 0.0]    # x, y, yaw
        self.true_vel = [0.0, 0.0, 0.0]
        self.true_trail = []
        self.odom_trail = []

        self._t = 0.0
        self._next_control = 0.0
        self._next_report = 0.0
        self._next_odom = 0.0

        # 轉向板 READY 的邊沿偵測，對應 bot.cpp 的 steer_ready_prev
        self._steer_ready_prev = False

        # ---- 急停（對應 bot.cpp 的 loop_bot_estop）----
        self.estop_switch = False      # 實體開關現在有沒有被按下（測試直接設它）
        self.estop_latched = False     # 閂鎖後的實際急停狀態
        self.estop_braking = False     # 這一拍是不是在灌煞車電流
        self._estop_raw = False
        self._estop_input = False
        self._estop_change_ms = 0.0
        self._estop_cmd_zero_since_ms = None
        self._t_ms = 0.0

    # ------------------------------------------------------------------
    def set_estop(self, pressed):
        """按下 / 放開急停開關。對應實體開關的動作，不是直接設閂鎖狀態。"""
        self.estop_switch = bool(pressed)

    def _estop_step(self, dt_s):
        """1:1 對應 bot.cpp 的 loop_bot_estop()，跑在 200 Hz"""
        self._t_ms += dt_s * 1000.0
        now = self._t_ms

        # 去彈跳：觸發快、解除慢
        raw = self.estop_switch
        if raw != self._estop_raw:
            self._estop_raw = raw
            self._estop_change_ms = now
        else:
            need = ESTOP_TRIP_DEBOUNCE_MS if raw else ESTOP_CLEAR_DEBOUNCE_MS
            if now - self._estop_change_ms >= need:
                self._estop_input = raw

        # 觸發
        if self._estop_input and not self.estop_latched:
            self.estop_latched = True
            self._estop_cmd_zero_since_ms = None
            return

        # 解除
        if self.estop_latched and not self._estop_input and self._estop_rearm_ok(now):
            self.estop_latched = False
            self._estop_cmd_zero_since_ms = None
            self._estop_clear_stale_cmd()

    def _estop_clear_stale_cmd(self):
        """解除的那一瞬間把急停前殘留的 /cmd_vel 作廢。

        對應 bot.cpp 的 estop_clear_stale_cmd()。ESTOP_REARM_REQUIRE_ZERO=False
        時這是唯一的暴衝防護：不清掉的話，急停前那筆「全速前進」在解除的
        下一拍就會被拿去用，車子在手還在開關上的時候就衝出去了。

        韌體那邊是把 cmd_last_ms 清成 0（讓 cmd_stale 成立），模擬沒有移植
        /cmd_vel 逾時，所以直接把速度歸零——效果一樣：要重新下指令才會動。
        """
        self.cmd = [0.0, 0.0, 0.0]

    def _estop_rearm_ok(self, now):
        """開關已放開，還要 /cmd_vel 維持零速夠久才准恢復。

        沒有這一條的話，上層導航還在催全速時放開開關，車子會立刻再暴衝一次。

        但實車上這一條讓手動操作幾乎不能用：teleop_twist_keyboard 按著方向鍵
        是持續發非零的，一邊打開急停一邊想開車，每按一次就把計時歸零，結果是
        「怎麼按都不動」。所以預設改成放開即解除，暴衝防護交給
        _estop_clear_stale_cmd()。詳見 swerve_config.h 的 ESTOP_REARM_REQUIRE_ZERO。
        """
        if not ESTOP_LATCH:
            return True
        if not ESTOP_REARM_REQUIRE_ZERO:
            return True

        zero = all(abs(v) < 1e-3 for v in self.cmd)
        if not zero:
            self._estop_cmd_zero_since_ms = None  # 又開始催了，重新計時
            return False
        if self._estop_cmd_zero_since_ms is None:
            self._estop_cmd_zero_since_ms = now
            return False
        return (now - self._estop_cmd_zero_since_ms) >= ESTOP_REARM_CMD_ZERO_MS

    # ------------------------------------------------------------------
    def set_cmd_vel(self, vx, vy, wz):
        self.cmd = [constrain(vx, -MAX_LINEAR_SPEED_MPS, MAX_LINEAR_SPEED_MPS),
                    constrain(vy, -MAX_LINEAR_SPEED_MPS, MAX_LINEAR_SPEED_MPS),
                    constrain(wz, -MAX_ANGULAR_SPEED_RPS, MAX_ANGULAR_SPEED_RPS)]

    def start_homing(self, start_deg=None):
        """真的跑一次開機歸零。

        start_deg 是四個模組「上電當下停在哪」的真實角度；不給就用預設的
        四個不同起點（含一個開機就壓在開關上的，那是最容易出事的情況）。
        """
        if start_deg is None:
            start_deg = [200.0, 60.0, 268.0, 1.0]
        for i, m in enumerate(self.steer):
            m.__init__(start_deg[i], homed=False)
            m.start_homing()
        self.kin.reset_odom()
        self.kin.seeded = False
        self._steer_ready_prev = False
        self.reported_deg = [m.angle_deg for m in self.steer]
        self.true_pose = [0.0, 0.0, 0.0]
        self.wheel_act_mps = [0.0] * N
        self.wheel_cmd_mps = [0.0] * N
        self.true_trail.clear()
        self.odom_trail.clear()

    @property
    def homing_done(self):
        return all(m.homed and m.phase == HOMING_DONE for m in self.steer)

    @property
    def homing_fault(self):
        return any(m.fault for m in self.steer)

    @property
    def ready(self):
        """對應 bot.cpp 的 steer_ready：沒好之前不給驅動輪出力"""
        return self.homing_done and not self.homing_fault

    def reset(self):
        for m in self.steer:
            m.__init__(STEER_HOMING_PARK_DEG)
        self.kin.reset_odom()
        self.kin.seeded = False
        self._steer_ready_prev = False
        self.reported_deg = [m.angle_deg for m in self.steer]
        self.true_pose = [0.0, 0.0, 0.0]
        self.wheel_act_mps = [0.0] * N
        self.wheel_cmd_mps = [0.0] * N
        self.true_trail.clear()
        self.odom_trail.clear()
        self._t = 0.0
        self._next_control = 0.0
        self._next_report = 0.0
        self._next_odom = 0.0

        self.estop_switch = False
        self.estop_latched = False
        self.estop_braking = False
        self._estop_raw = False
        self._estop_input = False
        self._estop_change_ms = 0.0
        self._estop_cmd_zero_since_ms = None
        self._t_ms = 0.0

    # ------------------------------------------------- 主控控制迴圈
    @staticmethod
    def _slew_toward(cur, tgt, accel, decel, dt_s):
        """1:1 對應 bot.cpp 的 slew_toward()。

        「加速」= 不換號而且絕對值變大，其餘（含要反向）都算減速，所以要反向時
        會先被拉到 0 再從 0 長出去，中間不會跳過零點。
        """
        speeding_up = (tgt * cur >= 0.0) and (abs(tgt) > abs(cur))
        step = (accel if speeding_up else decel) * dt_s
        d = tgt - cur
        if abs(d) <= step:
            return tgt
        return cur + (step if d > 0.0 else -step)

    def _control_step(self, dt):
        vx, vy, wz = self.cmd

        # 轉向板還沒校正好就停車，對應 bot.cpp 的 steer_ready 判斷。
        # 轉向角度指令照送（讓輪子守住），只是速度歸零。
        # 急停走同一條路：速度歸零之後逆運動學會沿用上一次的角度，輪子自然凍結。
        hard_stop = (not self.ready) or self.estop_latched
        if hard_stop:
            vx = vy = wz = 0.0

        # ---- 轉向板剛歸零完成：把「上次角度」對齊它現在停的地方 ----
        # 對應 bot.cpp loop_bot_control() 的 1.5 步。沒有這一段的話，
        # inverse() 內部那次 seed 會停在「開機當下」的角度（歸零都還沒開始），
        # 零速時走的又是「沿用上一次角度」的捷徑、不經過 resolve_angle()，
        # 於是歸零完成停在 park 的輪子會被一路拉回開機角度貼著限位開關。
        if self.ready and not self._steer_ready_prev:
            self.kin.seed_angles(self.reported_deg)
        self._steer_ready_prev = self.ready

        # ---- 轉向到位閘門（對應 bot.cpp 的 1.6 步）----
        # 用上一拍送出去的目標角度比，理由和韌體一樣：這一拍的目標要等逆運動學
        # 才算得出來，而逆運動學的輸入又要先知道閘門開不開。差一拍可以忽略。
        gate_any_closed = False
        for i in range(N):
            err = abs(self.target_deg[i] - self.reported_deg[i])
            self.align_gate_closed[i] = (err > DRIVE_ALIGN_RELEASE_DEG
                                         if self.align_gate_closed[i]
                                         else err > DRIVE_ALIGN_GATE_DEG)
            if self.align_gate_closed[i]:
                gate_any_closed = True
        # repoint_wait 是上一拍第 2.5 步留下來的：「要大變向，但車還在跑，先停」。
        # 它和閘門一樣要把底盤速度壓成零，但原因不同 —— 閘門是「輪子還沒轉到位」，
        # 它是「輪子還不可以開始轉」。
        gate_or_repoint = (gate_any_closed if DRIVE_ALIGN_GATE_LINKED else False) \
            or self.repoint_wait
        self.gate_stop = gate_or_repoint

        # ---- 加減速斜率限制（對應 bot.cpp 的 1.7 步）----
        if hard_stop:
            self.ramp = [0.0, 0.0, 0.0]     # 保護動作不走斜率，立刻歸零
        else:
            tvx, tvy, twz = (0.0, 0.0, 0.0) if self.gate_stop else (vx, vy, wz)
            self.ramp[0] = self._slew_toward(self.ramp[0], tvx,
                                             DRIVE_ACCEL_MPS2, DRIVE_DECEL_MPS2, dt)
            self.ramp[1] = self._slew_toward(self.ramp[1], tvy,
                                             DRIVE_ACCEL_MPS2, DRIVE_DECEL_MPS2, dt)
            self.ramp[2] = self._slew_toward(self.ramp[2], twz,
                                             DRIVE_ANG_ACCEL_RPS2, DRIVE_ANG_DECEL_RPS2, dt)

        # ---- 逆運動學：角度**只**由上層指令決定 ----
        # ⚠ 絕對不要把斜率限制後的 self.ramp 餵進 inverse()。角度解取決於
        #   vx:vy:wz 的**比例**，而斜率限制是各軸獨立的，過渡期間那個比例和
        #   指令不一樣 -> 「照 ramp 算的角度」和「照指令算的角度」是兩個不同的
        #   值，閘門一開一關、ramp 在零與非零之間來回，目標角就會在兩者之間跳。
        #   實車症狀（2026-08-16 回報）：穩態邊轉邊走正常，但**轉向還沒到位時**
        #   再動搖桿就抽搐；斜走也走不出來 —— 各軸以相同絕對速率漲落會把合成
        #   方向拉向較大的那一軸。
        #   速度改由 project_wheel_speeds() 投影到這些角度上（見第 4 步）。
        states = self.kin.inverse(vx, vy, wz, self.reported_deg)
        # 防禦性夾限，對應 bot.cpp 裡送 CAN 之前的那一道
        desired_deg = [constrain(s[0],
                                 STEER_ANGLE_MIN_DEG + STEER_ANGLE_MARGIN_DEG,
                                 STEER_ANGLE_MAX_DEG - STEER_ANGLE_MARGIN_DEG)
                       for s in states]

        # ---- 大角度變向：先停車再轉（對應 bot.cpp 的 2.5 步）----
        # 凍結期間 target_deg 維持不動，輪子守在「車體正在走的方向」上，完全不刮。
        # 「還要轉多少」要比**輪子實際指的角度**，不是 target_deg。target_deg
        # 平常每一拍都跟著 desired 更新，兩者的差其實是「這一拍方向變了多少」
        # ——那是角速度不是總轉角，拿去比角度門檻會變成「搖桿推快一點就停車」。
        pending_deg = max(abs(d - r) for d, r in zip(desired_deg, self.reported_deg))
        # 「車停了沒」要看實際輪速不是指令（指令被閘門壓成零的下一拍就是 0，
        # 但車體還在滑）。同時保底看斜率限制器的指令速度：VESC 回授掛掉時
        # 實際輪速會一直是 0，這條保護會靜默失效。用模組速度向量的大小，
        # 這樣自轉（只有 wz）也算得到。
        fastest_mps = max([abs(v) for v in self.wheel_act_mps]
                          + self.kin.module_speeds(*self.ramp))
        if DRIVE_REPOINT_SPEED_MPS <= 0.0 or pending_deg <= DRIVE_REPOINT_ANGLE_DEG:
            self.repoint_wait = False
        else:
            self.repoint_wait = fastest_mps > DRIVE_REPOINT_SPEED_MPS
        if not self.repoint_wait:
            self.target_deg = desired_deg

        # ---- 送轉向角度指令 ----
        # 轉向板只在 READY 狀態才會吃主控送來的角度（見 esp32_rad/src/main.cpp
        # 的 SWERVE_ID_STEER_CMD 處理）；歸零期間目標角由歸零狀態機自己掌控
        if self.ready:
            for i in range(N):
                self.steer[i].set_target_deg(self.target_deg[i])

        # ---- 輪速：把斜率限制後的底盤速度投影到「實際送出去的目標角」上 ----
        # 角度是上面照指令算的、和 ramp 無關，所以 ramp 怎麼變都不會動到角度。
        # 用 target_deg（可能被「先停車再轉」凍結）而不是 desired_deg：凍結期間
        # 投影出來就是「照原方向繼續滑行、同時減速」，正好是不刮地的那個行為。
        self.wheel_raw_mps = self.kin.project_wheel_speeds(
            self.ramp[0], self.ramp[1], self.ramp[2], self.target_deg)

        # ---- 餘弦補償 + 到位閘門（對應 bot.cpp 的第 4 步）----
        # 四顆連動模式下閘門已經在上面把底盤速度壓成零了，這裡不用再做；
        # 只有「各切各的」模式才在這裡逐顆歸零。
        for i in range(N):
            err_rad = math.radians(self.target_deg[i] - self.reported_deg[i])
            k = math.cos(err_rad)
            if k < 0.0:
                k = 0.0
            if (not DRIVE_ALIGN_GATE_LINKED) and self.align_gate_closed[i]:
                k = 0.0
            self.wheel_cmd_mps[i] = self.wheel_raw_mps[i] * k


    # --------------------------------------------- 主控里程計（獨立頻率）
    def _odom_step(self, dt):
        """對應 bot.cpp 的 loop_bot_odom()，跑在 LOOP_ODOM_HZ。

        用的是「回報角度」，也就是最多晚一個回報週期的舊值。轉向正在甩的
        瞬間這個落差會讓里程計短暫失真，實車也一樣，不是 bug。
        """
        self.kin.update_odom(self.reported_deg, self.wheel_act_mps, dt)
        o = self.kin.odom
        if not self.odom_trail or math.hypot(o['x'] - self.odom_trail[-1][0],
                                             o['y'] - self.odom_trail[-1][1]) > TRAIL_STEP_M:
            self.odom_trail.append((o['x'], o['y']))
            if len(self.odom_trail) > TRAIL_MAX_POINTS:
                del self.odom_trail[0]

    # ------------------------------------------------------- 真實物理積分
    def _physics_step(self, dt):
        # VESC 一階跟隨。急停時 VESC 收到的是煞車電流而不是速度指令，
        # 用一個短很多的時間常數代表「主動煞車」和「送 0 速度然後滑行」的差別。
        if self.estop_latched:
            fastest = max(abs(v) for v in self.wheel_act_mps)
            self.estop_braking = fastest > ESTOP_BRAKE_UNTIL_MPS
            tau = DRIVE_BRAKE_TAU_S if self.estop_braking else DRIVE_TAU_S
        else:
            self.estop_braking = False
            tau = DRIVE_TAU_S

        alpha = dt / (tau + dt)
        for i in range(N):
            self.wheel_act_mps[i] += (self.wheel_cmd_mps[i] - self.wheel_act_mps[i]) * alpha

        # 用「物理真實偏移」把四顆輪子的速度向量投影回車體速度
        sum_vx = sum_vy = sum_moment = 0.0
        for i in range(N):
            a = math.radians(self.steer[i].true_deg + self.physical_offset_deg[i])
            vxi = self.wheel_act_mps[i] * math.cos(a)
            vyi = self.wheel_act_mps[i] * math.sin(a)
            sum_vx += vxi
            sum_vy += vyi
            sum_moment += self.kin.module_x[i] * vyi - self.kin.module_y[i] * vxi
        vx = sum_vx / N
        vy = sum_vy / N
        wz = sum_moment / self.kin.radius_sum_sq
        self.true_vel = [vx, vy, wz]

        yaw_mid = self.true_pose[2] + wz * dt * 0.5
        self.true_pose[0] += (vx * math.cos(yaw_mid) - vy * math.sin(yaw_mid)) * dt
        self.true_pose[1] += (vx * math.sin(yaw_mid) + vy * math.cos(yaw_mid)) * dt
        self.true_pose[2] = SwerveKinematics.normalize_angle(self.true_pose[2] + wz * dt)

        if not self.true_trail or math.hypot(self.true_pose[0] - self.true_trail[-1][0],
                                             self.true_pose[1] - self.true_trail[-1][1]) > TRAIL_STEP_M:
            self.true_trail.append((self.true_pose[0], self.true_pose[1]))
            if len(self.true_trail) > TRAIL_MAX_POINTS:
                del self.true_trail[0]

    # ------------------------------------------------------------------
    def step(self, dt_s):
        """推進模擬。內部依韌體的實際頻率切成子步驟。"""
        sub = 1.0 / self.steer_hz
        remaining = min(dt_s, 0.2)  # 視窗卡住時不要一次補太多，會炸開
        while remaining > 1e-9:
            h = min(sub, remaining)
            remaining -= h
            self._t += h

            # 急停：對應 control_task 每一拍最先跑的 loop_bot_estop()
            self._estop_step(h)

            # 轉向板位置環（預設 200 Hz）
            for m in self.steer:
                m.update(h)

            # 四顆都歸零完成 -> 切到 RUN，對應 esp32_rad/src/main.cpp 的
            # 「board_state 轉 READY 時把 target 設成當下角度再切 SM_RUN」
            if self.homing_done and any(m.mode == SM_HOMING for m in self.steer):
                for m in self.steer:
                    m.set_target_deg(m.angle_deg)
                    m.mode = SM_RUN

            # 轉向板 -> 主控 角度回報（預設 50 Hz）
            if self._t >= self._next_report - 1e-12:
                self._next_report = self._t + 1.0 / self.report_hz
                self.reported_deg = [m.angle_deg for m in self.steer]

            # 主控控制迴圈（預設 50 Hz）：逆運動學 + 送指令
            if self._t >= self._next_control - 1e-12:
                self._control_step(1.0 / self.control_hz)
                self._next_control = self._t + 1.0 / self.control_hz

            # 主控里程計（預設 100 Hz），和控制迴圈拆開跑
            if self._t >= self._next_odom - 1e-12:
                self._odom_step(1.0 / self.odom_hz)
                self._next_odom = self._t + 1.0 / self.odom_hz

            self._physics_step(h)

    # ------------------------------------------------------------------
    def module_global_deg(self, i):
        """模組 i 的輪子在車體座標下的真實朝向（度）。"""
        return self.steer[i].true_deg + self.physical_offset_deg[i]

    def odom_error(self):
        o = self.kin.odom
        return math.hypot(o['x'] - self.true_pose[0], o['y'] - self.true_pose[1])

    # ---------------------------------------------- 限位安全相關的統計
    def clearance_now(self):
        """四顆之中，現在離微動開關最近的那顆還有幾度"""
        return min(m.limit_clearance_deg for m in self.steer)

    def worst_clearance(self):
        """從開機到現在，離微動開關最近曾經到過幾度"""
        return min(m.min_clearance_deg for m in self.steer)

    def total_switch_hits(self):
        """正常運轉時微動開關被壓下過幾次。這個數字必須永遠是 0。
        （歸零時碰開關是必要的，不算在內，見 homing_switch_hits）"""
        return sum(m.switch_hits for m in self.steer)

    def total_homing_switch_hits(self):
        """歸零時碰到開關的次數。二次逼近的話每顆開關會碰兩次，
        四個模組 x 兩顆開關 x 兩次 = 16 次才是正常的。"""
        return sum(m.homing_switch_hits for m in self.steer)

    def total_hard_stop_hits(self):
        """頂到硬擋塊的次數。這個比壓到開關嚴重，必須是 0。"""
        return sum(m.hard_stop_hits for m in self.steer)

    def out_of_range(self):
        """有沒有模組跑出實體行程（真的發生就是模擬或邏輯有問題）"""
        return [i for i, m in enumerate(self.steer)
                if m.true_deg < STEER_ANGLE_MIN_DEG - 1e-6
                or m.true_deg > STEER_ANGLE_MAX_DEG + 1e-6]
