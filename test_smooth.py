"""平順度測試：加減速斜率限制、轉向到位閘門、大變向先停車再轉、換解遲滯。

這四樣是為了「移動要乾淨俐落、車體不跳動」加的，全部在 bot.cpp 的
loop_bot_control() 裡（對應 swerve_model.py 的 _control_step）。

第 5 項是其中最反直覺的一個，值得先看：輪胎橫著刮地的原因**不是驅動輪在
出力**，是「車體還在往原方向跑，轉向卻已經開始甩」。所以光靠斷驅動動力
壓不下來（實測只從 0.21 降到 0.15 m/s，用更兇的減速去斷油甚至更糟），
真正有效的是「大變向時先凍結轉向目標，等車停了再轉」（降到 0.006 m/s）。

⚠ 這支存在的主要理由是第 4 項的**死結**。閘門把底盤速度壓成零之後，如果
  直接把零速餵進 inverse()，它會走「沿用上一次角度」的捷徑 —— 上層新下的
  行進方向永遠解不出來，輪子不轉向，閘門就永遠不會開，整台車卡死不動。
  韌體是靠「速度被壓成零但上層還在要求移動時，改用上層的速度去解角度」
  避開的。那一段很容易在後續重構時被當成多餘而拿掉，所以要有測試守著。

    python test_smooth.py
"""

import math
import sys

import swerve_model as M
from swerve_model import SwerveRobot

fails = []


def check(cond, msg):
    print(("  [ OK ] " if cond else "  [FAIL] ") + msg)
    if not cond:
        fails.append(msg)


def run(bot, seconds, dt=1.0 / 200.0):
    for _ in range(int(seconds / dt)):
        bot.step(dt)


def make_ready_bot():
    """做一台歸零完成、靜止待命的車"""
    bot = SwerveRobot()
    bot.set_cmd_vel(0.0, 0.0, 0.0)
    run(bot, 2.0)
    return bot


def sample_ramp(bot, seconds, dt=1.0 / 200.0):
    """跑一段時間，回傳每一拍的 (t, ramp_vx)"""
    out = []
    t = 0.0
    for _ in range(int(seconds / dt)):
        bot.step(dt)
        t += dt
        out.append((t, bot.ramp[0]))
    return out


# 控制迴圈是 LOOP_CONTROL_HZ（50 Hz），斜率限制器每 20 ms 才走一階。
# 用比控制週期短的窗去量瞬時斜率，量到的是階梯的垂直邊（會是上限的 4 倍），
# 那是取樣假象不是超標。所以窗口要涵蓋好幾個控制週期。
SLOPE_WINDOW_S = 0.2
SLOPE_TOL = 1.10   # 窗口內仍有不到一個控制週期的離散誤差


def max_slope(series, dt=1.0 / 200.0):
    """用 SLOPE_WINDOW_S 的滑動窗量最大變化率"""
    k = max(1, int(round(SLOPE_WINDOW_S / dt)))
    worst = 0.0
    for (t0, v0), (t1, v1) in zip(series, series[k:]):
        if t1 > t0:
            worst = max(worst, abs(v1 - v0) / (t1 - t0))
    return worst


# ---------------------------------------------------------------- 1
print("\n[1] 起步：加速度不可以超過 DRIVE_ACCEL_MPS2")
bot = make_ready_bot()
bot.set_cmd_vel(1.0, 0.0, 0.0)
series = sample_ramp(bot, 3.0)
slope = max_slope(series)
print(f"        指令 0 -> 1.0 m/s，實測最大斜率 {slope:.3f} m/s^2"
      f"（上限 {M.DRIVE_ACCEL_MPS2}）")
check(slope <= M.DRIVE_ACCEL_MPS2 * SLOPE_TOL,
      f"加速斜率沒有超標（{slope:.3f} <= {M.DRIVE_ACCEL_MPS2}）")
check(abs(bot.ramp[0] - 1.0) < 1e-6, "最後有確實到達指令值 1.0 m/s（不是永遠追不到）")

# 斜率限制是有作用的，不是形同虛設：1.0 m/s 除以 1.0 m/s^2 至少要 1 秒
t_reach = next((t for t, v in series if v >= 0.999), None)
print(f"        到達 1.0 m/s 花了 {t_reach:.2f} 秒")
check(t_reach is not None and t_reach >= 1.0 / M.DRIVE_ACCEL_MPS2 * 0.98,
      f"起步確實被拉長到 {1.0 / M.DRIVE_ACCEL_MPS2:.2f} 秒左右，不是瞬間到位")

# ---------------------------------------------------------------- 2
print("\n[2] 煞車：減速度不可以超過 DRIVE_DECEL_MPS2")
bot.set_cmd_vel(0.0, 0.0, 0.0)
series = sample_ramp(bot, 3.0)
slope = max_slope(series)
print(f"        指令 1.0 -> 0 m/s，實測最大斜率 {slope:.3f} m/s^2"
      f"（上限 {M.DRIVE_DECEL_MPS2}）")
check(slope <= M.DRIVE_DECEL_MPS2 * SLOPE_TOL,
      f"減速斜率沒有超標（{slope:.3f} <= {M.DRIVE_DECEL_MPS2}）")
check(abs(bot.ramp[0]) < 1e-6, "最後確實停下來")

# ---------------------------------------------------------------- 3
print("\n[3] 保護動作（急停 / 轉向板失效）要立刻歸零，不走斜率")
bot = make_ready_bot()
bot.set_cmd_vel(1.0, 0.0, 0.0)
run(bot, 3.0)
v_before = bot.ramp[0]
bot.set_estop(True)
bot.step(1.0 / 200.0)   # 只跑一拍
bot.step(1.0 / 200.0)   # 去彈跳需要幾拍，再給一點
run(bot, 0.05)
print(f"        急停前 {v_before:.3f} m/s -> 急停後 {bot.ramp[0]:.4f} m/s")
check(v_before > 0.9, "前置條件：確實在全速")
check(abs(bot.ramp[0]) < 1e-6,
      "急停的那一瞬間速度就是零（保護動作不可以為了平順而拖 0.5 秒）")

# ---------------------------------------------------------------- 4
print("\n[4] 死結防護：閘門關著、速度被壓成零時，轉向還是要跟著上層走")
# 這一項失敗 = 車子一停下來就再也轉不了向，而閘門要等轉到位才開 -> 永遠卡死
bot = make_ready_bot()
bot.set_cmd_vel(0.4, 0.0, 0.0)      # 先往前走，讓輪子對到「前進」的角度
run(bot, 3.0)
fwd_target = list(bot.target_deg)
print("        前進時的目標角 " + " ".join(f"{a:7.2f}" for a in fwd_target))

# 突然改成橫move：四顆要轉 90 度，閘門會關、速度會被壓成零
bot.set_cmd_vel(0.0, 0.4, 0.0)
run(bot, 0.3)
gated = bot.gate_stop
ramp_now = math.hypot(bot.ramp[0], bot.ramp[1])
print(f"        改成橫移 0.3 秒後：閘門關={gated}  底盤速度={ramp_now:.4f} m/s")

run(bot, 4.0)
side_target = list(bot.target_deg)
print("        橫移時的目標角 " + " ".join(f"{a:7.2f}" for a in side_target))
moved = max(abs(s - f) for s, f in zip(side_target, fwd_target))
check(moved > 45.0,
      f"目標角確實跟著上層改了（最大變化 {moved:.1f}° > 45°）—— 沒有卡在舊角度")

# 而且最後一定要真的動起來（閘門有開、速度有長回來）
final_speed = math.hypot(bot.ramp[0], bot.ramp[1])
print(f"        穩定後底盤速度 {final_speed:.3f} m/s（指令 0.4）")
check(final_speed > 0.35, f"轉到位之後速度有長回來（{final_speed:.3f} > 0.35），閘門沒有卡住")

# ---------------------------------------------------------------- 5
print("\n[5] 大變向：輪胎不可以橫著刮地")
# 量的是**側向刮地速度** = 輪速 x sin(角度誤差)，也就是輪胎真正被橫拖的速度，
# 那才是把車體推歪、造成跳動的東西。
#
# ⚠ 不能斷言「誤差超過門檻就完全不出力」。閘門關的時候是把底盤速度目標設成 0，
#   再用 DRIVE_DECEL_MPS2 平順減速，不是一刀切斷 —— 切斷在 VESC 的速度模式下
#   等於全力煞車，那本身就是一次跳動（正是斜率限制要消滅的東西）。所以減速的
#   那 0.2~0.3 秒裡輪子還在轉、誤差也還在長，這是設計上刻意的取捨。
#   有意義的驗收是「和沒有閘門相比刮得少很多」，不是「完全沒刮」。


def module_scrub(bot, i):
    """模組 i 的側向滑移速度 (m/s)：車體在該模組位置的實際速度，
    投影到「垂直於輪子實際指向」的方向 —— 那就是輪胎真的在地上被橫拖的量。

    ⚠ 一定要用**實際指向**（reported_deg）和**車體實際速度**（true_vel），
      不能用目標角或指令速度。輪胎不是被指令刮的，是被車體的動量刮的。
    """
    vx, vy, wz = bot.true_vel
    mx, my = bot.kin.module_x[i], bot.kin.module_y[i]
    vxi = vx - wz * my          # 車體在模組 i 位置的速度（車體座標）
    vyi = vy + wz * mx
    # 輪子實際指向（車體座標）。物理偏移才是真實朝向，不是設定值
    a = math.radians(bot.reported_deg[i] + bot.physical_offset_deg[i])
    # 垂直分量 = 叉積大小
    return abs(vxi * math.sin(a) - vyi * math.cos(a))


def peak_scrub(**overrides):
    """90 度大變向，回傳過程中四顆的最大側向滑移速度 (m/s)。

    overrides 用來暫時改 swerve_model 的模組層級常數，跑完會還原。
    """
    saved = {k: getattr(M, k) for k in overrides}
    for k, v in overrides.items():
        setattr(M, k, v)
    try:
        b = make_ready_bot()
        b.set_cmd_vel(0.5, 0.0, 0.0)
        run(b, 3.0)
        b.set_cmd_vel(0.0, 0.5, 0.0)
        worst = 0.0
        dt = 1.0 / 200.0
        for _ in range(int(3.0 / dt)):
            b.step(dt)
            worst = max(worst, max(module_scrub(b, i) for i in range(M.N)))
        return worst
    finally:
        for k, v in saved.items():
            setattr(M, k, v)


scrub_now = peak_scrub()
# 對照組：門檻拉到天上 = 閘門永不關、也不會先停車再轉 = 改動前的行為
scrub_before = peak_scrub(DRIVE_ALIGN_GATE_DEG=1e6, DRIVE_ALIGN_RELEASE_DEG=1e6,
                          DRIVE_REPOINT_SPEED_MPS=0.0)
print(f"        改動後：最大側向滑移 {scrub_now:.4f} m/s")
print(f"        改動前：最大側向滑移 {scrub_before:.4f} m/s")
check(scrub_before > 0.05, f"前置條件：改動前確實會刮（{scrub_before:.4f} m/s），"
                           "這一項要是失敗代表這個測試沒測到東西")
check(scrub_now < scrub_before * 0.5,
      f"側向滑移壓到不到一半（{scrub_now:.4f} < {scrub_before * 0.5:.4f} m/s）")

# 穩定之後誤差一定要收進 RELEASE 以內，否則閘門會一直開開關關
bot = make_ready_bot()
bot.set_cmd_vel(0.5, 0.0, 0.0)
run(bot, 5.0)
err_steady = max(abs(t - r) for t, r in zip(bot.target_deg, bot.reported_deg))
print(f"        穩定行駛時的轉向誤差 {err_steady:.2f}°")
check(err_steady <= M.DRIVE_ALIGN_RELEASE_DEG,
      f"穩定行駛時誤差收在 RELEASE 以內（{err_steady:.2f}° <= "
      f"{M.DRIVE_ALIGN_RELEASE_DEG}°），閘門不會一直開開關關")

# ---------------------------------------------------------------- 6
print("\n[6] 閘門遲滯：RELEASE 一定要比 GATE 小，否則會在門檻上抖")
check(M.DRIVE_ALIGN_RELEASE_DEG < M.DRIVE_ALIGN_GATE_DEG,
      f"RELEASE({M.DRIVE_ALIGN_RELEASE_DEG}) < GATE({M.DRIVE_ALIGN_GATE_DEG})")
# 門檻也要明顯大於轉向板的容差，否則位置環在死區裡的正常漂移就會把閘門關掉
check(M.DRIVE_ALIGN_RELEASE_DEG > M.STEER_TOLERANCE_DEG,
      f"RELEASE({M.DRIVE_ALIGN_RELEASE_DEG}) > 轉向容差({M.STEER_TOLERANCE_DEG})"
      " —— 否則位置環停在死區邊緣就足以把閘門關掉，車子會一直停一下走一下")

# ---------------------------------------------------------------- 7
print("\n[7] 換解遲滯：在兩個等價解等距的臨界點上不可以反覆翻 180 度")
# 直接對運動學做：把目前角度固定在「離兩個解各 90 度」的臨界點，
# 讓目標方向在臨界點兩側來回抖動，數翻面次數。
kin = M.SwerveKinematics()
kin.set_chassis_param(M.CHASSIS_WHEELBASE_M, M.CHASSIS_TRACK_M, M.MAX_WHEEL_SPEED_MPS)
kin.set_steer_range(M.STEER_ANGLE_MIN_DEG, M.STEER_ANGLE_MAX_DEG, M.STEER_ANGLE_MARGIN_DEG)
kin.set_mount_offsets(M.MOUNT_OFFSET_DEG)
kin.set_limit_keepout(M.STEER_KEEPOUT_DEG)


def count_flips(hysteresis_deg):
    kin.set_flip_hysteresis(hysteresis_deg)
    kin.last_flip = [0] * M.N
    current = 135.0                  # park，離 45 和 225 各 90 度
    prev = None
    flips = 0
    for n in range(200):
        # 目標角在臨界點兩側 ±0.2 度來回（模擬上層速度的微小雜訊）
        desired = 45.0 + (0.2 if n % 2 else -0.2)
        r = kin.resolve_angle(0, desired, current)
        if r is None:
            continue
        if prev is not None and abs(r[0] - prev) > 90.0:
            flips += 1
        prev = r[0]
    return flips


flips_off = count_flips(0.0)
flips_on = count_flips(M.STEER_FLIP_HYSTERESIS_DEG)
print(f"        遲滯關閉：翻面 {flips_off} 次 / 200 拍")
print(f"        遲滯 {M.STEER_FLIP_HYSTERESIS_DEG:.0f}°：翻面 {flips_on} 次 / 200 拍")
check(flips_off > 50, f"前置條件：沒有遲滯時確實會狂翻（{flips_off} 次），"
                      "這一項要是失敗代表這個測試沒測到東西")
check(flips_on == 0, f"加上遲滯之後完全不翻（{flips_on} 次）")

# ---------------------------------------------------------------- 8
print("\n[8] 轉向目標角不可以受「斜率限制器的狀態」影響（抽搐迴歸）")
# 2026-08-16 實車回報的迴歸：轉向還沒到位時再動搖桿，輪子會在轉向過程中抽搐；
# 而且斜走走不出來。原因是當時把**斜率限制後**的速度餵進 inverse() 去解角度。
# 角度解取決於 vx:vy:wz 的比例，而斜率限制是各軸獨立的 —— 過渡期間那個比例
# 和上層指令不一樣，於是「照 ramp 算的角度」和「照指令算的角度」是兩個不同的
# 值，閘門一開一關就會在兩者之間跳。
#
# 這一項直接驗那個不變量：同一個指令，不管斜率限制器現在停在哪，
# 解出來的目標角都必須一模一樣。
kin2 = M.SwerveKinematics()
kin2.set_chassis_param(M.CHASSIS_WHEELBASE_M, M.CHASSIS_TRACK_M, M.MAX_WHEEL_SPEED_MPS)
kin2.set_steer_range(M.STEER_ANGLE_MIN_DEG, M.STEER_ANGLE_MAX_DEG, M.STEER_ANGLE_MARGIN_DEG)
kin2.set_mount_offsets(M.MOUNT_OFFSET_DEG)
kin2.set_limit_keepout(M.STEER_KEEPOUT_DEG)
kin2.set_flip_hysteresis(M.STEER_FLIP_HYSTERESIS_DEG)
cur = [135.0] * M.N

# 指令是 vx=0.4, wz=1.0；ramp 在過渡期間可能是任何「比例不同」的中間值
cmd = (0.4, 0.0, 1.0)
kin2.seed_angles(cur)
ang_cmd = [s[0] for s in kin2.inverse(*cmd, cur)]
worst = 0.0
for frac_v, frac_w in ((1.0, 0.3), (0.3, 1.0), (0.5, 0.9), (0.05, 1.0), (1.0, 0.05)):
    ramped = (cmd[0] * frac_v, cmd[1] * frac_v, cmd[2] * frac_w)
    a = [s[0] for s in kin2.inverse(*ramped, cur)]
    worst = max(worst, max(abs(x - y) for x, y in zip(a, ang_cmd)))
print(f"        照 ramp 中間值解出來的角度，和照指令解的最大差 {worst:.2f}°")
check(worst > 5.0,
      f"前置條件：ramp 的比例失真確實會改變角度解（{worst:.1f}°），"
      "所以「角度只能用指令算」不是多餘的講究")

# 真正的驗收：整車跑一次，轉向過程中目標角不可以來回擺盪
bot = make_ready_bot()
bot.set_cmd_vel(0.4, 0.0, 0.0)
run(bot, 4.0)
bot.set_cmd_vel(0.3, 0.3, 0.8)     # 轉向還沒到位時就改成斜走 + 自轉
dt = 1.0 / 200.0
series = []
for _ in range(int(4.0 / dt)):
    bot.step(dt)
    series.append(bot.target_deg[0])

# 數「變化方向反轉」的次數。平滑的轉向最多反轉一兩次；抽搐會有一大堆。
revs = 0
prev_dir = 0
for a, b in zip(series, series[1:]):
    d = b - a
    if abs(d) < 1e-6:
        continue
    cur_dir = 1 if d > 0 else -1
    if prev_dir != 0 and cur_dir != prev_dir:
        revs += 1
    prev_dir = cur_dir
print(f"        轉向過程中 M0 目標角的方向反轉 {revs} 次 / {len(series)} 拍")
check(revs <= 2, f"目標角沒有來回擺盪（反轉 {revs} 次 <= 2）")

# ---------------------------------------------------------------- 9
print("\n[9] 斜走：合成方向不可以被各軸獨立的斜率限制拉歪")
bot = make_ready_bot()
bot.set_cmd_vel(0.3, 0.3, 0.0)     # 正 45 度斜走
run(bot, 6.0)
# 四顆的機械角度換成車體角度之後，應該都指向 45 度
worst_dir = 0.0
for i in range(M.N):
    g = bot.kin.local_to_global_deg(i, bot.target_deg[i])
    # 反向解會差 180 度，取「和 45 度或 225 度較近的那個」
    d = min(abs(((g - 45.0) + 180) % 360 - 180), abs(((g - 225.0) + 180) % 360 - 180))
    worst_dir = max(worst_dir, d)
print(f"        穩定後四顆指向與 45° 的最大偏差 {worst_dir:.2f}°")
check(worst_dir < 1.0, f"斜走方向正確（偏差 {worst_dir:.2f}° < 1°）")

# 而且真的走出斜線：里程計的 vx 與 vy 要相當
run(bot, 2.0)
ovx, ovy = bot.kin.odom['vx'], bot.kin.odom['vy']
print(f"        里程計 vx={ovx:.3f} vy={ovy:.3f}")
check(abs(ovx) > 0.05 and abs(ovy) > 0.05 and abs(abs(ovx) - abs(ovy)) < 0.05 * max(abs(ovx), abs(ovy)) + 0.02,
      f"真的走出 45 度斜線（vx 與 vy 相當：{ovx:.3f} vs {ovy:.3f}）")

# ---------------------------------------------------------------- 10
print("\n[10] nav2 連續指令：跑一般路徑時不可以斷斷續續")
# 自主移動的指令不是階躍的：nav2 的 velocity_smoother 以 80 Hz 輸出、
# controller 以 20 Hz 重規劃，方向是連續變的。這一項驗「到位閘門」與
# 「大變向先停車再轉」在那種輸入下不會誤觸發把車弄停。
#
# 順便把 nav2 的 max_accel「和韌體一致 / 不一致 / 完全沒有 smoother」三種
# 都跑一次做對照。
#
# 實測結論（2026-08-16）：三者的**停滯時間與平均車速幾乎一樣**，所以加速度
# 設定不一致**不會**造成頓挫 —— 那只影響「多久才到達指令速度」，也就是路徑
# 追隨的精度（nav2 以為車子衝得比實際快，彎道會切得比較外面），不影響平順度。
# 原本以為它是頓挫的原因，量了才發現不是，這裡把對照留著避免又猜錯。


def nav2_like(cruise=0.5, nav2_accel=None, seconds=10.0):
    """模擬 nav2：直線 -> 弧線 -> 直線，指令用斜率限制平滑過（就是 velocity_smoother）。

    nav2_accel = None 表示指令直接階躍（沒有 smoother）。
    回傳 (停滯秒數, 平均車速, 閘門關閉比例)
    """
    bot = SwerveRobot()
    bot.set_cmd_vel(0.0, 0.0, 0.0)
    run(bot, 2.0)

    dt = 1.0 / 200.0
    cur = [0.0, 0.0, 0.0]
    stalled = 0.0
    speed_sum = 0.0
    closed = 0.0
    n = int(seconds / dt)
    for k in range(n):
        t = k * dt
        # 路徑：0~3 s 直線加速、3~7 s 弧線（同時給 wz）、7~ 直線
        if t < 3.0:
            tgt = [cruise, 0.0, 0.0]
        elif t < 7.0:
            tgt = [cruise, 0.0, 0.6]
        else:
            tgt = [cruise, 0.0, 0.0]

        if nav2_accel is None:
            cur = list(tgt)
        else:
            for j, lim in enumerate((nav2_accel, nav2_accel, 3.0)):
                d = tgt[j] - cur[j]
                step = lim * dt
                cur[j] += max(-step, min(step, d))

        bot.set_cmd_vel(*cur)
        bot.step(dt)

        spd = math.hypot(bot.true_vel[0], bot.true_vel[1])
        speed_sum += spd
        if t > 1.0 and spd < 0.05:
            stalled += dt
        if bot.gate_stop:
            closed += dt
    return stalled, speed_sum / n, closed / seconds


for label, acc in (("nav2 加速度 1.0（和韌體一致）", 1.0),
                   ("nav2 加速度 3.5（目前 nav2 設定）", 3.5),
                   ("沒有 smoother（純階躍）", None)):
    stall, spd, closed = nav2_like(nav2_accel=acc)
    print(f"        {label:<26} 停滯 {stall:5.2f}s  平均車速 {spd:.3f}  "
          f"閘門關閉 {closed * 100:3.0f}%")

stall_ok, spd_ok, closed_ok = nav2_like(nav2_accel=1.0)
check(stall_ok < 0.05,
      f"指令連續時完全不會被迫停下來（停滯 {stall_ok:.2f} s < 0.05）")
check(closed_ok < 0.05,
      f"閘門幾乎不關（{closed_ok * 100:.0f}% < 5%）—— 一般路徑不會斷斷續續")
check(spd_ok > 0.4,
      f"平均車速有跟上（{spd_ok:.3f} > 0.4，指令 0.5）")

# ---------------------------------------------------------------- 11
print("\n[11] nav2 的加速度設定應該和韌體對齊（影響追隨精度，不影響平順度）")
# 這一項不是測程式行為，是把「兩邊該對齊的數字」印出來當提醒。
# 第 10 項已經量過：不對齊不會造成頓挫，但 nav2 會以為車子衝得比實際快，
# 規劃出來的軌跡追不上，彎道會切得比較外面。
print(f"        韌體 DRIVE_ACCEL_MPS2      = {M.DRIVE_ACCEL_MPS2}")
print(f"        韌體 DRIVE_DECEL_MPS2      = {M.DRIVE_DECEL_MPS2}")
print(f"        韌體 DRIVE_ANG_ACCEL_RPS2  = {M.DRIVE_ANG_ACCEL_RPS2}")
print("        -> nav2_params.yaml 的 velocity_smoother 要設成")
print(f"           max_accel: [{M.DRIVE_ACCEL_MPS2}, {M.DRIVE_ACCEL_MPS2}, "
      f"{M.DRIVE_ANG_ACCEL_RPS2}]")
print(f"           max_decel: [-{M.DRIVE_DECEL_MPS2}, -{M.DRIVE_DECEL_MPS2}, "
      f"-{M.DRIVE_ANG_DECEL_RPS2}]")
print(f"           max_velocity 的線速度不可超過 {M.MAX_LINEAR_SPEED_MPS}"
      f"、角速度不可超過 {M.MAX_ANGULAR_SPEED_RPS}（超過會被韌體夾掉）")
check(M.DRIVE_DECEL_MPS2 >= M.DRIVE_ACCEL_MPS2,
      "減速上限 >= 加速上限（煞得住比衝得快重要）")

# ---------------------------------------------------------------- 收尾
print()
if fails:
    print(f"有 {len(fails)} 項失敗：")
    for f in fails:
        print("  - " + f)
    sys.exit(1)
print("全部通過")
