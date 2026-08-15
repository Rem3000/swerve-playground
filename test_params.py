"""參數測試：模擬器的參數和韌體對得上嗎、彼此自洽嗎？

同一組物理參數散在三個地方 —— 主控韌體、轉向板韌體、這個模擬器。改一處就要改
另外兩處，否則模擬失去意義、或者兩塊板對行程的認知不一致直接打架。

這支把韌體那一側的值**抄在下面的 FIRMWARE 表裡**，拿去對 swerve_model.py。
不去讀韌體原始碼，所以 sim/ 單獨拿出來（例如公開的模擬器 repo）也跑得起來。

    python test_params.py

⚠ 代價要講清楚：下面那張表是韌體 .h 的**第二份拷貝**。
  改韌體的參數時，swerve_model.py 與這張表**兩邊都要跟著改**，不然這支測試
  只會告訴你「sim 和這張表一致」，而那張表本身可能已經過期了。
  來源檔案都標在每一列後面，對照時照著去找。

  （原本這支是直接讀 ../Esp32_wheel 與 ../esp32_rad 的 .h 自動比對，不需要
   第二份拷貝；改成寫死是為了讓只有 sim/ 的公開 repo 也能跑，不用讓訪客看到
   一支莫名其妙 skip 掉的測試。要換回自動比對的話，把這支移到 sim/ 外面即可。）
"""

import sys

import swerve_model as M

fails = []


def check(cond, msg):
    print(("  [ OK ] " if cond else "  [FAIL] ") + msg)
    if not cond:
        fails.append(msg)


# ---------------------------------------------------------------- 韌體那一側
# (韌體名稱, sim 名稱或 None 表示同名, 值, 來源檔案)
FIRMWARE = [
    ("CHASSIS_TRACK_M",           None,             0.2,   "swerve_config.h"),
    ("CHASSIS_WHEELBASE_M",       None,             0.2,   "swerve_config.h"),
    ("ESTOP_BRAKE_UNTIL_MPS",     None,             0.05,  "swerve_config.h"),
    ("ESTOP_CLEAR_DEBOUNCE_MS",   None,             50,    "swerve_config.h"),
    ("ESTOP_LATCH",               None,             1,     "swerve_config.h"),
    ("ESTOP_REARM_CMD_ZERO_MS",   None,             1000,  "swerve_config.h"),
    ("ESTOP_REARM_REQUIRE_ZERO",  None,             0,     "swerve_config.h"),
    ("ESTOP_TRIP_DEBOUNCE_MS",    None,             3,     "swerve_config.h"),
    ("LIMIT_DEBOUNCE_MS",         None,             5,     "LimitBank.h"),
    ("LIMIT_HEARTBEAT_MS",        None,             20,    "swerve_config.h"),
    ("LIMIT_LATCH_REPEATS",       None,             3,     "LimitBank.h"),
    ("LOOP_ODOM_HZ",              None,             100,   "swerve_config.h"),
    ("MAX_ANGULAR_SPEED_RPS",     None,             2,     "swerve_config.h"),
    ("MAX_LINEAR_SPEED_MPS",      None,             1,     "swerve_config.h"),
    ("MAX_WHEEL_SPEED_MPS",       None,             1.1,   "swerve_config.h"),
    ("STEER_ANGLE_MARGIN_DEG",    None,             3,     "swerve_config.h"),
    ("STEER_ANGLE_MAX_DEG",       None,             270,   "swerve_config.h + steer_config.h"),
    ("STEER_ANGLE_MIN_DEG",       None,             0,     "swerve_config.h + steer_config.h"),
    ("STEER_ENCODER_PPR",         None,             13,    "steer_config.h"),
    ("STEER_HOMING_PARK_DEG",     None,             135,   "steer_config.h"),
    ("STEER_HOMING_PWM",          None,             320,   "steer_config.h"),
    ("STEER_HOMING_REBOUND_DEG",  None,             4,     "steer_config.h"),
    ("STEER_HOMING_SLOW_PWM",     None,             170,   "steer_config.h"),
    ("STEER_HOMING_TIMEOUT_MS",   None,             20000, "steer_config.h"),
    ("STEER_KEEPOUT_DEG",         None,             30,    "swerve_config.h"),
    ("STEER_LIMIT_AGE_COMPENSATE", None,            1,     "SteerModule.h"),
    ("STEER_MAX_PWM",             None,             800,   "steer_config.h"),
    ("STEER_MEASURE_SPAN",        None,             1,     "steer_config.h"),
    ("STEER_MIN_MOVE_PWM",        None,             45,    "steer_config.h"),
    ("STEER_MOTOR_GEARBOX",       None,             71,    "steer_config.h"),
    ("STEER_PID_KD_DEF",          "STEER_PID_KD",   6.0,   "steer_config.h"),
    ("STEER_PID_KI_DEF",          "STEER_PID_KI",   0,     "steer_config.h"),
    ("STEER_PID_KP_DEF",          "STEER_PID_KP",   110,   "steer_config.h"),
    ("STEER_PWM_MAX",             None,             1023,  "SteerModule.h"),
    ("STEER_REPORT_HZ",           "LOOP_REPORT_HZ", 100,   "steer_config.h"),
    ("STEER_RING_STAGE",          None,             4.5,   "steer_config.h"),
    ("STEER_SOFT_LIMIT_DEG",      None,             10,    "steer_config.h"),
    ("STEER_SPAN_TOL_FRAC",       None,             0.25,  "steer_config.h"),
    ("STEER_TOLERANCE_DEG",       None,             1.5,   "steer_config.h"),
    ("WHEEL_DIAMETER_M",          None,             0.116, "swerve_config.h"),
]

# 安裝偏移角：韌體是四個 #define，sim 是一個 tuple
FW_MOUNT_OFFSET = (-180.0, 90.0, -90.0, 0.0)  # STEER_MOUNT_OFFSET_FL/FR/RL/RR

# 撞塊銷相對輪子朝向的角度差。韌體沒有這個常數（它不進任何運算），
# 但它和量到的開關位置一起決定了上面那組 MOUNT_OFFSET，所以放在這裡當提醒。
EXPECT_STRIKER_OFFSET = (90.0, 90.0, 90.0, 90.0)


def sim_value(name):
    return getattr(M, name, None)


# ---------------------------------------------------------------- 1
print("\n[1] 模擬器的參數 vs 韌體")
bad = []
for fw_name, sim_alias, fw_val, src in FIRMWARE:
    sim_name = sim_alias or fw_name
    v = sim_value(sim_name)
    if v is None:
        bad.append(f"{sim_name} 在 swerve_model.py 裡找不到（韌體 {fw_name} = {fw_val}，{src}）")
        continue
    if isinstance(v, bool):
        v = 1.0 if v else 0.0
    if abs(float(v) - float(fw_val)) > 1e-9:
        bad.append(f"{fw_name} = {fw_val}（{src}）但 sim.{sim_name} = {v}")
for b in bad:
    print("         " + b)
check(not bad, f"{len(FIRMWARE)} 個參數和韌體一致")

# ---------------------------------------------------------------- 2
print("\n[2] 安裝偏移角")
sim_off = tuple(M.MOUNT_OFFSET_DEG)
print(f"         韌體 STEER_MOUNT_OFFSET_* {list(FW_MOUNT_OFFSET)}")
print(f"         sim  MOUNT_OFFSET_DEG     {list(sim_off)}")
check(len(sim_off) == 4 and
      all(abs(a - b) < 1e-9 for a, b in zip(FW_MOUNT_OFFSET, sim_off)),
      "安裝偏移角一致（填錯的話實車會朝斜的方向走）")

sim_st = tuple(M.STRIKER_OFFSET_DEG)
check(len(sim_st) == 4 and
      all(abs(a - b) < 1e-9 for a, b in zip(EXPECT_STRIKER_OFFSET, sim_st)),
      f"撞塊偏移 {list(sim_st)}（CAD 確認 +90，和開關位置一起決定了上面的偏移角）")

# ---------------------------------------------------------------- 3
print("\n[3] keep-out 沒有超過行程寬度給的上限")
span = M.STEER_ANGLE_MAX_DEG - M.STEER_ANGLE_MIN_DEG
usable = span - 2 * M.STEER_ANGLE_MARGIN_DEG
ko_max = (usable - 180.0) / 2.0
print(f"         可用行程 {usable:.0f}°，收緊後要 >= 180°，所以 keep-out <= {ko_max:.0f}°")
check(M.STEER_KEEPOUT_DEG <= ko_max,
      f"STEER_KEEPOUT_DEG = {M.STEER_KEEPOUT_DEG:.0f}° <= 上限 {ko_max:.0f}°"
      "（超過的話某些方向兩個解都會被擋掉）")

# ---------------------------------------------------------------- 4
print("\n[4] 頻率相依鏈：回授不可以慢於積分")
print(f"         LOOP_ODOM_HZ={M.LOOP_ODOM_HZ:.0f}  LOOP_REPORT_HZ={M.LOOP_REPORT_HZ:.0f}")
check(M.LOOP_REPORT_HZ >= M.LOOP_ODOM_HZ,
      f"回報 {M.LOOP_REPORT_HZ:.0f} Hz >= 積分 {M.LOOP_ODOM_HZ:.0f} Hz"
      "（慢了只是拿同一筆資料重複積分）")

# ---------------------------------------------------------------- 5
print("\n[5] 慢速逼近的 PWM 要推得動馬達")
print(f"         HOMING_PWM={M.STEER_HOMING_PWM:.0f}  SLOW_PWM={M.STEER_HOMING_SLOW_PWM:.0f}"
      f"  MIN_MOVE_PWM={M.STEER_MIN_MOVE_PWM:.0f}")
check(M.STEER_HOMING_SLOW_PWM == 0 or M.STEER_HOMING_SLOW_PWM > M.STEER_MIN_MOVE_PWM,
      f"SLOW_PWM ({M.STEER_HOMING_SLOW_PWM:.0f}) > MIN_MOVE_PWM ({M.STEER_MIN_MOVE_PWM:.0f})，"
      "否則二次逼近會卡住不動")
check(M.STEER_HOMING_PWM > M.STEER_MIN_MOVE_PWM,
      f"HOMING_PWM ({M.STEER_HOMING_PWM:.0f}) > MIN_MOVE_PWM ({M.STEER_MIN_MOVE_PWM:.0f})")

# ---------------------------------------------------------------- 6
print("\n[6] 刻度：減速比與編碼器推出來的 ticks/度")
tpd = M.STEER_ENCODER_PPR * 4.0 * M.STEER_MOTOR_GEARBOX * M.STEER_RING_STAGE / 360.0
print(f"         PPR {M.STEER_ENCODER_PPR:.0f} x4 x 減速比 "
      f"({M.STEER_MOTOR_GEARBOX:.0f} x {M.STEER_RING_STAGE:.1f}) / 360 "
      f"= {tpd:.2f} ticks/度")
check(abs(tpd - M.STEER_TICKS_PER_DEG_NOMINAL) < 1e-6,
      f"和 STEER_TICKS_PER_DEG_NOMINAL ({M.STEER_TICKS_PER_DEG_NOMINAL:.2f}) 對得上")
check(M.STEER_SPAN_TOL_FRAC > 0.0,
      f"行程合理性檢查是開著的（容差 ±{M.STEER_SPAN_TOL_FRAC * 100:.0f}%）—— "
      "減速比填錯一倍就是靠它攔下來")

# ---------------------------------------------------------------- 總結
print("\n" + "=" * 64)
if fails:
    print(f"失敗 {len(fails)} 項：")
    for f in fails:
        print("   - " + f)
    print("\n注意：這支比對的是「sim vs 本檔案裡抄下來的韌體值」。")
    print("      如果你剛改過韌體，要確認改的是 swerve_model.py 還是這張表。")
    sys.exit(1)
print("全部通過")
