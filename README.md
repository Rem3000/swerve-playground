# Swerve 底盤模擬器

四輪獨立轉向底盤的模擬器，程式邏輯是從韌體 **1:1 抄過來**的，不是另外寫一套：

| 韌體 | 模擬器 |
|---|---|
| `Esp32_wheel/lib/SwerveKinematics/SwerveKinematics.cpp` | `swerve_model.SwerveKinematics` |
| `esp32_rad/lib/SteerModule/SteerModule.cpp`（位置環） | `swerve_model.SteerBoardModule` |
| `Esp32_wheel/src/bot.cpp`（`loop_bot_control`） | `swerve_model.SwerveRobot` |
| `Esp32_wheel/lib/LimitBank` + CAN `0x080` | `swerve_model.LimitLink` |

移植原則是「**連 bug 都照抄**」。改韌體公式時這邊要跟著改，否則模擬失去意義。

它主要回答三個上車前沒辦法便宜驗證的問題：

- **安裝偏移角**（四個模組朝內對稱裝）在逆運動學與里程計裡處理對了沒有
- **微動開關會不會被撞壞** —— 撞壞就要拆車
- **開機歸零**完之後，韌體以為的角度和真實角度差多少

## 快速開始

純標準函式庫，沒有外部相依。

```bash
python swerve_sim.py     # 開 tkinter 視窗，數字鍵盤操控
python test_limits.py    # 跑限位安全測試（其餘測試見下）
cd web && python serve.py   # 手機版：印出區網網址，手機開那個網址
```

## 檔案

| 檔案 | 用途 |
|---|---|
| `swerve_model.py` | 純邏輯，沒有 GUI。運動學 + 轉向板位置環 + 車體物理 |
| `swerve_sim.py` | tkinter 視窗（桌面版） |
| `test_*.py` | 五支自動測試，見下一節 |
| `web/index.html` | 手機版，瀏覽器裡用 Pyodide 跑 `swerve_model.py` |
| `web/serve.py` | 把 `web/` 開成 HTTP 服務，讓手機連進來 |
| `build_exe.py` | 把桌面版打包成單一 exe |
| `web/make_qr.py` | 重產公開網址的 QR code（換網址才要跑） |

## 測試

沒有 runner，就是五支獨立腳本，全過印「全部通過」，有失敗 `sys.exit(1)`。

| 腳本 | 守的是什麼 |
|---|---|
| `test_model.py` | 運動學：逆→正閉環、偏移角轉換、齒隙與取樣率對里程計的影響 |
| `test_limits.py` | **限位安全**：任何操作都不可以壓到微動開關 |
| `test_homing.py` | **開機歸零**：歸零後的角度精度、刻度校正、CAN 延遲補償 |
| `test_estop.py` | **急停**：閂鎖、重新致能條件、去彈跳不對稱 |
| `test_input.py` | 數字鍵盤對應 |

> `test_input.py` 第 10 項在 Linux 上必然失敗（它把 Windows VK code 灌進 Tk 再讀
> keysym），那是環境差異不是回歸。

## 桌面版操作

| 鍵 | 動作 |
|---|---|
| `8` `2` `4` `6` | 前 / 後 / 左平移 / 右平移 |
| `7` `9` `1` `3` | 四個斜向 |
| `0` / `.` | 逆時針 / 順時針自轉 |
| `5` | 全停 |
| `+` `-` | 速度倍率（0.1 ~ 1.0） |
| `C` | **跑一次開機歸零**（四個模組從不同起點，含開機就壓在開關上的） |
| `X` | **全方向掃描自檢**（30 秒，自動確認會不會壓到開關） |
| `R` / `O` | 全部重置 / 只把里程計歸零 |
| `T` / `P` | 開關軌跡 / 暫停 |
| `M` | 注入 FL +15° 的校正誤差（看里程計會怎麼飄） |
| `H` / `Esc` | 開關說明 / 離開 |

按鍵可以疊加，例如 `8` + `0` 就是邊前進邊自轉。沒有數字鍵盤的話
`W A S D` + `Q E` 或方向鍵也可以，`空白鍵` 等同 `5`。

### 底盤視圖（右）＝ 限位機構

每個模組畫的是**實車的限位機構**，不是抽象示意圖：

- **亮色方塊** = 撞塊（輪叉上的圓柱銷），跟著模組一起轉
- **藍色小方塊** = 微動開關，固定在車架上不會轉；**灰色方塊** = 硬擋塊
- **灰扇形** = 撞塊可以走的 **270°**；**紅色斜線區** = 到不了的 90° 死角
- **兩段彩色圓弧** = 撞塊離兩顆開關**各還剩幾度**，在撞塊處交會。綠→黃→紅，
  哪一段變短就是往那顆開關靠近了。兩段加起來永遠是 270°
- **黃帶** = 程式主動避開的 keep-out 區（30°）；**紅帶** = 安全邊界（3°），
  程式絕不會下達落在裡面的目標角度
- **紫弧** = 輪子能指向的角度範圍，輪子那條線的粗端永遠落在上面
- **黃色箭頭** = 這顆輪子實際把車往哪推（輪速為負時和輪子指向相反）

> 兩顆開關在車體座標上看起來只差 90°，但撞塊是繞**另外那一邊**走 270° 到第二顆
> 的，中間那 90° 缺口它永遠不會經過，所以不會誤觸另一顆開關。
>
> 撞塊偏移（`STRIKER_OFFSET_DEG = -45°`）是由「實車俯視圖量到的開關角度」配上
> mount offset 反推出來的唯一解。它**不進入任何運算** —— 機械角度的刻度
> （0 = 碰 min 開關、270 = 碰 max 開關）本來就吸收掉了機構細節 —— 純粹是為了把
> 限位機構畫在正確的位置。算出來的死角剛好朝車體中心，正是開關裝在內側的必然
> 結果，可以拿來當佐證。

### 世界視圖（左）

綠色是真實軌跡、橘色是里程計推算的。兩條應該幾乎重疊；按 `M` 注入校正誤差就會
看到橘線慢慢飄開，那就是偏移角填錯時的症狀。

### 限位安全面板（右下）

- **微動開關觸發次數** —— 正常運轉時這個數字必須永遠是 **0**
- **全程最小 clearance** —— 從開機到現在離開關最近曾經到過幾度

直接掃這張即可測試（或開
<https://rem3000.github.io/swerve-playground/web/>）：

![手機模擬器](web/qr.png)

## 韌體看得到的 vs 物理真實的

模擬器刻意把這兩套狀態分開，不然測不出「歸零準不準」：

| 韌體看得到 | 物理真實 |
|---|---|
| `angle_deg` 由編碼器推算的角度 | `true_deg` 模組真正的角度 |
| `ticks_per_deg` 刻度（可能填錯） | `HW_TICKS_PER_DEG` 真實刻度 |
| `lim_min/max` 經 CAN 傳過來的開關狀態 | 撞塊有沒有真的壓到開關 |

`swerve_model.py` 裡的 `HW_*` 常數就是拿來製造兩者落差的，都可以調大來看系統撐不
撐得住：

- `HW_TICKS_PER_DEG` 真實刻度（預設比估算值大 4%，模擬減速比填不準）
- `HW_BACKLASH_DEG` 齒隙 1.2°（編碼器在馬達側，輪子在輸出側）
- `HW_HARD_STOP_OVERTRAVEL_DEG` 開關觸發後還有 2.5° 才頂到硬擋塊

### 開關的傳輸鏈（`LimitLink`）

微動開關實體接在**主控板**，轉向板要靠 CAN `0x080` 才知道開關被壓下，所以「撞塊
碰到開關」和「歸零狀態機看到」之間隔了一整條路徑，模擬器把它建出來了：

```
true_deg 碰到 0/270  ->  主控 200 Hz 掃描  ->  5 ms 去彈跳
                     ->  有變化立刻送 CAN（+0.5 ms）  ->  轉向板 200 Hz 消化
```

相關參數：`LIMIT_POLL_HZ`、`LIMIT_DEBOUNCE_MS`、`LIMIT_CAN_LATENCY_MS`、
`LIMIT_HEARTBEAT_MS`、`LIMIT_LATCH_REPEATS`，以及補償開關
`STEER_LIMIT_AGE_COMPENSATE`（對應韌體同名巨集）。

一個 `LimitLink` 管一個模組的兩顆開關。實車是四個模組共用同一個訊框，拆開模擬會
少掉「別的模組觸發時順便把我這顆也早一點送到」，所以模擬的延遲**略偏悲觀** ——
對安全測試而言方向是對的。

## 三層限位防護

實車最怕的就是把微動開關撞壞，所以做了三層，任何一層失效另外兩層還在：

| 層 | 在哪 | 做什麼 |
|---|---|---|
| 1. 挑解時避開 | `SwerveKinematics::resolve_angle` | 把可用窗口從 `[3,267]` 收緊成 `[33,237]`，硬性排除靠近開關的解 |
| 2. 送 CAN 前夾限 | `bot.cpp` | 防禦性 clamp，上游算錯也不會把「去撞開關」的指令送出去 |
| 3. 轉向板軟限位 | `SteerModule::run_position_pid` | 越接近末端越壓低往末端的出力，PID 過衝也頂不到開關 |

第 1 層之所以能保證有效，是因為可用行程 264° ≥ 180° + 2×30°，所以「θ 與 θ+180
至少有一個落在收緊後的窗口內」仍然成立，永遠找得到安全解。

> 注意第 1 層一定要用**硬性篩選**，不能用「越近罰越重」的軟性成本。換到另一個解
> 固定要轉 180°，任何小於 180 的懲罰在連續轉向時都推不動它，結果就是輪子還是會
> 一路貼到 clearance=0 才被迫翻面，等於白做。`test_limits.py` 的第 5 項就是在對照
> 這件事（關閉時 0.00°、開啟時 30.00°）。

## 幾個可以拿來對答案的姿態

| 指令 | 四顆機械角 | 說明 |
|---|---|---|
| 原地順時針自轉 | 全部 ≈ 0° | 歸零位置本來就是 X 形，也就是自轉姿態 |
| 原地逆時針自轉 | 全部 ≈ 180° | 同一條線的另一端，輪速取負 |
| 直線前進 | FL 45°、FR 135°、RL 135°、RR 45°/225° | 後輪會用反向解 |

RR 拿到 45°（反向）或 225°（正解）都對 —— 兩者差 180° 又把輪速取負，物理上是同一
件事，程式挑離目前角度近的那個。

## 附錄

### 打包成 exe

```bash
pip install pyinstaller
python build_exe.py       # 產物 dist/SwerveSim.exe，單檔約 8.6 MB
```

目標電腦不需要裝 Python。

### 發佈到 GitHub Pages

`sim/` 可以單獨推到一個**公開** repo 掛 Pages，韌體與 `pinout/`、`tools/` 全留在
私有 repo。用 `git subtree` 推，不要用複製的 —— 複製出來的 `swerve_model.py` 遲早
會跟正本分家，那正是這整套設計要避開的事。

```bash
git remote add sim-public git@github.com:<你>/<新 repo>.git   # 一次性
git subtree push --prefix=sim sim-public main                  # 每次更新
```

Pages 設定：Settings → Pages，Source 選 `Deploy from a branch`、branch `main` /
`(root)`。網址是 `https://<你>.github.io/<新 repo>/web/`。

- `.nojekyll` 不能刪，不然 Jekyll 會插手處理檔案
- 換網址之後 QR 要重產：`pip install segno && cd web && python make_qr.py`
  （`qr.png` 給 README 嵌入、`qr.svg` 印大張不會糊，兩個都進版控）

### 鍵盤沒反應時

不同平台 / NumLock 狀態下 Tk 給的 keysym 不一樣（`KP_8`、`KP_Up`、`8` 都可能），
三種都已經接住了。真的還是有問題就查實際收到什麼：

```bash
set SWERVE_KEYLOG=keys.txt
python swerve_sim.py
```

按幾下之後看 `keys.txt`，把裡面出現的 keysym 加進 `swerve_sim.py` 的
`TRANSLATE_KEYS` / `ROTATE_KEYS` 即可。
