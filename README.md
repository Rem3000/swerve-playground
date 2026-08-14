# Swerve 底盤模擬器

用來驗證「朝內對稱安裝」的四個轉向模組，其**安裝偏移角**在逆運動學與里程計裡
到底處理對了沒有。程式邏輯是從韌體 1:1 抄過來的，不是另外寫一套。

```
Esp32_wheel/lib/SwerveKinematics/SwerveKinematics.cpp  ->  swerve_model.SwerveKinematics
esp32_rad/lib/SteerModule/SteerModule.cpp（位置環）     ->  swerve_model.SteerBoardModule
Esp32_wheel/src/bot.cpp（loop_bot_control）            ->  swerve_model.SwerveRobot
Esp32_wheel/lib/LimitBank + CAN 0x080                  ->  swerve_model.LimitLink
```

改韌體的公式時，這邊要跟著改，否則模擬就失去意義。

## 檔案

| 檔案 | 用途 |
|---|---|
| `swerve_model.py` | 純邏輯，沒有 GUI。運動學 + 轉向板位置環 + 車體物理 |
| `swerve_sim.py` | tkinter 視窗（只用標準函式庫，沒有外部相依） |
| `test_model.py` | 運動學的自動測試 |
| `test_limits.py` | **限位安全測試**：會不會撞壞微動開關 |
| `test_homing.py` | **開機歸零測試**：歸零完之後角度準不準 |
| `test_estop.py` | **急停測試**：閂鎖與重新致能的邏輯 |
| `test_input.py` | 數字鍵盤對應的自動測試 |
| `build_exe.py` | 打包成單一 exe |
| `web/index.html` | 手機版模擬（瀏覽器裡用 Pyodide 跑 `swerve_model.py`） |
| `web/serve.py` | 把 `sim/` 開成 HTTP 服務，讓手機連進來 |

## 跑起來

```bash
python swerve_sim.py     # 開視窗
python test_model.py     # 跑運動學測試
python test_limits.py    # 跑限位安全測試
python test_homing.py    # 跑開機歸零測試
python test_estop.py     # 跑急停測試
python test_input.py     # 跑按鍵測試
```

### 手機版

```bash
cd web && python serve.py     # 印出區網網址，手機開那個網址
```

網頁不是另一套模擬 —— 它用 Pyodide（瀏覽器裡的 CPython）**直接載入 `sim/swerve_model.py`
正本**，一行都沒改寫，所以改模型手機那邊重新整理就跟著變，不會有第二份公式要同步。
`index.html` 裡只有畫面與觸控，加上一小段把模型輸出打包成 JSON 的膠水。

- 第一次開啟要從 CDN 抓 Pyodide（約 10 MB，之後瀏覽器會快取），**這一步需要網路**
- 手機要跟電腦在同一個 Wi-Fi；連不上通常是防火牆，Windows 第一次跑要允許「私人網路」
- 不能直接用 `file://` 開，`fetch` 會被 CORS 擋掉
- 模擬跑在手機自己的瀏覽器裡，`serve.py` 只負責送檔案
- `serve.py` 只開放 `web/` 底下白名單內的檔案，加上一條指向 `swerve_model.py`
  正本的路由；`sim/` 其他東西（測試、tkinter 版）不會被送出去
- 分頁切到背景時 `requestAnimationFrame` 會被瀏覽器暫停，模擬跟著停，切回來繼續

操控：左邊搖桿平移（放開回中）、右邊橫條自轉、`×0.6` 切速度倍率、`重新歸零`
真的跑一次歸零狀態機、`急停` 是閂鎖式（放開後還要零速一秒才解除，和韌體一樣）。
畫面上的圓是每個模組的撞塊軌跡：灰色是能走的 270°、暗紅是到不了的 90° 死角、
紅短線是兩顆微動開關、圓點是撞塊現在的位置（越接近開關越紅）、藍虛線是韌體
下的目標角、粗線是輪子朝向（綠 = 正轉、紅 = 反轉）。

桌面版 `swerve_sim.py` 的世界視圖、軌跡、里程計對照、掃描自檢還沒搬過去。

### 讓 Wi-Fi 以外的人也連得到（GitHub Pages）

`sim/` 這個目錄可以單獨推到一個**公開** repo 去掛 GitHub Pages，韌體、`pinout/`、
`tools/` 全都留在原本的私有 repo 裡。用 `git subtree` 推，不要用複製的 ——
複製出來的 `swerve_model.py` 遲早會跟正本分家，那正是這整套設計要避開的事。

一次性設定（先在 GitHub 上開一個空的 public repo，不要勾 README）：

```bash
git remote add sim-public git@github.com:<你>/<新 repo>.git
git subtree push --prefix=sim sim-public main
```

然後到新 repo 的 Settings → Pages，Source 選 `Deploy from a branch`、
branch `main` / `(root)`。網址會是 `https://<你>.github.io/<新 repo>/web/`。

之後每次改完模擬器，在主 repo commit 完再跑一次同一行 `git subtree push` 就更新了。

- `sim/.nojekyll` 是給 Pages 用的：不加的話 Jekyll 會插手處理檔案
- 公開 repo 拿到的是 `sim/` 的全部（含測試與 tkinter 版），這些本來就沒有機密；
  真的只想給網頁的話就改推 `sim/web`，但那樣 `swerve_model.py` 要另外想辦法帶過去
- Pages 上 `web/index.html` 會先試同目錄的 `swerve_model.py`（404），再抓
  `../swerve_model.py`（repo 根目錄，命中）。多一個 404 請求，不影響功能

## 韌體看得到的 vs 物理真實的

模擬器刻意把這兩套狀態分開，不然測不出「歸零準不準」：

| 韌體看得到 | 物理真實 |
|---|---|
| `angle_deg` 由編碼器推算的角度 | `true_deg` 模組真正的角度 |
| `ticks_per_deg` 刻度（可能填錯） | `HW_TICKS_PER_DEG` 真實刻度 |
| `lim_min/max` 經 CAN 傳過來的開關狀態 | 撞塊有沒有真的壓到開關 |

`swerve_model.py` 裡的 `HW_*` 常數就是拿來製造兩者落差的，都可以調大來看
系統撐不撐得住：

- `HW_TICKS_PER_DEG` 真實刻度（預設比估算值大 4%，模擬減速比填不準）
- `HW_BACKLASH_DEG` 齒隙 1.2°（編碼器在馬達側，輪子在輸出側）
- `HW_HARD_STOP_OVERTRAVEL_DEG` 開關觸發後還有 2.5° 才頂到硬擋塊

### 開關的傳輸鏈（`LimitLink`）

微動開關實體接在**主控板**，轉向板要靠 CAN `0x080` 才知道開關被壓下，所以
「撞塊碰到開關」和「歸零狀態機看到」之間隔了一整條路徑，模擬器把它建出來了：

```
true_deg 碰到 0/270  ->  主控 200 Hz 掃描  ->  5 ms 去彈跳
                     ->  有變化立刻送 CAN（+0.5 ms）  ->  轉向板 200 Hz 消化
```

相關參數：`LIMIT_POLL_HZ`、`LIMIT_DEBOUNCE_MS`、`LIMIT_CAN_LATENCY_MS`、
`LIMIT_HEARTBEAT_MS`、`LIMIT_LATCH_REPEATS`，以及補償開關
`STEER_LIMIT_AGE_COMPENSATE`（對應韌體同名巨集）。

一個 `LimitLink` 管一個模組的兩顆開關。實車是四個模組共用同一個訊框，拆開模擬
會少掉「別的模組觸發時順便把我這顆也早一點送到」，所以模擬的延遲**略偏悲觀**
—— 對安全測試而言方向是對的。

## 操作

| 鍵 | 動作 |
|---|---|
| `8` `2` `4` `6` | 前 / 後 / 左平移 / 右平移 |
| `7` `9` `1` `3` | 四個斜向 |
| `0` / `.` | 逆時針 / 順時針自轉 |
| `5` | 全停 |
| `+` `-` | 速度倍率 |
| `C` | **跑一次開機歸零**（四個模組從不同起點，含開機就壓在開關上的） |
| `X` | **全方向掃描自檢**（30 秒，自動確認會不會壓到開關） |
| `R` / `O` | 全部重置 / 只把里程計歸零 |
| `T` / `P` | 開關軌跡 / 暫停 |
| `M` | 注入 FL +15° 的校正誤差（看里程計會怎麼飄） |
| `H` / `Esc` | 開關說明 / 離開 |

按鍵可以疊加，例如 `8` + `0` 就是邊前進邊自轉。沒有數字鍵盤的話
`W A S D` + `Q E` 也可以。

## 畫面在看什麼

### 底盤視圖（右）＝ 限位機構

每個模組畫的是**實車的限位機構**，不是抽象示意圖：

- **亮色方塊** = 撞塊（輪叉上的圓柱銷），跟著模組一起轉。
- **藍色小方塊** = 微動開關，固定在車架上不會轉。**灰色方塊** = 硬擋塊。
- **灰扇形** = 撞塊可以走的 **270°**；**紅色斜線區** = 到不了的 90° 死角。
- **兩段彩色圓弧** = 撞塊離兩顆開關**各還剩幾度**，在撞塊處交會。
  綠→黃→紅，哪一段變短就是往那顆開關靠近了。兩段加起來永遠是 270°。
- **黃帶** = 程式主動避開的 keep-out 區（30°）；**紅帶** = 安全邊界（3°），
  程式絕不會下達落在裡面的目標角度。
- **紫弧** = 輪子能指向的角度範圍。輪子那條線的粗端永遠落在這條弧上。
- **黃色箭頭** = 這顆輪子實際把車往哪推（輪速為負時和輪子指向相反）。

> 兩顆開關在車體座標上看起來只差 90°，但撞塊是繞**另外那一邊**走 270° 到
> 第二顆的，中間那 90° 缺口它永遠不會經過，所以不會誤觸另一顆開關。
>
> 撞塊偏移（`STRIKER_OFFSET_DEG = -45°`）是由「實車俯視圖量到的開關角度」
> 配上 mount offset 反推出來的唯一解。它**不進入任何運算** —— 機械角度的
> 刻度（0 = 碰 min 開關、270 = 碰 max 開關）本來就吸收掉了機構細節 ——
> 純粹是為了把限位機構畫在正確的位置。算出來的死角剛好朝車體中心，
> 正是開關裝在內側的必然結果，可以拿來當佐證。

### 世界視圖（左）

綠色是真實軌跡、橘色是里程計推算的。兩條應該幾乎重疊；按 `M` 注入校正誤差
就會看到橘線慢慢飄開，那就是偏移角填錯時的症狀。

### 限位安全面板（右下）

- **微動開關觸發次數** —— 正常運轉時這個數字必須永遠是 **0**
- **全程最小 clearance** —— 從開機到現在離開關最近曾經到過幾度

## 幾個可以拿來對答案的姿態

| 指令 | 四顆機械角 | 說明 |
|---|---|---|
| 原地順時針自轉 | 全部 ≈ 0° | 歸零位置本來就是 X 形，也就是自轉姿態 |
| 原地逆時針自轉 | 全部 ≈ 180° | 同一條線的另一端，輪速取負 |
| 直線前進 | FL 45°、FR 135°、RL 135°、RR 45°/225° | 後輪會用反向解 |

RR 拿到 45°（反向）或 225°（正解）都對 —— 兩者差 180° 又把輪速取負，
物理上是同一件事，程式挑離目前角度近的那個。

## 三層限位防護

實車最怕的就是把微動開關撞壞，所以做了三層，任何一層失效另外兩層還在：

| 層 | 在哪 | 做什麼 |
|---|---|---|
| 1. 挑解時避開 | `SwerveKinematics::resolve_angle` | 把可用窗口從 `[3,267]` 收緊成 `[33,237]`，硬性排除靠近開關的解 |
| 2. 送 CAN 前夾限 | `bot.cpp` | 防禦性 clamp，上游算錯也不會把「去撞開關」的指令送出去 |
| 3. 轉向板軟限位 | `SteerModule::run_position_pid` | 越接近末端越壓低往末端的出力，PID 過衝也頂不到開關 |

第 1 層之所以能保證有效，是因為：可用行程 264° ≥ 180° + 2×30°，所以
「θ 與 θ+180 至少有一個落在收緊後的窗口內」仍然成立，永遠找得到安全解。

> 注意第 1 層一定要用**硬性篩選**，不能用「越近罰越重」的軟性成本。
> 換到另一個解固定要轉 180°，任何小於 180 的懲罰在連續轉向時都推不動它，
> 結果就是輪子還是會一路貼到 clearance=0 才被迫翻面，等於白做。
> `test_limits.py` 的第 4 項就是在對照這件事（關閉時 0.00°、開啟時 30.00°）。

## 打包成 exe

```bash
pip install pyinstaller
python build_exe.py
```

產物在 `dist/SwerveSim.exe`，單檔約 8.6 MB，目標電腦不需要裝 Python。

## 鍵盤沒反應時

不同平台 / NumLock 狀態下，Tk 給的 keysym 會不一樣（`KP_8`、`KP_Up`、`8` 都可能）。
三種都已經接住了。真的還是有問題的話可以查實際收到什麼：

```bash
set SWERVE_KEYLOG=keys.txt
python swerve_sim.py
```

按幾下之後看 `keys.txt`，把裡面出現的 keysym 加進 `swerve_sim.py` 的
`TRANSLATE_KEYS` / `ROTATE_KEYS` 即可。
