# VarScope 配套文件

给 `C:\Users\ghko5\Desktop\电控小工具4\VarScope` 用的 elf + OpenOCD cfg。
VarScope 需要两样东西才能工作：**带 DWARF 的 `.elf`** 和 **OpenOCD `.cfg`**。本目录提供这两样。

## 目录内容

| 文件 | 说明 |
|---|---|
| `gimbal_task.elf` | 由 `MDK-ARM\gimbal_task\gimbal_task.axf` 复制而来（生成物，不入库） |
| `gc_chass.elf` | 由 `MDK-ARM\gc_chass\MDK-ARM\gc_chass\gc_chass.axf` 复制而来（生成物，不入库） |
| `stm32f407-stlink.cfg` | **ST-Link** 探针（SWD，hla 驱动） |
| `stm32f407-daplink.cfg` | **DAPLink / CMSIS-DAP** 探针（SWD） |
| `stm32f407-jlink.cfg` | **J-Link** 探针（SWD） |
| **`同步 ELF.cmd`** | ⭐ 双击同步（桌面也有快捷方式） |
| `sync-elf.ps1` | 同步脚本本体，被上面那个 cmd 调用 |
| `编译后自动同步-开关.cmd` | 把同步挂到 Keil「编译后自动执行」（可开可关） |
| `keil-autosync.ps1` | 上面那个开关的实现，改 `.uvprojx` 的 `<AfterMake>` |

## 同步 elf（重点）

Keil 编译只会更新 `.axf`，不会动这里的 `.elf`，所以**改完代码要同步一次**，否则 VarScope 读到的还是旧变量地址。

### 方式一：点一下（默认）

双击 **`同步 ELF.cmd`**，或者双击桌面上的 **「同步 VarScope ELF」** 快捷方式。

脚本会检查 `.axf` 是否比 `.elf` 新，只在必要时复制，然后给出结果：

```
================================================
  Keil 编译产出 .axf  -->  VarScope 使用的 .elf
================================================
[gimbal_task] 已同步 → gimbal_task.elf  (1550 KB, 11-11 19:15:28)
[gc_chass] 已是最新（10-26 19:41:40）

完成：更新 1，跳过 1，失败 0
```

同步完在 VarScope 里**重新点一次「连接」**即可，elf 路径不用改。

### 方式二：编译后全自动（更省事）

双击 **`编译后自动同步-开关.cmd`** → 选 `1` 开启。

它会在 `gimbal_task.uvprojx` 和 `gc_chass.uvprojx` 的 `<AfterMake>` 里写入一条
`powershell ... sync-elf.ps1 -Quiet`，这样 **Keil 每次编译完就自动同步，不用再点**。

几个安全设计：

- 改动前自动备份成 `*.uvprojx.bak-时间戳`
- 写完回读校验，校验不过自动回滚
- 选 `2` 取消后，工程文件**逐字节还原**（已实测：36651 B / 50457 B 完全一致）
- Keil（UV4）开着时拒绝写入，避免被 Keil 覆盖；`-Status` 只读不受影响

⚠️ 命令里写的是**绝对路径**，如果以后挪动了仓库目录，重新跑一次开关脚本即可。
也可以直接在 Keil 里关掉：`Options for Target` → `User` → 取消勾选 `Run #1`。

## 为什么 .axf 就是 .elf

Keil 的 `.axf`（ARM Executable Format）本身就是标准 **ELF32 little-endian ARM**：

```
7F 45 4C 46 01 01 01 00 ... 02 00 28 00
.  E  L  F  | ELF32 | LE   | EXEC | 0x28 = ARM
```

两个 `.axf` 的 DWARF 段也已确认存在（`.debug_info` / `.debug_abbrev` / `.debug_line` / `.symtab` / `.strtab`），
所以 VarScope 能解析全局变量。同步成 `.elf` 只是为了配合文件对话框的后缀过滤。

## 用法

1. 重新编译 Keil 工程。
2. 在本目录跑一次同步（`.axf` 是新的才会覆盖）：

   ```powershell
   powershell -NoProfile -ExecutionPolicy Bypass -File .\sync-elf.ps1
   ```

3. 打开 VarScope：
   - **elf** 一栏 → `MDK-ARM\varscope\gimbal_task.elf`
   - **cfg** 一栏 → 按你实际用的探针选一个：
     - ST-Link → `stm32f407-stlink.cfg`
     - DAPLink / CMSIS-DAP → `stm32f407-daplink.cfg`
     - J-Link → `stm32f407-jlink.cfg`
4. 点 `连接` → 搜索变量（如 `Eulr.yaw`、`gyro.z`、`motor[0].speed`）→ `开始绘图`。

配置路径会被 VarScope 的「存档」记住，下次直接切。注意存档存的是**路径**，移动工程目录要重选。

## 增强版：历史容量 + 右键自适应

原版 `VarScope.exe` 有两个限制，已在源码版修好：

| 问题 | 原版 | 修改后 |
|---|---|---|
| 主图存不了太多数据 | 硬编码 30 s / 12 000 点 | 界面可选 **30 s ~ 1 小时**，上限 **600 000 点/变量** |
| 不能一键自适应 | pyqtgraph 右键菜单被显式禁用 | 主图**右键** → 自适应（X+Y）/ 仅 Y / X 铺满 / 回到最新 |

### 附带修复：连不上板子

上游 main 分支有一个回归，`varscope/core/openocd.py` 里多了一个 `-c`：

```python
command.extend(["-c", "-c", f"gdb_port {self._gdb_port}", ...])   # ← 多了第二个 "-c"
```

拼出的命令行变成 `openocd ... -c -c "gdb_port 61709" ...`，OpenOCD 报
`Unexpected command line argument: gdb_port 61709` 后直接退出，任何板子都连不上。
发布版 v1.1 写法是正确的（`"-c", "gdb_port disabled"`），所以旧 exe 不会遇到。
删掉多余的 `"-c"` 即可。

### 验证记录

- 离屏冒烟测试：写入 200 000 样本**全部保留**（原版 12 000），自适应把 Y 收紧到数据范围。
- 冻结 exe 字节码反汇编：确认 `start()` 常量序列是 `-c / gdb_port / -c / telnet_port ...`，无重复 `-c`。
- 真实硬件全链路：elf 解析 → 变量解析 → `controller.connect()` **1.59 s 成功** →
  采集 26 帧，读到 `xMinimumEverFreeBytesRemaining = 18456`、
  `MOUSE_CHASSIS_ACCELERATE = 10.0`、`Const_GimbalPitchSpdParam[0][0] = 4.5`。

源码在 `C:\Users\ghko5\Documents\Github\VarScope-src`（来自 `github.com/Wheat-001/VarScope` 的 main 分支），
改动为 `varscope/ui/main_window.py`（+194 行）和 `varscope/core/openocd.py`（-1 行），
补丁见 `varscope-history-autofit.patch`。

### 增强版已直接装到原路径

`C:\Users\ghko5\Desktop\电控小工具4\VarScope\VarScope.exe` **已经是增强版**，
用 PyInstaller 重新打包，Python 依赖全部内置 —— 队友拿到就能用，**不用装 Python**。

| 位置 | 说明 |
|---|---|
| `电控小工具4\VarScope\` | 增强版（已替换，日常就用这个） |
| `电控小工具4\VarScope-v1.1-backup\` | 原版备份，出问题可以直接换回来 |
| `电控小工具4\VarScope-portable-v1.2-enhanced.zip` | 发给队友的分发包（47.3 MB，无个人数据） |
| `电控小工具4\VarScope\增强版说明.txt` | 给队友看的功能说明 |

重新打包步骤（改完源码后）：

```powershell
cd C:\Users\ghko5\Documents\Github\VarScope-src
python -m PyInstaller --noconfirm --clean --windowed --onedir --name VarScope `
    --icon resources\VarScope.ico `
    --add-data "Wheat.jpg;." --add-data "VarScope.ico;." `
    --add-data "runtime\default.varscope.json;runtime" `
    --exclude-module tkinter launch_varscope.py

# 关键：裁掉原版没有的 Qt 模块，否则体积会大 47 MB
$int = "dist\VarScope\_internal\PySide6"
Remove-Item "$int\opengl32sw.dll","$int\translations","$int\Qt6Quick.dll","$int\Qt6Qml.dll",
  "$int\Qt6QmlModels.dll","$int\Qt6QmlMeta.dll","$int\Qt6QmlWorkerScript.dll",
  "$int\Qt6Pdf.dll","$int\Qt6VirtualKeyboard.dll","$int\Qt6Test.dll",
  "$int\Qt6Network.dll","$int\QtNetwork.pyd","$int\QtTest.pyd" -Recurse -Force
```

未裁剪前 163.7 MB，裁剪后 110.3 MB（原版 111.4 MB）。

### 开发时从源码跑

```powershell
$env:PYTHONPATH="C:\Users\ghko5\Documents\Github\VarScope-src"
python "C:\Users\ghko5\Documents\Github\VarScope-src\launch_varscope.py" --elf ... --cfg ...
```

或双击本目录的 `启动 VarScope.cmd`（自动带上本目录的 elf/cfg）。

离屏冒烟测试（验证两项功能）：

```powershell
$env:QT_QPA_PLATFORM="offscreen"; $env:PYTHONPATH="C:\Users\ghko5\Documents\Github\VarScope-src"
python "C:\Users\ghko5\Documents\Github\VarScope-src\tests\smoke_history_and_fit.py"
```

实测结果：写入 200 000 个样本后**全部保留**（原版只留 12 000）；
`自适应（X+Y）` 把 X 铺满 `-2..202 s`、Y 收紧到 `-1.1..1.1`。

## cfg 内容说明

三个 cfg 都是 `interface/*.cfg` + `target/stm32f4x.cfg` 的标准组合：

```
adapter speed 4000        # 4 MHz SWD；接线短可提到 8000
reset_config srst_only srst_nogate
```

`target/stm32f4x.cfg` 会自动探测 Flash 容量（`flash bank ... stm32f2x 0 0`），
所以 STM32F407IG（1 MB）和 F407ZG 都能用同一份 cfg，不需要手写 `FLASH_SIZE`。

cfg 里注释只用 ASCII，避免 OpenOCD/Tcl 在中文编码上出错。

## 故障排查：`CMSIS-DAP command CMD_INFO failed`

**这个报错 99% 是有残留的 `openocd.exe` 占着调试器**，不是探针坏，也不是 cfg 错。

### 为什么会残留

VarScope 每次连接都会起一个 `openocd.exe` 子进程。旧版代码里有两处没兜住：

```python
# openocd.py: start() 里 Popen 之后如果 _connect_socket() 超时抛异常，
#            子进程没人回收 —— 直接变成孤儿进程
# controller.py: connect() 里 client 是局部变量，start() 抛异常时
#            永远不会走到 self.openocd = client，disconnect() 也就管不到它
```

孤儿进程继续占着 USB 调试器，于是**之后每次连接都失败**，表现为「一打开就连不上」。
增强版已修复这两处（`_terminate_process()` + try/except 兜底），并有回归测试
`tests/test_openocd_leak.py` 覆盖。

### 已经中招了怎么办

双击 **`清理残留 OpenOCD.cmd`**，它会列出并结束所有 `openocd.exe`，然后回 VarScope 点「连接」即可。
也可以在任务管理器里手动结束 `openocd.exe`。

### 如何确认是这个原因

两条命令就能区分：

```
# 有残留时：能读到 DPIDR，但随后命令错乱（两个进程抢同一个探针）
Info : SWD DPIDR 0x2ba01477
Error: CMSIS-DAP command mismatch. Sent 0x3 received 0x5
Error: CMSIS-DAP command CMD_DISCONNECT failed.

# 杀掉残留后：一路到底，能读内存
Info : [stm32f4x.cpu] Cortex-M4 r0p1 processor detected
0x08000000: 2000f388 0800025d
```

`2000f388` 是初始 SP，`0800025d` 是 Reset_Handler，两个都在 STM32F407 的合法范围内。

## 附：早期一次误判（保留作记录）

最早排查时曾判定这颗 **Horco CMSIS-DAP v2**（`VID_FAED:PID_4870`，序列号 `507874000318`）
「固件损坏、任何 OpenOCD 都连不上」，理由是多种工具链都在 `CMD_INFO` 处失败。
**这个结论是错的** —— 真实原因是当时有孤儿 openocd 占着探针（以及一度目标板没上电）。
清理后同一颗探针在两个 OpenOCD、pyOCD 和真实采集下都完全正常。

探针本身是复合设备：MI_00 = CMSIS-DAPv2 (WinUSB)、MI_01 = HID(v1)、MI_02 = 串口 COM16，
固件版本 `Horco v0.2`，SWD 与 JTAG 都支持。

真正**必要**的修复是上游那个多余的 `-c`（见上文「附带修复：连不上板子」）。

若清理后仍连不上，才按顺序查：换 USB 线/口 → 设备管理器重装驱动 → 确认目标板已上电
（SWDIO/SWCLK/GND，vtref 要有电压）→ 换 ST-Link（`stm32f407-stlink.cfg` 已就绪）。
