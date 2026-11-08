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
| `sync-elf.ps1` | 重新编译后把 `.axf` 同步成 `.elf` |

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

源码在 `C:\Users\ghko5\Documents\Github\VarScope-src`（来自 `github.com/Wheat-001/VarScope`），
改动只涉及 `varscope/ui/main_window.py`，补丁见 `varscope-history-autofit.patch`。

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

## 已知环境问题：当前探针连不上 OpenOCD

本机探针是 **Horco CMSIS-DAP v2**（`VID_FAED:PID_4870`，序列号 `507874000318`），
它是复合设备：MI_00 = CMSIS-DAPv2 (WinUSB)、MI_01 = HID(v1)、MI_02 = 串口 COM16。

实测**两个后端都失败**，与 cfg 无关：

```
# v2 bulk 后端
Info : Using CMSIS-DAPv2 interface with VID:PID=0xfaed:0x4870, serial=507874000318
Debug: cmsis_dap_usb_read(): USB timeout @ 0
Debug: cmsis_dap_usb_read(): command write USB timeout @ 0
Error: CMSIS-DAP command CMD_INFO failed.

# HID 后端
Error: error writing data: hid_write/WaitForSingleObject: (0x000003E5)
Error: CMSIS-DAP command CMD_INFO failed.
```

`CMD_INFO` 写下去了但 6 秒内**没有任何回应**。已用两个 OpenOCD 验证，排除版本问题：

| OpenOCD | 版本 | v2 bulk | HID |
|---|---|---|---|
| `C:\cliontoolchain\OpenOCD-20231002-0.12.0` (sysprogs, 2023) | 0.12.0 | ✗ CMD_INFO | ✗ 无 hidapi |
| xPack 0.12.0-7 (dev e5888bda3, 2025-10) | 0.12.0+dev | ✗ CMD_INFO | ✗ ERROR_IO_PENDING |

### 进一步验证：换工具链也一样失败

为了区分「OpenOCD 实现问题」还是「探针问题」，用三条**完全独立**的代码路径测同一个探针：

| 路径 | 接口 | 结果 |
|---|---|---|
| OpenOCD sysprogs 0.12.0 | v2 bulk | 写入成功，**读 6 s 超时** → `CMD_INFO failed` |
| OpenOCD xPack 0.12.0+dev (2025-10) | v2 bulk | 同上 |
| OpenOCD xPack | v1 HID | `hid_write/WaitForSingleObject (0x000003E5)` |
| pyOCD（libusb/pyusb，与 OpenOCD 无关） | v2 bulk | **`ep_out.write` 直接 `USBTimeoutError 10060`** |
| 裸 hidapi，照 pyOCD 官方写法 `write([0]+cmd)` | v1 HID | **`hid write failed (-1)`**，全部 info 项无一响应 |

结论：探针能枚举、能读回字符串（`Horco` / `Horco CMSIS-DAP v1` / `v2`），
但**任何 DAP 命令都发不进去**。两个互不相关的工具链、三种 USB 路径同时失败，
说明这不是 cfg 能解决的问题，而是探针**固件或驱动**层面的故障。

### 排查顺序

1. **换 USB 线、换 USB 口**。只供电不传数据的劣质线会造成完全一致的症状。
2. **设备管理器里卸载该复合设备（勾选删除驱动）→ 拔插重插**，让 Windows 重新枚举。
   若之前用 Zadig 之类工具给 MI_00 装过 WinUSB，驱动版本不对就只超时不报错。
3. **用 Keil 建个调试会话试**。Keil 走自己的 CMSIS-DAP DLL，若 Keil 也连不上，
   基本可判定探针损坏，直接换。
4. 换到另一台电脑上试一次，排除本机 USB 驱动栈问题。
5. 若能换探针，**优先用 ST-Link**：`stm32f407-stlink.cfg` 已就绪，且本工程 Keil 配置里
   `gimbal_task.uvoptx` 的 `pMon` 本来就是 `STLink\ST-LINKIII-KEIL_SWO.dll`。
6. 确认目标板已上电、SWD 接好（SWDIO/SWCLK/GND，vtref 要有电压）。
