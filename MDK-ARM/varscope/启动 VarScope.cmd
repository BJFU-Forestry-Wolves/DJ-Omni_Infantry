@echo off
chcp 65001 >nul
setlocal
rem ============================================================
rem  启动打了补丁的 VarScope（源码版）
rem  改动：1) 历史窗口可切换到 1 小时  2) 主图右键自适应（VOFA 风格）
rem  依赖：已 pip install -r requirements.txt
rem ============================================================

set "SRC=C:\Users\ghko5\Documents\Github\VarScope-src"
set "PYTHONPATH=%SRC%"

if not exist "%SRC%\launch_varscope.py" (
    echo [错误] 找不到源码目录: %SRC%
    pause
    exit /b 1
)

rem 默认加载 gimbal_task；想用 gc_chass 就改下面两行
set "ELF=%~dp0gimbal_task.elf"
set "CFG=%~dp0stm32f407-daplink.cfg"

if not exist "%ELF%" (
    echo [提示] %ELF% 不存在，先运行 sync-elf.ps1
)

python "%SRC%\launch_varscope.py" --elf "%ELF%" --cfg "%CFG%"
if errorlevel 1 pause
endlocal
