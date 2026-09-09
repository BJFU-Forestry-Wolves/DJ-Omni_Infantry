@echo off
cd /d "%~dp0"
del /s /q *.bak *.lst *.map *.obj *.axf *.o *.d *.tmp *.crf *.dep *.lnp *.plg *.htm *.uvguix.* *.__i *.tra *.iex
echo 清理完成！
pause
