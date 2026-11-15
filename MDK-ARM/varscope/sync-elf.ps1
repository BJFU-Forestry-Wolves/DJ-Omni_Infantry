# sync-elf.ps1  (v2)
# 把 Keil 编译出的 .axf 同步成 .elf，给 VarScope 用。
# .axf 本身就是标准 ELF32 小端 ARM，改后缀只为配合 VarScope 的文件对话框。
#
# 用法：
#   powershell -ExecutionPolicy Bypass -File .\sync-elf.ps1
#   ... -WhatIf      只看会做什么，不实际复制
#   ... -Quiet       只输出结果行（供 .cmd 调用）
[CmdletBinding(SupportsShouldProcess = $true)]
param(
    [switch]$Quiet
)

$ErrorActionPreference = 'Stop'

# 工程的 MDK-ARM 根目录（本脚本在 MDK-ARM\varscope\ 下）
$mdkRoot = Split-Path -Parent $PSScriptRoot

$projects = @(
    @{ Name = 'gimbal_task'; Axf = Join-Path $mdkRoot 'gimbal_task\gimbal_task.axf' }
    @{ Name = 'gc_chass';    Axf = Join-Path $mdkRoot 'gc_chass\MDK-ARM\gc_chass\gc_chass.axf' }
)

function Write-Info($msg) { if (-not $Quiet) { Write-Host $msg } }

$updated = 0
$skipped = 0
$failed  = 0

# 中文横幅放在这里（PowerShell 输出不受 console 代码页影响），
# 这样调用它的 .cmd 可以保持纯 ASCII，避免 cmd 解析多字节文本出问题。
if (-not $Quiet) {
    Write-Host '================================================'
    Write-Host '  Keil 编译产出 .axf  -->  VarScope 使用的 .elf'
    Write-Host '================================================'
    Write-Host ''
}

# VarScope 正在运行时可能占用 elf，提前提示
$vsRunning = @(Get-Process -Name VarScope -ErrorAction SilentlyContinue).Count -gt 0
if ($vsRunning) {
    Write-Host "提示：VarScope 正在运行。若它已加载该 elf，复制可能失败——建议先关掉 VarScope。" -ForegroundColor Yellow
}

foreach ($p in $projects) {
    $name = $p.Name
    if (-not (Test-Path $p.Axf)) {
        Write-Host "[$name] 跳过：找不到 $($p.Axf)" -ForegroundColor DarkYellow
        $skipped++
        continue
    }

    $axf  = Get-Item $p.Axf
    $dest = Join-Path $PSScriptRoot "$name.elf"

    # 1) 确认是 ELF 魔数 7F 45 4C 46
    $fs = [System.IO.File]::OpenRead($axf.FullName)
    try {
        $magic = New-Object byte[] 4
        [void]$fs.Read($magic, 0, 4)
    } finally { $fs.Dispose() }

    if (-not ($magic[0] -eq 0x7F -and $magic[1] -eq 0x45 -and $magic[2] -eq 0x4C -and $magic[3] -eq 0x46)) {
        Write-Host "[$name] 跳过：$($axf.Name) 不是 ELF 文件" -ForegroundColor DarkYellow
        $skipped++
        continue
    }

    # 2) 是否比目标新
    $needsCopy = $true
    if (Test-Path $dest) {
        $needsCopy = (Get-Item $dest).LastWriteTime -lt $axf.LastWriteTime
    }
    if (-not $needsCopy) {
        Write-Host "[$name] 已是最新（$((Get-Item $dest).LastWriteTime.ToString('MM-dd HH:mm:ss'))）" -ForegroundColor DarkGray
        $skipped++
        continue
    }

    # 3) 复制
    if (-not $PSCmdlet.ShouldProcess($dest, "从 $($axf.Name) 同步")) { continue }
    try {
        Copy-Item $axf.FullName $dest -Force -ErrorAction Stop
    } catch {
        Write-Host "[$name] 失败：$($_.Exception.Message)" -ForegroundColor Red
        Write-Host "        如果 VarScope 正开着并加载了这个 elf，先关掉它再试。" -ForegroundColor Red
        $failed++
        continue
    }

    $size = [math]::Round((Get-Item $dest).Length / 1KB)
    Write-Host "[$name] 已同步 → $name.elf  (${size} KB, $((Get-Item $dest).LastWriteTime.ToString('MM-dd HH:mm:ss')))" -ForegroundColor Green
    $updated++
}

if (-not $Quiet) {
    Write-Host ''
    Write-Host "完成：更新 $updated，跳过 $skipped，失败 $failed"
}

if ($failed -gt 0) { exit 1 }
exit 0
