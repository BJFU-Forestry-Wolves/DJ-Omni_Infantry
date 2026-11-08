# sync-elf.ps1
# 把 Keil 的 .axf(本身就是 ELF) 同步成本目录下的 .elf，供 VarScope 选择。
# Keil 每次重新编译后跑一次即可。
#
# 用法（在本目录下）：
#   pwsh -File .\sync-elf.ps1
#   pwsh -File .\sync-elf.ps1 -WhatIf   # 只看会复制什么
[CmdletBinding(SupportsShouldProcess = $true)]
param()

$ErrorActionPreference = 'Stop'

# 工程的 MDK-ARM 根目录（本脚本所在目录的上一级）
$mdkRoot = Split-Path -Parent $PSScriptRoot

$projects = @(
    @{ Name = 'gimbal_task'; Axf = Join-Path $mdkRoot 'gimbal_task\gimbal_task.axf' }
    @{ Name = 'gc_chass';    Axf = Join-Path $mdkRoot 'gc_chass\MDK-ARM\gc_chass\gc_chass.axf' }
)

$any = $false
foreach ($p in $projects) {
    if (-not (Test-Path $p.Axf)) {
        Write-Warning "[$($p.Name)] 找不到: $($p.Axf)"
        continue
    }

    $axf  = Get-Item $p.Axf
    $dest = Join-Path $PSScriptRoot "$($p.Name).elf"

    # 校验确实是 ELF（0x7F 'E' 'L' 'F'）
    $fs  = [System.IO.File]::OpenRead($axf.FullName)
    try {
        $magic = New-Object byte[] 4
        [void]$fs.Read($magic, 0, 4)
    } finally { $fs.Dispose() }

    if (-not ($magic[0] -eq 0x7F -and $magic[1] -eq 0x45 -and $magic[2] -eq 0x4C -and $magic[3] -eq 0x46)) {
        Write-Warning "[$($p.Name)] 不是 ELF 文件，跳过: $($p.Axf)"
        continue
    }

    $stale = $true
    if (Test-Path $dest) {
        $stale = (Get-Item $dest).LastWriteTime -lt $axf.LastWriteTime
    }

    if (-not $stale) {
        Write-Host "[$($p.Name)] 已是最新: $dest"
        continue
    }

    if ($PSCmdlet.ShouldProcess($dest, "从 $($p.Axf) 同步")) {
        Copy-Item $axf.FullName $dest -Force
        $t = (Get-Item $dest).LastWriteTime
        Write-Host "[$($p.Name)] 已更新: $dest  ($([math]::Round((Get-Item $dest).Length/1KB)) KB, $t)"
        $any = $true
    }
}

if (-not $any) { Write-Host '没有需要更新的 .elf。' }
