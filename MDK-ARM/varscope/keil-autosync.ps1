# keil-autosync.ps1
# 把 sync-elf.ps1 挂到 Keil 的 After-Build（编译后自动执行），或取消。
#
# 用法：
#   powershell -ExecutionPolicy Bypass -File .\keil-autosync.ps1 -Status
#   powershell -ExecutionPolicy Bypass -File .\keil-autosync.ps1 -Enable
#   powershell -ExecutionPolicy Bypass -File .\keil-autosync.ps1 -Disable
#
# 做法：直接对 .uvprojx 的 <AfterMake> 段做文本替换（不经过 XmlDocument），
# 以最大程度保留原文件的排版与编码。
param(
    [switch]$Enable,
    [switch]$Disable,
    [switch]$Status,
    [switch]$Force
)

$ErrorActionPreference = 'Stop'

$mdkRoot = Split-Path -Parent $PSScriptRoot
$syncScript = Join-Path $PSScriptRoot 'sync-elf.ps1'

$projects = @(
    Join-Path $mdkRoot 'gimbal_task.uvprojx'
    Join-Path $mdkRoot 'gc_chass\MDK-ARM\gc_chass.uvprojx'
)

# Keil 开着时改工程文件会被覆盖，先拦住（只读的 -Status 不受影响）
if (-not $Status -and -not $Force -and @(Get-Process -Name UV4 -ErrorAction SilentlyContinue).Count -gt 0) {
    Write-Host "检测到 Keil (UV4) 正在运行，请先关闭 Keil 再操作。" -ForegroundColor Red
    exit 2
}

$runCommand = 'powershell -NoProfile -ExecutionPolicy Bypass -WindowStyle Hidden -File "{0}" -Quiet' -f $syncScript

function Get-AfterMakeBlock([string]$text) {
    $startTag = '<AfterMake>'
    $endTag = '</AfterMake>'
    $i = $text.IndexOf($startTag, [System.StringComparison]::Ordinal)
    if ($i -lt 0) { return $null }
    $j = $text.IndexOf($endTag, $i, [System.StringComparison]::Ordinal)
    if ($j -lt 0) { return $null }
    return @{ Start = $i; End = $j + $endTag.Length; Text = $text.Substring($i, $j + $endTag.Length - $i) }
}

function Get-BlockState([string]$block) {
    $m = [regex]::Match($block, '<RunUserProg1>\s*(\d+)\s*</RunUserProg1>')
    $enabled = ($m.Success -and $m.Groups[1].Value -eq '1')
    $n = [regex]::Match($block, '<UserProg1Name>(.*?)</UserProg1Name>', [System.Text.RegularExpressions.RegexOptions]::Singleline)
    $name = if ($n.Success) { $n.Groups[1].Value } else { '' }
    return @{ Enabled = $enabled; Name = $name }
}

$changed = 0
$failed = 0
$anyEnabled = $false

foreach ($proj in $projects) {
    $leaf = Split-Path $proj -Leaf
    if (-not (Test-Path $proj)) {
        Write-Host "[$leaf] 跳过：文件不存在" -ForegroundColor DarkYellow
        continue
    }

    # 按原始编码读入（UTF-8 无 BOM）
    $bytes = [System.IO.File]::ReadAllBytes($proj)
    $enc = New-Object System.Text.UTF8Encoding($false)
    $text = $enc.GetString($bytes)

    $block = Get-AfterMakeBlock $text
    if ($null -eq $block) {
        Write-Host "[$leaf] 失败：找不到 <AfterMake> 段" -ForegroundColor Red
        $failed++
        continue
    }

    $state = Get-BlockState $block.Text

    if ($Status) {
        $mark = if ($state.Enabled) { '已开启' } else { '未开启' }
        Write-Host ("[$leaf] $mark")
        if ($state.Enabled) {
            Write-Host ("        命令: " + $state.Name) -ForegroundColor DarkGray
            if ($state.Name -like '*sync-elf*') { $anyEnabled = $true }
        }
        continue
    }

    $want = $Enable.IsPresent
    if ($state.Enabled -eq $want) {
        Write-Host ("[$leaf] 已经是" + $(if ($want) { '开启' } else { '关闭' }) + "状态，无需改动") -ForegroundColor DarkGray
        if ($want -and $state.Name -like '*sync-elf*') { $anyEnabled = $true }
        continue
    }

    # 在块内做替换
    $newBlock = $block.Text
    if ($want) {
        $newBlock = [regex]::Replace($newBlock, '<RunUserProg1>\s*\d+\s*</RunUserProg1>', '<RunUserProg1>1</RunUserProg1>')
        $escaped = [System.Security.SecurityElement]::Escape($runCommand)
        $newBlock = [regex]::Replace($newBlock, '<UserProg1Name>.*?</UserProg1Name>', "<UserProg1Name>$escaped</UserProg1Name>", 'Singleline')
    } else {
        $newBlock = [regex]::Replace($newBlock, '<RunUserProg1>\s*\d+\s*</RunUserProg1>', '<RunUserProg1>0</RunUserProg1>')
        $newBlock = [regex]::Replace($newBlock, '<UserProg1Name>.*?</UserProg1Name>', '<UserProg1Name></UserProg1Name>', 'Singleline')
    }

    $updated = $text.Substring(0, $block.Start) + $newBlock + $text.Substring($block.End)

    # 备份后写回（保持 UTF-8 无 BOM）
    $backup = $proj + '.bak-' + (Get-Date -Format 'yyyyMMdd-HHmmss')
    Copy-Item $proj $backup -Force
    [System.IO.File]::WriteAllText($proj, $updated, $enc)

    # 回读校验
    $check = $enc.GetString([System.IO.File]::ReadAllBytes($proj))
    $vb = Get-AfterMakeBlock $check
    $vs = if ($null -ne $vb) { Get-BlockState $vb.Text } else { $null }
    if ($null -eq $vs -or $vs.Enabled -ne $want) {
        Write-Host "[$leaf] 写入后校验失败！正在回滚..." -ForegroundColor Red
        Copy-Item $backup $proj -Force
        $failed++
        continue
    }

    Write-Host ("[$leaf] " + $(if ($want) { '已开启编译后自动同步' } else { '已取消编译后自动同步' }) + "  (备份: $(Split-Path $backup -Leaf))") -ForegroundColor Green
    $changed++
    if ($want) { $anyEnabled = $true }
}

Write-Host ''
if ($Status) {
    if ($anyEnabled) {
        Write-Host '当前：编译后会自动同步 elf。' -ForegroundColor Green
    } else {
        Write-Host '当前：没有开启自动同步，需要手动双击「同步 ELF.cmd」。' -ForegroundColor Yellow
    }
    exit 0
}

Write-Host "完成：改动 $changed 个工程，失败 $failed 个"
if ($failed -gt 0) { exit 1 }
exit 0
