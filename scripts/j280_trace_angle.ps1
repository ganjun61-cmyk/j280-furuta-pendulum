<#
J280: log the live angle shown in the "J280" watch window's title bar.

Samples the window title every 150 ms and appends only changes, so a series of
SW1/SW2 presses can be inspected afterwards for zero drift / coupling slip.

Usage:
  powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\j280_trace_angle.ps1 -Seconds 900
  powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\j280_trace_angle.ps1 -ProcId 137680
#>
param(
    [int]$ProcId = 0,
    [int]$Seconds = 900,
    [string]$OutFile = "$env:USERPROFILE\Downloads\j280_angle_trace.txt"
)

$ErrorActionPreference = 'Stop'

function Get-AngleValue {
    param([int]$Id)
    if ($Id -gt 0) {
        $p = Get-Process -Id $Id -ErrorAction SilentlyContinue
    } else {
        $p = Get-Process | Where-Object { $_.MainWindowTitle -like 'J280 angle*' } | Select-Object -First 1
    }
    if (-not $p) { return $null }
    if ($p.MainWindowTitle -notmatch 'angle\s*=\s*(\d+)') { return $null }
    return $Matches[1]
}

"# j280 angle trace - start $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')" | Set-Content -LiteralPath $OutFile

$deadline = (Get-Date).AddSeconds($Seconds)
$last = ''
while ((Get-Date) -lt $deadline) {
    $v = Get-AngleValue -Id $ProcId
    if ($v -and $v -ne $last) {
        $last = $v
        ("{0}  angle={1}" -f (Get-Date -Format 'HH:mm:ss.fff'), $v) | Add-Content -LiteralPath $OutFile
    }
    Start-Sleep -Milliseconds 150
}

"# j280 angle trace - end $(Get-Date -Format 'HH:mm:ss')" | Add-Content -LiteralPath $OutFile
