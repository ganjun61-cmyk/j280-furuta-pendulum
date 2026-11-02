<#
J280: live angle readout of the Interface_test UART stream (9600 8N1).

Use it while adjusting the angle-sensor coupling (user manual 3.6): the goal is
990~1040 with the rod hanging naturally and untouched. The newest value is
refreshed in place and mirrored in the window title, so it stays readable while
you work on the hardware.

Usage:
  powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\j280_watch_angle.ps1
  powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\j280_watch_angle.ps1 -Seconds 300
#>
param(
    [string]$PortName = 'COM5',
    [int]$Seconds = 3600
)

$ErrorActionPreference = 'Stop'

try { $host.UI.RawUI.WindowTitle = 'J280 angle watch' } catch { }

$port = New-Object System.IO.Ports.SerialPort($PortName, 9600, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
$port.ReadTimeout = 300
$port.Open()

Write-Host "J280 角度实时读数 - $PortName 9600 8N1 (最多 $Seconds 秒)"
Write-Host "目标 990~1040，摆杆自然下垂、手不要碰。没数据就按一下核心板 SW3。"
Write-Host ""

$sw = [System.Diagnostics.Stopwatch]::StartNew()
$win = New-Object System.Collections.Generic.List[int]
$frames = 0
$lastFrame = 0.0
$lastDraw = -1.0
$status = '等待数据 ... 按一下 SW3'

try {
    while ($sw.Elapsed.TotalSeconds -lt $Seconds) {
        try { $b = $port.ReadByte() } catch { $b = -1 }
        if ($b -ge 0) {
            $win.Add($b)
            if ($win.Count -gt 16) { $win.RemoveAt(0) }
            $c = $win.Count
            if ($c -ge 10 -and $win[$c-10] -eq 0xAA -and $win[$c-9] -eq 0x55 -and $win[$c-2] -eq 0x0D -and $win[$c-1] -eq 0x0A) {
                $angle = ([int]$win[$c-8] * 256) + [int]$win[$c-7]
                $enc1  = ([int]$win[$c-6] * 256) + [int]$win[$c-5]
                $frames++
                $lastFrame = $sw.Elapsed.TotalSeconds
                $dev = $angle - 1015
                $mark = if ($angle -ge 990 -and $angle -le 1040) { 'OK' } else { '--' }
                $status = "$mark   angle = $angle   (差 $dev)   enc1 = $enc1"
                try { $host.UI.RawUI.WindowTitle = "J280 angle = $angle  (差 $dev)  $mark" } catch { }
                $win.Clear()
            }
        }
        $now = $sw.Elapsed.TotalSeconds
        if ($now - $lastDraw -ge 0.2) {
            if ($frames -gt 0 -and ($now - $lastFrame) -gt 5) { $status = '没有数据 ... 按一下核心板 SW3' }
            Write-Host ("`r" + $status.PadRight(64)) -NoNewline
            $lastDraw = $now
        }
    }
} finally {
    $port.Close()
}

Write-Host ""
Write-Host "watch finished, frames=$frames"
