<#
J280: capture the Interface_test UART stream (9600 8N1) into a text file.

Frame layout (from src/uart.v): AA 55 <angle hi> <angle lo> <enc1 hi> <enc1 lo>
                               <enc2 hi> <enc2 lo> 0D 0A   (10 bytes, sent ~2/s)

Usage:
  powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\j280_serial_capture.ps1 -Seconds 600
#>
param(
    [string]$PortName = 'COM5',
    [int]$Seconds = 600,
    [string]$OutFile = "$env:USERPROFILE\Downloads\j280_serial_capture.txt"
)

$ErrorActionPreference = 'Stop'

$port = New-Object System.IO.Ports.SerialPort($PortName, 9600, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
$port.ReadTimeout = 300
$port.Open()

[IO.File]::WriteAllText($OutFile, "J280 serial capture - port=$PortName baud=9600 - started $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')`r`n")

$sw = [System.Diagnostics.Stopwatch]::StartNew()
$win = New-Object System.Collections.Generic.List[int]
$frames = 0

try {
    while ($sw.Elapsed.TotalSeconds -lt $Seconds) {
        try { $b = $port.ReadByte() } catch { continue }
        if ($b -lt 0) { continue }
        $win.Add($b)
        if ($win.Count -gt 16) { $win.RemoveAt(0) }
        $c = $win.Count
        if ($c -ge 10 -and $win[$c-10] -eq 0xAA -and $win[$c-9] -eq 0x55 -and $win[$c-2] -eq 0x0D -and $win[$c-1] -eq 0x0A) {
            $angle = ([int]$win[$c-8] * 256) + [int]$win[$c-7]
            $enc1  = ([int]$win[$c-6] * 256) + [int]$win[$c-5]
            $enc2  = ([int]$win[$c-4] * 256) + [int]$win[$c-3]
            $frames++
            $line = "{0,8:N2}s  angle={1,5}  enc1={2,6}  enc2={3,6}" -f $sw.Elapsed.TotalSeconds, $angle, $enc1, $enc2
            [IO.File]::AppendAllText($OutFile, $line + "`r`n")
            $win.Clear()
        }
    }
} finally {
    $port.Close()
}

[IO.File]::AppendAllText($OutFile, "capture finished $(Get-Date -Format 'HH:mm:ss'), frames=$frames`r`n")
