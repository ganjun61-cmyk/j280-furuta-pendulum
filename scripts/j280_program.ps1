<#
J280 (Gowin GW2A-55C) SRAM programming helper.

Wraps Gowin programmer_cli.exe with the settings that actually work on this PC:
  * cable must be selected explicitly: --cable-index 0 = "Gowin USB Cable(GWU2X)".
    The CLI default picks "Gowin USB Cable(FT2CH)" and every write operation
    fails immediately with exit code 50.
  * the bitstream is staged to an ASCII-only path first; programmer_cli chokes
    on non-ASCII paths (this repo lives under a Chinese folder name).

Operations (--run):
  0  Read Device Codes       (link check, no license needed)
  1  Reprogram               (reload config from external flash)
  2  SRAM Program            (default here; volatile, good for debugging)
  4  SRAM Program + Verify   (readback compare may fail if a security bit is set)
  24 SRAM Erase

Usage:
  powershell -ExecutionPolicy Bypass -File .\scripts\j280_program.ps1
  powershell -ExecutionPolicy Bypass -File .\scripts\j280_program.ps1 -Fs .\vendor\J280\key_led\impl\pnr\key_led.fs
  powershell -ExecutionPolicy Bypass -File .\scripts\j280_program.ps1 -Operation 0
#>
param(
    [string]$Fs = (Join-Path $PSScriptRoot '..\vendor\J280\Interface_test\Interface_test_new\impl\pnr\Interface_test.fs'),
    [int]$Operation = 2,
    [string]$Device = 'GW2A-55C',
    [int]$CableIndex = 0
)

$ErrorActionPreference = 'Stop'

$cli = 'C:\Gowin\Gowin_V1.9.12.03_x64\Gowin_V1.9.12.03_x64\Programmer\bin\programmer_cli.exe'
if (-not (Test-Path -LiteralPath $cli)) { throw "programmer_cli.exe not found: $cli" }

$stagedFs = $null
if ($Operation -ne 0 -and $Operation -ne 1 -and $Operation -ne 24) {
    if (-not (Test-Path -LiteralPath $Fs)) { throw "bitstream not found: $Fs" }
    $stageDir = Join-Path $env:USERPROFILE 'Downloads\j280_prog'
    New-Item -ItemType Directory -Force -Path $stageDir | Out-Null
    $stagedFs = Join-Path $stageDir ([IO.Path]::GetFileName($Fs))
    Copy-Item -LiteralPath (Resolve-Path -LiteralPath $Fs) -Destination $stagedFs -Force
    Write-Host "bitstream staged: $stagedFs"
}

$cliArgs = @('--device', $Device, '--run', $Operation, '--cable-index', $CableIndex)
if ($stagedFs) { $cliArgs += @('--fsFile', $stagedFs) }

Write-Host ("running: programmer_cli.exe " + ($cliArgs -join ' '))
& $cli @cliArgs
exit $LASTEXITCODE
