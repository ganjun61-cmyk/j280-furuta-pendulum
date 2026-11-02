# J280 驱动安装脚本（需要管理员权限运行）
# 1) 安装 CH340 串口驱动（pnputil，静默）
# 2) 静默安装 Gowin USB 线缆驱动 V5（下载口 GWU2X 用）
# 3) 输出已登记驱动包摘要到日志

$ErrorActionPreference = 'Continue'
$log = 'C:\Users\shift\Downloads\J280_setup\install_drivers.log'

function Write-Log($msg) { $msg | Out-File -LiteralPath $log -Append -Encoding UTF8 }

"=== J280 驱动安装 $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss') ===" | Out-File -LiteralPath $log -Encoding UTF8

# 1) CH340 串口驱动
$inf = 'C:\Users\shift\Downloads\J280_setup\ch340_extract\CH341SER.INF'
if (Test-Path -LiteralPath $inf) {
    Write-Log '--- pnputil /add-driver CH341SER.INF /install ---'
    Write-Log ((& pnputil.exe /add-driver "$inf" /install 2>&1 | Out-String))
} else {
    Write-Log "缺少文件: $inf"
}

# 2) Gowin USB 线缆驱动 V5（NSIS 静默安装，最多等 180 秒）
$v5 = 'C:\Gowin\Gowin_V1.9.12.03_x64\Gowin_V1.9.12.03_x64\Programmer\driver\GowinUSBCableDriverV5_for_win7+.exe'
if (Test-Path -LiteralPath $v5) {
    Write-Log '--- 启动 GowinUSBCableDriverV5 (/S) ---'
    $p = Start-Process -FilePath $v5 -ArgumentList '/S' -PassThru
    if (-not $p.WaitForExit(180000)) {
        Write-Log 'V5 安装器 180 秒未退出（可能在等待界面操作）'
    } else {
        Write-Log ("V5 退出码: " + $p.ExitCode)
    }
} else {
    Write-Log "缺少文件: $v5"
}

# 3) 汇总
Write-Log '--- pnputil /enum-drivers 中与 CH34x / Gowin / FTDI 相关的驱动包 ---'
$drv = & pnputil.exe /enum-drivers 2>&1 | Out-String
foreach ($block in ($drv -split "`r?`n`r?`n")) {
    if ($block -match '(?i)ch34|gowin|ftdi') { Write-Log $block.Trim() }
}
Write-Log '=== 脚本结束 ==='
