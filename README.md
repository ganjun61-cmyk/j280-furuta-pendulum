# 动中守衡 — 姿态与位置协同控制

基于高云 GW2A-55 FPGA 的旋转倒立摆（Furuta 摆）实时控制系统。传感器采集、状态估计、控制计算、轨迹生成与电机驱动全部在 FPGA 内以 500 Hz 固定周期运行。

- 自动起摆与持续倒立平衡
- 约 20° 外力推动后自动恢复
- 摆臂 90° 步进、4×4 键盘角度定位、DIP2 连续旋转与三档速度
- v6 协议 100 Hz 遥测与 Python/Tk 上位机

## 主要功能

1. **自动起摆**：摆杆由下垂状态通过摆臂泵能与掉头接杆自动抬升到倒立位置，约 1.5–2 s。
2. **持续平衡**：500 Hz 四状态 Kalman 观测 + 扰动观测器（DOB）+ LQR 状态反馈。
3. **抗扰恢复**：约 20° 瞬时推力后约 3 s 回稳。
4. **摆臂位置控制**：SW3/SW4 相对当前参考 ±90° 步进；4×4 键盘输入 0–360°、方向与 30/60/90°/s 速度，按 `D` 提交。
5. **连续旋转**：DIP2 拨到 ON 后连续旋转，短按 SW3/SW4 换向，长按升/降档（0.5× / 1.0× / 1.5×）。
6. **遥测与配置**：v6 自描述帧 100 Hz 遥测，上位机支持配置载入、实时曲线、数据录制与 A/B 分析。
7. **安全守护**：行程、速度、角度、采样超时、故障锁存与 SW2 急停。

## 硬件平台

| 项目 | 规格 |
| --- | --- |
| 主控 | GW2A-LV55PG484C8/I7，50 MHz |
| 摆杆角度 | WDD35D4 电位器（0–3.3 V 分压到 0–2.0 V）→ 3PA1030 10 位 ADC |
| 摆臂位置 | 1040 count/圈正交编码器，四倍频 + 2 µs 滤波 |
| 电机驱动 | TB6612 双通道，20 kHz PWM，占空比限幅 45% |
| 机械 | 一级旋转倒立摆，L 型摆臂，齿隙约 3.5°（约 10 count） |
| 套件 | 武汉易思达科技有限公司 J280 姿态控制系统竞赛套件 |

详细引脚与接口见 [docs/hardware.md](docs/hardware.md)。

## 仓库结构

```text
rtl/                 Verilog 源码、约束 .cst、时序 .sdc
  j280_swing/        主线：j280_swing_hyb_top、观测/控制、起摆、轨迹、守护、键盘
  j280_balance/      平衡固件与编码器、PWM 等共享模块
  j280_state/        ADC/编码器原始量估计与遥测
  j280_measure/      早期测量固件（研发记录）
  j280_motor_test/   早期电机测试固件（研发记录）
sim/                 Icarus Verilog 测试台
scripts/             构建、烧录、上位机、定点模型、轨迹/配置工具
tests/               Python + RTL 回归测试
configs/             比赛主配置 obs-973.json
firmware/            最终固件 j280_swing_973_keypad.fs
docs/                硬件接口说明、测试所需的最小数据样本
```

主线固件由 `j280_swing_hyb_top` 构建；`j280_swing_lean_top`、`j280_swing_pip_top`、`j280_swing_phase_top`、`j280_swing_sc_top` 等为研发过程中的实验版本，保留作记录，不作为构建入口。

## 快速开始

### 环境

- Gowin V1.9.12.03（默认安装路径；可用环境变量 `J280_GOWIN` 指定 `gw_sh.exe`）
- Python 3.10+（脚本使用标准库 + numpy/scipy；上位机另需 pyserial）
- RTL 测试需要 Icarus Verilog（`iverilog` / `vvp`）

> gw_sh 对多级中文路径敏感，建议将仓库放在不含中文/空格的路径下构建。

### 构建

```bash
python scripts/j280_build_gear.py
```

输出 `build/gear/impl/pnr/j280_swing.fs`，同时打印 Logic/Register/DSP 与时序余量。

### 烧录（SRAM）

```powershell
powershell -ExecutionPolicy Bypass -File scripts/j280_program.ps1 -Fs firmware/j280_swing_973_keypad.fs
```

固件 SHA256：

```text
6859532167c62b26ffe475295af8b37ccdca31e0c6a551d51b5ebc8e27bb2122  j280_swing_973_keypad.fs
```

### 上位机

```bash
python scripts/j280_panel.py
```

选择实际串口（如 COM8），载入 `configs/obs-973.json`，拨 DIP1 到 ON（ARMED），按 SW1 自动起摆，SW2 停止。

### 按键与键盘

- DIP1 ON：ARMED；SW1 起摆/接管；SW2 停止。
- SW3/SW4：相对当前参考 ±90° 步进。
- DIP2 ON：连续旋转；短按 SW3/SW4 换向，长按升/降档。
- 4×4 键盘：数字输入 0–360°，`A`/`B`/`C` = 30/60/90°/s，`*` 逆时针，`#` 顺时针，`D` 提交；长按数字 0.5 s 从该数字重新输入。

## 算法概览

- **状态观测**：四状态 Kalman（摆臂角度/角速度、摆杆角度/角速度）。摆杆角速度与 20 Hz 状态变量滤波器（SVF）混合，摆臂速度经 2 Hz 低通。
- **扰动观测器**：由速度差分与模型加速度估计集总扰动，补偿摩擦、齿隙与模型失配。
- **控制律**：LQR 型状态反馈，含摩擦补偿、前馈、轨迹积分与限幅。
- **起摆**：能量泵能与掉头接杆策略，接住后切平衡。
- **轨迹**：90° 步进、键盘定位、连续旋转；梯形加减速、积分、lean 参考与网格停位。
- **守护**：行程窗口、角度/速度限幅、采样超时、故障锁存与状态机。

主配置 `configs/obs-973.json` 使用算法 19：Kalman + DOB + LQR，摆杆角速度与 SVF 混合，摆臂速度额外 2 Hz 低通。

## 实测指标

| 指标 | 结果 |
| --- | --- |
| 控制周期 | 500 Hz（2 ms），50 MHz 时钟 |
| 自动起摆 | 约 1.5–2 s |
| 稳态摆杆波动 | 约 1.0–1.5°（std） |
| 摆臂稳态波动 | 有利相位 4–6 count，不利相位 9–15 count |
| 抗扰恢复 | 约 20° 推扰后约 3 s |
| 位置控制 | 90° 步进、键盘 0–360°、30/60/90°/s |
| 连续旋转 | 0.5× / 1.0× / 1.5×，顺/逆时针 |
| FPGA 时序 | Setup 余量约 +0.30 ns |
| 资源占用 | Logic 47%，Register 45%，DSP 18.75/20 |

## 测试

```bash
python -m unittest discover -s tests -p "test_j280_*.py"
```

- `*_rtl.py` 与集成测试会调用 Icarus Verilog；默认查找 `%LOCALAPPDATA%\J280Tools\iverilog-msys2\ucrt64\bin` 或 `PATH`，也可通过环境变量 `J280_IVERILOG_BIN` 指定 `bin` 目录。
- 纯 Python 测试无需 iverilog。
- 部分集成测试为完整 RTL 仿真，耗时数分钟。

## 已知限制

- 齿轮齿隙约 3.5°（10 count），摆臂在部分相位仍有 9–15 count 的低速粘滑极限环；有利相位 4–6 count。
- 控制算法全部在 FPGA 内实现，参数需通过上位机配置载入。
- 未包含官方套件资料、Gowin 工具 license 与开发过程数据；固件可直接烧录，也可以从源码重新构建。

## 许可证与致谢

本项目采用 [MIT License](LICENSE)。

感谢武汉易思达科技有限公司提供的 J280 姿态控制系统竞赛套件与用户手册。官方套件资料、示例工程及 Gowin 工具授权文件不包含在本仓库中。
