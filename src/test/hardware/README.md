# Custom-link 真机联调 CLI

[custom_link_hardware.cli](custom_link_hardware.cli) 从当前仿真测试提取了可迁移的链路基线，
并把需要硬件信息的命令保留为**注释示例**。它不是开箱即飞、自动拦截或已验证的返航配置。
尚未提供飞控型号、接线、接收机协议、GPS/罗盘型号、动力实测和场地净空，
因此不能安全地替你填写这些项目。不要将 SITL EEPROM 导入真机。

## 1. 实际执行的三个设置

| 参数 | 设置 | 依据与限制 |
| --- | --- | --- |
| `custom_link_watchdog_ms` | `50` | 当前固件默认；bridge 的 `100 Hz` 控制流名义周期为 `10 ms`。真机串口延迟、抖动和丢帧必须实测；watchdog 超时只使主机控制失去新鲜度，不等于自动返航或自动安全落地。 |
| `custom_link_motors_stream` | `OFF` | 真机不需要 AirSim 的电机输出下传，减少额外带宽。不会关闭物理电机输出，也不会阻止解锁。 |
| `gps_rescue_allow_arming_without_fix` | `OFF` | 保留 GPS Rescue 的定位约束；不能替代人工确认新鲜 3D fix、home、卫星数和定位质量。并非所有手动模式都保证因缺少 GPS 而禁止解锁。 |

其余活动命令只读取状态。脚本**不会执行 `save`**，不会重置整机配置，
也不会改动现有 PID、滤波、接收机、串口、AUX 模式、失控策略或返航高度。
如果已有链路的延迟要求与 `50 ms` 不兼容，先停止配置并检查通信，而不是盲目放宽保护。

## 2. 固件与串口前提

- 必须使用本分支、针对实际板卡编译且包含 `USE_CUSTOM_LINK` 的固件。
  普通官方 Betaflight 不保证存在这些参数。`set` 返回未知参数时不能继续照抄。
- 要使用 GPS、ALTHOLD、POSHOLD 和 Rescue，板卡构建还必须支持对应传感器与功能。
  启用 CLI 参数不能补上未编译进固件的能力。
- Custom link 独占一个 UART：功能掩码为 `2097152`（`1 << 21`）。
  波特率读取 **telemetry baud** 字段；`AUTO` 时固件默认 `921600`。
  示例 `serial UART3 2097152 115200 57600 921600 115200` 只有确认 UART3 接伴飞机时才可取消注释。
- GPS 需要另一个实际串口；示例 UART4/UBLOX/57600 只是语法示例，不是你的接线结论。
  接收机 UART、串行 RX 协议、反相/半双工、资源分配和供电均保留原机值。
- companion TX 接 FC RX、companion RX 接 FC TX、共地，并确认电平兼容。
  `link.serial_port` 是伴飞机上的设备名，不能据此反推 FC 的 UART 编号。

## 3. 遥控通道和 bridge 必须匹配

以下是仿真使用的布局，**仅当真机遥控器也输出该布局时**才可采用 CLI 的注释示例。
先检查实际通道输出及全部现存 `aux` 规则，尤其是 PREARM、ARM 和 OFFBOARD；
不能只覆盖前几个槽后忽略其它槽内的旧规则。

| 物理通道 | 用途 | 数值/规则 |
| --- | --- | --- |
| CH1..4 | Roll / Pitch / Throttle / Yaw | 实际接收顺序确认后才使用 `map AETR1234`。 |
| CH5 / AUX1 | 六段模式 | 1000 ACRO；1200 ANGLE；1400 HORIZON；1600 ALTHOLD；1800 POSHOLD+ALTHOLD；2000 GPS Rescue。 |
| CH6 / AUX2 | 仿真中的手动 OFFBOARD 开关 | 常态保持低；不需要它来启动任务。已有 ID 58 的高电平规则可能持续授权 OFFBOARD，必须核对。 |
| CH7 / AUX3 | ARM | 高于 1700；拆桨确认起桨/停桨边沿由 bridge 产生 tracker ON/OFF。 |
| CH8 / AUX4 | bridge 触发键 | 高于 1900 超过 0.5 s；低于 1100 复位锁存。锁定且空闲时启动，运行中再次长按为 DISMISS。**不是飞控 AUX 模式规则**。 |

**当前仓库里的真机 bridge 配置和仿真不一致：**
`apm_bridge/parameters/custom_link_parameters.yaml` 的 `channel_trigger` 为 `9`，
即 **CH10/AUX6**；仿真为 `7`，即 **CH8/AUX4**。
如保留上述通道布局，应把真机 bridge 的该参数改为 `7`；
如保留真机的 CH10 布局，则必须把实际触发键放在 CH10。FC CLI 无法修改 ROS 参数。

MISSION/DISMISS 仍由 bridge 的 RC 逻辑生成，并联动主机模式请求；
手动接管、GPS Rescue 和主机请求的优先级由飞控仲裁。
不要通过额外的 OFFBOARD 常高开关绕过这条验证路径。
AirSim 特有的“未解锁 AUX4 视觉预览”不适用于硬件配置。

## 4. 必须留给真机验证的设置

| 项目 | 真机处理 |
| --- | --- |
| Rate PID / feedforward / 滤波 | 保留已完成手动飞行验证的机体调参。不要复制仿真的 roll `37/62/22`、pitch `32/57/22` 或 yaw `10/0/0`。 |
| 悬停与推力 | `ap_hover_throttle`、油门范围以及 bridge 的机重、电池、推力模型须来自本机测量；`1275`、仿真机重和仿真电机曲线不是硬件结果。 |
| 位置/高度环 | 仿真专用 `ap_position_p=6`、`ap_velocity_drag_coeff=0`、`ap_max_angle=25`、`ap_nav_accel=100` 等不自动下发。真实 GPS 延迟、振动、重心和风况不同。 |
| GPS | 不得使用 `gps_provider=VIRTUAL`。按真实模块设置 provider、串口及更新率，确认 home 在解锁时正确建立。 |
| 罗盘/姿态 | 不复制 `mag_declination=20`、`trust_mag=ON` 或传感器偏置。按实际安装、当地磁偏角和室外校准检查磁干扰与朝向。 |
| 解锁保护 | 不复制仿真 `small_angle=65` 或 `pwr_on_arm_grace=5`；保持已审核的正常解锁保护和 PREARM。 |
| 返航 | `gps_rescue_return_alt`、`initial_climb`、速度、下降距离和着陆率必须根据地形/障碍及真机试飞确定。当前分支默认返回高度为 **8 m**，不能因“未改动”便认为适合户外。 |
| 失控策略 | 不自动覆盖 `failsafe_procedure`。先区分 RC 接收机失联、伴飞机断流、UART 断线与电源故障，不把它们当同一种情况。 |
| 主机控制限幅 | 查看 `custom_link_rate_limit_dps`、`custom_link_angle_limit_deg`；`0` 表示不限。角度限幅仅切除越界后继续增大倾角方向的角速度命令，**不会主动回平，不是完整姿态保护/地理围栏**。根据真机评审选择值，不从仿真推导认证值。 |

当前构建若启用 `ENABLE_RESCUE_PLAN`，Rescue 由统一位置/高度控制器执行；
没有此构建能力时可能走旧 Rescue 控制器。不能只比较 `gps_rescue_*` 数值就断言两者等价；
例如旧控制器的 sanity 配置不能代替任务执行器自身的退出条件。

bridge 还需确认 `airsim.enabled=false`、`link.transport=serial`、
`link.baudrate=921600`、`cli.bind_rx_on_connect=false`。
不要直接复用真机 YAML 中现成的机重/推力拟合；它们是另一套具体硬件的数据。
相机安装角、云台限位和传感器坐标也不是 FC CLI 能解决的项目。

## 5. 拆桨执行顺序

1. 停止伴飞机任务/控制流，拆除全部桨叶，禁止自动解锁或自动任务启动。
2. 通过本机 USB/可靠 CLI 连接执行 `version`、`status`、`dump all`、`diff all` 并保存输出。
3. 阅读脚本。先只执行未注释的基线；不要一次性取消所有示例注释。
4. 根据接线配置独立 UART、实际 GPS 和接收机；核对输出，不应出现 `###ERROR`。
5. 逐通道确认开关、模式及接管行为。AUX 模式示例须在保留其它安全规则的前提下手动合并。
6. 查看脚本末尾读回值。确认实际 PID/滤波、failsafe、返航高度及机体参数没有被仿真值替换。
7. 审核通过后，才手动输入 `save`；重启后重新读回并与备份比较。
8. 先完成拆桨链路/超时/接收机失联验证，再独立开展手动模式、位置/高度保持和返航验证；
   这些阶段的实测不能由 SITL 通过结果替代。未验证前不放行自主任务。

CLI 语法和数值读回可用 SITL 克隆 EEPROM 验证，但该验证不证明实际 UART、
板卡功能、遥控链路、GPS、动力或真机飞行安全。本文件未连接、修改或刷写实际飞控。

2026-10-22 已用本仓库 SITL 的 `--config` 对克隆 EEPROM 验证：活动命令及注释中的
UART/GPS/RC 示例均无 CLI 错误，三个设置读回一致。基线执行前后的 `dump all` 对比确认，
除上述三个基线键之外的设置、串口、通道映射及 AUX 规则不变。
`--config` 会自动保存克隆 EEPROM；这不代表脚本本身包含真机自动 `save`。
