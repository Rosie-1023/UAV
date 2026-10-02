# 无人机硬件组成报告

**平台**：Lidar300 蜂群无人机
**撰写日期**：2026-10-02
**文档版本**：v3.0

> **变更记录**
> - v1.0：系统总览与五大硬件模块组成、接口与参数。
> - v2.0：新增第 8 章动力系统性能计算（推力/功率/悬停时间）、第 9 章电源链路分析与供电校验。
> - v3.0：新增第 10 章通信链路带宽与延迟量化分析、第 11 章可靠性设计（链路冗余、失效降级、EMC），并给出第 12 章设计结论。

---

## 1. 系统总览

本平台采用典型的"感知—决策—控制—执行"分层硬件架构，由传感器、处理器、动力组件、遥控模块与电源模块五大模块构成。

```
传感器 ──(以太网/USB/UART)──▶ 处理器 ──(UART·MAVLink)──▶ 飞控 ──(DShot600)──▶ 电调 ──(三相交流)──▶ 电机 ──▶ 桨叶
```

分层结构如下：

| 层次 | 硬件 | 职责 |
|------|------|------|
| 感知层 | 激光雷达、相机、光流 | 采集环境与自身运动状态 |
| 决策层 | NVIDIA Orin NX | 定位建图、路径规划、任务调度 |
| 控制层 | 微空 NXT PX4 飞控 | 姿态/位置闭环控制、模式管理 |
| 执行层 | 电调、电机、桨叶 | 将控制量转化为推力 |
| 交互层 | 遥控器、接收机 | 人工介入与应急接管 |
| 能源层 | 锂电池、分电板 | 全系统供电与电压轨分配 |

## 2. 传感器模块

| 器件 | 接口 | 用途 |
|------|------|------|
| Livox Mid360 激光雷达 | 以太网 | 三维点云采集、SLAM 建图与避障 |
| RGB-90 前视相机 | USB | 前方环境视觉感知 |
| RGB-120 下视相机 | USB | 下方视觉定位与目标识别 |
| 优象光流 | UART 串口桥 + MAVLink | 低空速度测量与悬停增稳 |

激光雷达点云数据量按单帧点数 $N$ 与帧率 $f$ 估算：

$$
R_{\text{lidar}} = N \cdot f \cdot b
$$

其中 $b$ 为单返回点字节数，Mid360 典型每点含三维坐标、反射率与时间戳。

## 3. 处理器模块

- **NVIDIA Orin NX 机载电脑**：承担 SLAM、视觉推理与任务规划等算力密集任务；
- **微空 NXT PX4 飞控**：运行 PX4 固件，负责高频姿态闭环与安全保护。

二者通过 **UART + MAVLink** 双向通信：上行下发期望位姿/速度指令，下行回传 IMU、电池与姿态遥测，构成闭环。

## 4. 动力组件

| 器件 | 参数/协议 |
|------|-----------|
| 燕岛 50A 电调 | 输入直流，输出三相交流；控制协议 DShot600 |
| F90 电机 | KV1300，无刷三相 |
| GF 7037 桨叶 | 三叶竞速桨，直径 7 英寸 |

电机空载转速与电压关系为

$$
n_{\text{no-load}} = K_V \cdot U
$$

## 5. 遥控模块

- **AT9S Pro 遥控器**（地面）与 **R12DSM 接收机**（机载）之间为 2.4 GHz 无线链路；
- 接收机与飞控之间为 **SBUS** 串行协议，用于人工操控与模式切换。

## 6. 电源模块

- **21700-6s1p 锂电池**：6 节串联，标称电压 $U_{\text{nom}} = 6 \times 3.7\ \text{V} = 22.2\ \text{V}$，满电约 $25.2\ \text{V}$；
- **分电板**：向动力与处理单元分配 24 V，经 DC-DC 降压输出 12 V 为传感器供电，实现噪声隔离。

## 7. 小结

本平台硬件选型兼顾了带宽（以太网/USB）、实时性（DShot600/SBUS）与标准化互联（MAVLink）。后续版本将补充动力系统性能计算、电源链路分析与通信链路定量评估。

---

## 8. 动力系统性能计算

### 8.1 桨叶气动推力模型

旋翼产生的推力可表示为

$$
T = C_T \cdot \rho \cdot n^2 \cdot D^4
$$

其中 $C_T$ 为推力系数，$\rho = 1.225\ \mathrm{kg/m^3}$ 为海平面空气密度，$n$ 为桨叶转速（r/s），$D$ 为桨直径（m）。GF 7037 桨标称直径 7 英寸，即

$$
D = 7 \times 0.0254\ \mathrm{m} = 0.1778\ \mathrm{m}
$$

所需扭矩与功率为

$$
Q = C_P \cdot \rho \cdot n^2 \cdot D^5, \qquad P = 2\pi n Q
$$

### 8.2 电机转速估算

F90 电机 $K_V = 1300$，在有效电压 $U_{\text{eff}}$（扣除电调压降与反电动势影响后的等效值）下空载转速为

$$
n_0 = K_V \cdot U_{\text{eff}}
$$

以 6s 电池标称电压 $U_{\text{nom}} = 22.2\ \mathrm{V}$ 计：

$$
n_0 = 1300 \times 22.2 \approx 2.89 \times 10^4\ \mathrm{rpm} \approx 481\ \mathrm{r/s}
$$

实际带载后转速下降，通常取负载系数 $\eta_n \approx 0.75$：

$$
n = \eta_n \cdot n_0 \approx 361\ \mathrm{r/s}
$$

该转速已接近 7 英寸桨的实用上限，需在飞控中设置油门限幅以避免超速。

### 8.3 悬停功率与续航估算

设整机起飞质量 $m$，四轴布局下单轴所需悬停推力为

$$
T_{\text{hover}} = \frac{m g}{4}
$$

整机悬停气动功率（理想动量理论）为

$$
P_{\text{ideal}} = \frac{(m g)^{3/2}}{\sqrt{2 \rho A_{\text{total}}}}, \qquad A_{\text{total}} = 4 \cdot \frac{\pi D^2}{4} = \pi D^2
$$

考虑电机效率 $\eta_m$、电调效率 $\eta_e$ 与桨叶效率 $\eta_p$，实际电功率为

$$
P_{\text{elec}} = \frac{P_{\text{ideal}}}{\eta_m \eta_e \eta_p}
$$

以 $m = 2.4\ \mathrm{kg}$、$\eta_m = 0.85$、$\eta_e = 0.95$、$\eta_p = 0.7$ 为例：

$$
P_{\text{ideal}} = \frac{(2.4 \times 9.81)^{1.5}}{\sqrt{2 \times 1.225 \times \pi \times 0.1778^2}} \approx 233\ \mathrm{W}
$$

$$
P_{\text{elec}} \approx \frac{233}{0.85 \times 0.95 \times 0.7} \approx 412\ \mathrm{W}
$$

对应母线电流

$$
I_{\text{bus}} = \frac{P_{\text{elec}}}{U_{\text{nom}}} = \frac{412}{22.2} \approx 18.6\ \mathrm{A}
$$

四轴分摊后单机电流约 $4.6\ \mathrm{A}$，远低于燕岛 50A 电调额定值，留有充足的瞬时过载余量。

### 8.4 续航时间估算

21700 电芯典型容量 $C = 5.0\ \mathrm{Ah}$，6s1p 无并联，故总容量仍为 $5.0\ \mathrm{Ah}$，可用放电深度取 $\mathrm{DoD} = 0.8$：

$$
t_{\text{endurance}} = \frac{C \cdot \mathrm{DoD}}{I_{\text{bus}}} = \frac{5.0 \times 0.8}{18.6}\ \mathrm{h} \approx 0.215\ \mathrm{h} \approx 12.9\ \mathrm{min}
$$

扣除机载电脑（Orin NX 典型 $15\sim25\ \mathrm{W}$）与传感器（约 $10\ \mathrm{W}$）消耗后，实际可用任务时间约 $11\sim12\ \mathrm{min}$。

## 9. 电源链路分析

### 9.1 电压轨分配

```
21700-6s1p (22.2 V nominal / 25.2 V full)
        │
        ▼
     分电板
        ├── 24 V 母线 ──▶ 电调/电机、Orin NX（经模块内二次降压）
        └── DC-DC 降压 12 V ──▶ Mid360、RGB-90、RGB-120、光流
```

### 9.2 线损与压降校验

供电线缆压降为

$$
\Delta U = I \cdot R_{\text{wire}} = I \cdot \frac{2 \rho_{\text{Cu}} L}{S}
$$

其中铜电阻率 $\rho_{\text{Cu}} = 1.72 \times 10^{-8}\ \Omega\cdot\mathrm{m}$，$L$ 为单程线长，$S$ 为线截面积，$2L$ 计入回路双线。取 $L = 0.3\ \mathrm{m}$、$S = 2.5\ \mathrm{mm^2}$、$I = 18.6\ \mathrm{A}$：

$$
\Delta U = 18.6 \times \frac{2 \times 1.72 \times 10^{-8} \times 0.3}{2.5 \times 10^{-6}} \approx 0.077\ \mathrm{V}
$$

压降占比仅 $\frac{0.077}{22.2} \approx 0.35\%$，线径选型合理。

### 9.3 传感器供电隔离

动力回路在加减速时会产生大幅电流纹波，若与传感器共用同一电压轨，将通过共阻抗耦合引入噪声：

$$
U_{\text{noise}} = \frac{\mathrm{d}i}{\mathrm{d}t} \cdot L_{\text{loop}} + i \cdot R_{\text{common}}
$$

本设计通过独立 12 V DC-DC 支路为传感器供电，并在传感器侧并联去耦电容 $C_{\text{dec}}$，其纹波抑制比为

$$
\left| \frac{U_{\text{out}}}{U_{\text{in}}} \right| \approx \frac{1}{\omega^2 L_f C_{\text{dec}}}
$$

从而实现动力与感知的电源域隔离，避免点云与图像在低电压/高噪声条件下出现异常。

---

## 10. 通信链路带宽与延迟量化分析

### 10.1 链路清单与速率

| 链路 | 物理接口 | 协议 | 典型速率 | 数据特征 |
|------|----------|------|----------|----------|
| Mid360 → Orin NX | 以太网 | Livox SDK / UDP | 100/1000 Mbps | 点云，约 200,000 点/秒 |
| RGB-90 → Orin NX | USB 3.0 | UVC | ≤ 5 Gbps | 1080p@30fps 未压缩约 1.5 Gbps，实际 MJPEG 压缩后 15~50 Mbps |
| RGB-120 → Orin NX | USB 3.0 | UVC | ≤ 5 Gbps | 同上 |
| 光流 → PX4 | UART | MAVLink | 115200 bps | 小包、高频（50~100 Hz） |
| Orin NX ↔ PX4 | UART | MAVLink | 921600 bps | 中小包、双向 |
| PX4 → 电调 | 单线数字 | DShot600 | 600 kbit/s | 16 bit/帧，确定性延迟 |
| 接收机 → PX4 | 反相串口 | SBUS | 100 kbit/s | 25 字节/帧，7 ms 周期 |

### 10.2 总线占用率计算

UART 链路上单帧传输时间为

$$
t_{\text{frame}} = \frac{N_{\text{bit}}}{B}
$$

其中 $N_{\text{bit}}$ 为一帧总位数（含起始位、停止位），$B$ 为波特率。以 MAVLink `ATTITUDE` 消息（28 字节载荷 + 8 字节帧头 + 2 字节校验 = 38 字节）为例，UART 8N1 下每字节 10 bit：

$$
N_{\text{bit}} = 38 \times 10 = 380\ \text{bit}
$$

在 $B = 921600\ \mathrm{bps}$ 下：

$$
t_{\text{frame}} = \frac{380}{921600} \approx 0.41\ \mathrm{ms}
$$

若姿态消息以 $f = 100\ \mathrm{Hz}$ 下发，则总线占用率为

$$
\eta_{\text{bus}} = t_{\text{frame}} \cdot f = 0.41 \times 10^{-3} \times 100 = 4.1\%
$$

说明 921600 bps 下 MAVLink 上行/下行仍有充裕余量，可同时承载里程计、目标点、心跳等多条消息流。

### 10.3 DShot600 时序分析

DShot 每帧 16 bit（11 bit 油门 + 1 bit 遥测请求 + 4 bit CRC），位周期为

$$
T_{\text{bit}} = \frac{1}{600 \times 10^3} \approx 1.67\ \mu\mathrm{s}
$$

整帧时长

$$
T_{\text{frame}} = 16 \times T_{\text{bit}} \approx 26.7\ \mu\mathrm{s}
$$

若飞控控制环以 $f_{\text{ctrl}} = 400\ \mathrm{Hz}$ 运行，则每个控制周期有

$$
\frac{1}{f_{\text{ctrl}}} = 2.5\ \mathrm{ms} \gg 26.7\ \mu\mathrm{s}
$$

即电调指令延迟占控制周期比例不足 $1.1\%$，相比传统 PWM（1000~2000 μs 脉宽，周期 2.5 ms）具有明显实时性优势，且数字编码避免了模拟脉宽的温漂与抖动。

### 10.4 端到端延迟预算

从传感器采样到执行器响应的端到端延迟可分解为

$$
\tau_{\text{total}} = \tau_{\text{sense}} + \tau_{\text{trans}} + \tau_{\text{compute}} + \tau_{\text{cmd}} + \tau_{\text{act}}
$$

各项典型值：

| 环节 | 符号 | 典型值 |
|------|------|--------|
| 传感器采样与曝光 | $\tau_{\text{sense}}$ | 5~10 ms |
| 数据传输 | $\tau_{\text{trans}}$ | 1~3 ms（以太网/USB） |
| 机载电脑感知与规划 | $\tau_{\text{compute}}$ | 20~50 ms |
| MAVLink 指令下发 | $\tau_{\text{cmd}}$ | 0.4~2 ms |
| 飞控控制环与电调响应 | $\tau_{\text{act}}$ | 2.5~5 ms |

合计

$$
\tau_{\text{total}} \approx 29 \sim 70\ \mathrm{ms}
$$

该量级对 1-3 m/s 飞行速度下的避障任务可接受（对应位移 3-21 cm），但需要在规划层引入状态外推补偿：

$$
\hat{\boldsymbol{p}}(t + \tau) = \boldsymbol{p}(t) + \boldsymbol{v}(t)\tau + \frac{1}{2}\boldsymbol{a}(t)\tau^2
$$

以抵消延迟带来的位姿滞后。

## 11. 可靠性设计

### 11.1 控制链路冗余

飞控同时接收两路指令源，按优先级仲裁：

$$
\boldsymbol{u}_{\text{out}} = 
\begin{cases}
\boldsymbol{u}_{\text{RC}}, & \text{手动模式或 RC 触发} \
\boldsymbol{u}_{\text{companion}}, & \text{自动模式且链路健康} \
\boldsymbol{u}_{\text{failsafe}}, & \text{链路超时}
\end{cases}
$$

其中链路健康判据为心跳超时

$$
t_{\text{now}} - t_{\text{hb}} > \tau_{\text{timeout}}
$$

典型取 $\tau_{\text{timeout}} = 2\ \mathrm{s}$，超时后触发失效保护。

### 11.2 传感器冗余

惯导（飞控 IMU）与光流/激光雷达里程计构成互补：IMU 高频但存在漂移，外部里程计低频但无累积误差。二者的误差特性为

$$
\sigma_{\text{IMU}}(t) \propto \sqrt{t}, \qquad \sigma_{\text{odom}}(t) \approx \text{const}
$$

通过 EKF 融合可使长期位姿误差收敛到外部里程计量级，避免纯 IMU 积分发散。

### 11.3 电磁兼容（EMC）

- 电调输出到电机的三相大电流回路与信号线**分层走线**，避免平行长距离布线；
- 电感耦合干扰电压满足

$$
U_{\text{ind}} = M \cdot \frac{\mathrm{d}i}{\mathrm{d}t}
$$

  通过减小互感 $M$（拉开间距、双绞、垂直交叉）抑制；
- GNSS/接收机天线远离电调与电源线，2.4 GHz 接收机链路保留 $20\ \mathrm{dB}$ 以上链路余量：

$$
\text{Margin} = P_{\text{rx}} - P_{\text{sensitivity}}
$$

### 11.4 电源失效保护

电池电压低于阈值时触发分级保护：

$$
U_{\text{cell}} < U_{\text{warn}} \Rightarrow \text{告警}; \qquad U_{\text{cell}} < U_{\text{crit}} \Rightarrow \text{强制返航/降落}
$$

6s 电池对应 $U_{\text{warn}} \approx 3.5\ \mathrm{V/cell}$（21 V）、$U_{\text{crit}} \approx 3.3\ \mathrm{V/cell}$（19.8 V）。

## 12. 设计结论

1. **带宽分层合理**：点云/图像走以太网与 USB，控制与状态走 UART/MAVLink，实时执行走 DShot600，各链路占用率均留有 2 倍以上余量。
2. **实时性满足**：端到端延迟 $29\sim70\ \mathrm{ms}$，配合状态外推可满足中低速自主飞行需求。
3. **动力余量充足**：悬停母线电流约 18.6 A，电调 50 A 额定值留有 2.7 倍峰值余量；续航约 11~12 min，可通过提高电池容量或降低起飞质量改善。
4. **电源与 EMC 设计到位**：12 V/24 V 双轨隔离、去耦与分层布线有效隔离动力噪声。
5. **改进方向**：若后续需更高算力或更长续航，建议评估 6s2p 电池方案与 8 英寸桨低转速效率区间。
