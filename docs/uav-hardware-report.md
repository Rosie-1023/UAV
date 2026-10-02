# 无人机硬件组成报告

**平台**：Lidar300 蜂群无人机
**撰写日期**：2026-10-02
**文档版本**：v2.0

> **变更记录**
> - v1.0：系统总览与五大硬件模块组成、接口与参数。
> - v2.0：新增第 8 章动力系统性能计算（推力/功率/悬停时间）、第 9 章电源链路分析与供电校验。

---

## 1. 系统总览

本平台采用典型的"感知—决策—控制—执行"分层硬件架构，由传感器、处理器、动力组件、遥控模块与电源模块五大模块构成。

$$
\text{传感器} \xrightarrow{\text{以太网 / USB / UART}} \text{处理器} \xrightarrow{\text{UART(MAVLink)}} \text{飞控} \xrightarrow{\text{DShot600}} \text{电调} \xrightarrow{\text{三相交流}} \text{电机} \rightarrow \text{桨叶}
$$

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

- **21700-6s1p 锂电池**：6 节串联，标称电压 $U_{\text{nom}} = 6 \times 3.7\,\text{V} = 22.2\,\text{V}$，满电约 $25.2\,\text{V}$；
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

其中 $C_T$ 为推力系数，$\rho = 1.225\,\mathrm{kg/m^3}$ 为海平面空气密度，$n$ 为桨叶转速（r/s），$D$ 为桨直径（m）。GF 7037 桨标称直径 7 英寸，即

$$
D = 7 \times 0.0254\,\mathrm{m} = 0.1778\,\mathrm{m}
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

以 6s 电池标称电压 $U_{\text{nom}} = 22.2\,\mathrm{V}$ 计：

$$
n_0 = 1300 \times 22.2 \approx 2.89 \times 10^4\,\mathrm{rpm} \approx 481\,\mathrm{r/s}
$$

实际带载后转速下降，通常取负载系数 $\eta_n \approx 0.75$：

$$
n = \eta_n \cdot n_0 \approx 361\,\mathrm{r/s}
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

以 $m = 2.4\,\mathrm{kg}$、$\eta_m = 0.85$、$\eta_e = 0.95$、$\eta_p = 0.7$ 为例：

$$
P_{\text{ideal}} = \frac{(2.4 \times 9.81)^{1.5}}{\sqrt{2 \times 1.225 \times \pi \times 0.1778^2}} \approx 233\,\mathrm{W}
$$

$$
P_{\text{elec}} \approx \frac{233}{0.85 \times 0.95 \times 0.7} \approx 412\,\mathrm{W}
$$

对应母线电流

$$
I_{\text{bus}} = \frac{P_{\text{elec}}}{U_{\text{nom}}} = \frac{412}{22.2} \approx 18.6\,\mathrm{A}
$$

四轴分摊后单机电流约 $4.6\,\mathrm{A}$，远低于燕岛 50A 电调额定值，留有充足的瞬时过载余量。

### 8.4 续航时间估算

21700 电芯典型容量 $C = 5.0\,\mathrm{Ah}$，6s1p 无并联，故总容量仍为 $5.0\,\mathrm{Ah}$，可用放电深度取 $\mathrm{DoD} = 0.8$：

$$
t_{\text{endurance}} = \frac{C \cdot \mathrm{DoD}}{I_{\text{bus}}} = \frac{5.0 \times 0.8}{18.6}\,\mathrm{h} \approx 0.215\,\mathrm{h} \approx 12.9\,\mathrm{min}
$$

扣除机载电脑（Orin NX 典型 $15\sim25\,\mathrm{W}$）与传感器（约 $10\,\mathrm{W}$）消耗后，实际可用任务时间约 $11\sim12\,\mathrm{min}$。

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

其中铜电阻率 $\rho_{\text{Cu}} = 1.72 \times 10^{-8}\,\Omega\cdot\mathrm{m}$，$L$ 为单程线长，$S$ 为线截面积，$2L$ 计入回路双线。取 $L = 0.3\,\mathrm{m}$、$S = 2.5\,\mathrm{mm^2}$、$I = 18.6\,\mathrm{A}$：

$$
\Delta U = 18.6 \times \frac{2 \times 1.72 \times 10^{-8} \times 0.3}{2.5 \times 10^{-6}} \approx 0.077\,\mathrm{V}
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
