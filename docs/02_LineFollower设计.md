# LineFollower 网络设计

## 1. 职责

LineFollower 是上层任务模块：根据灰度观测和期望基础速度决定小车希望的线速度与角速度。它不读取编码器、PWM、电机参数或驱动器状态，也不负责执行器闭环。

```text
GrayObservation + DesiredSpeed -> LineFollower -> TargetTwist(v, omega)
                                      -> DifferentialDriveMixer
                                      -> TargetWheelSpeed
```

## 2. 灰度位置协议

第一版使用最多 16 个固定长度数组元素；跨车辆版本不再把数组下标解释为标准空间槽位，而是按实际物理位置从左到右排序：

```text
sensor[0] ... sensor[N-1]  # 从左到右
sensor[N] ... sensor[15]   # padding，exist=0
```

数组下标只表示存储顺序，不表示传感器空间位置。6/8/10/12/16 路以及非均匀阵列都可以直接使用；padding 元素为 `exist=0, valid=0`。硬件适配层负责把 ADC/channel 顺序重排为从左到右，禁止使用 nearest-slot 映射。

每个元素有独立的物理位置 `sensor_position_m[16]`；`sensor_to_axle_m` 记录灰度阵列到控制参考轴的纵向距离。`track_width_m` 不进入网络，属于后面的 DifferentialDriveMixer/vehicle profile。

单车快速调试可以使用固定槽位简化输入，但该模型不宣称跨车辆泛化。

## 3. 模块接口

```text
LineFollowerInput {
    gray_value[16] : float32
    gray_exist[16] : uint8
    gray_valid[16] : uint8
    sensor_position_norm[16] : float32
    sensor_to_axle_norm : float32
    desired_linear_speed_mps : float32
    timestamp_us   : uint64
    dt_s           : float32
}
```

跨车辆版本输入张量固定为 `float32[66]`：

```text
x_line = [
  gray_value[0:16], gray_exist[0:16], gray_valid[0:16],
  sensor_position_norm[0:16], sensor_to_axle_norm, desired_speed_norm
]
```

输出张量为 `float32[2]`：

```text
y_line = [target_linear_speed_mps, target_angular_speed_radps]
```

几何输入由公共 scaler 转换：

```text
sensor_position_norm = clip(sensor_position_m / position_ref_m, -1, 1)
sensor_to_axle_norm  = clip(sensor_to_axle_m / axle_ref_m, -1, 1)
```

其中 `desired_speed_norm = desired_linear_speed_mps / speed_ref_mps`。网络输出不是自由预测的线速度，而是：

```text
s_v       ∈ [0, 1]
omega_norm ∈ [-1, 1]
```

最终确定性组合为：

```text
v_target     = s_v * desired_linear_speed_mps
omega_target = omega_norm * omega_ref_radps
```

因此网络可以在弯道减速，但不能超过驾驶/任务层的 `desired_linear_speed_mps`。第一阶段规定 `desired_linear_speed_mps >= 0`；若 desired speed 为 0，wrapper 直接输出 `v_target=0, omega_target=0`，不调用网络猜测停止行为。

单车 MVP 可以暂时使用 `gray + exist + valid + desired_speed` 的 49 维输入；其 MLP 只用于接口和训练流程验证，不作为跨车辆模型。

统一生命周期：

```text
reset(reason)
step(input, dt_s) -> TargetTwist
get_state() -> diagnostics
load_model(model_bundle) -> status
```

## 4. MLP baseline

```text
66 -> Linear(64) + ReLU
   -> Linear(32) + ReLU
   -> Linear(16) + ReLU
   -> Linear(2)  + [sigmoid(s_v), tanh(omega_norm)]
```

跨车辆版本含 bias 参数量为 `6,930`。单车 49 维版本参数量为 `5,842`。第一版不使用 CNN、Transformer 或大型特征提取器。

## 5. Temporal Model 版本

保持 Sensor Encoder 和 Control Head 接口不变：

```text
66 -> 64 -> 32 -> latent(16)
                  -> TemporalModel -> ControlHead -> 2
```

统一接口：

```text
reset()
step(z_t, dt_s) -> h_t
get_state() -> h_t
set_state(h_t)
```

替换项：

- `None/MLP`：无状态，作为 baseline；
- `GRU`：固定 hidden size；
- `LSTM`：显式管理 `h/c`；
- `CfC/LNN`：显式使用 `dt_s`，用于不规则采样和连续时间控制研究。

比较时固定 Encoder、Head、数据切分、延迟和隐藏宽度，并报告参数量、RAM、Flash、推理延迟和 WCET。

## 6. 训练数据

每个 episode 使用 `[T,...]` 数组，至少包括：

```text
t_us, dt_s                         [T]
gray_value                         [T,16]
gray_exist, gray_valid             [T,16]
sensor_position_m                  [T,16]  # raw profile geometry
sensor_to_axle_m, desired_speed_mps [T]
sensor_position_norm                [T,16]  # model input
sensor_to_axle_norm                 [T]
target_linear_speed_mps            [T]
target_angular_speed_radps         [T]
expert_source, label_delay_s       [T]
```

建议同时记录由 Mixer 得到的 target wheel speeds、实际轮速、横向误差、线置信度、场景标签和 Safety 状态。标签是专家希望执行的 `v, omega`，不是实际轮速。

训练时由标签派生：

```text
s_v_label = clip(target_linear_speed_mps / desired_linear_speed_mps, 0, 1)
```

当 `desired_linear_speed_mps=0` 时不计算该比值，直接使用确定性停止规则。

数据可由传统灰度 PID、人工遥控或 Expert Controller 生成。必须区分 `invalid_expert`、`valid_recovery` 和普通 expert：传感器/时间戳损坏或不安全的专家动作可以排除；接近出线后的有效恢复行为必须保留。

## 7. 采集和增强

采集直线、普通弯、急弯、S 弯、交叉线、短时丢线和横向扰动，覆盖不同速度、光照、线路宽度、传感器数量、物理跨度和 sensor-to-axle 距离。

训练时主动加入：

- 单个、多个和成片灰度传感器 dropout；
- 反射率、增益、零点、整体光照和局部强光变化；
- 随机噪声、线路宽度变化、控制延迟和传感器延迟；
- 速度变化、横向扰动、弯道和短时间丢线。

硬件缺失设置 `exist=0, valid=0`；瞬时失效设置 `exist=1, valid=0`。校准和归一化后，最终模型 value 清零但保留 mask。

## 8. Covariate Shift 与 DAgger 数据

纯 Expert 数据主要覆盖正常轨迹附近，LineFollower 一旦犯错可能进入训练集没有覆盖的状态。后续加入 intervention/DAgger：

```text
NN 驾驶
  -> 进入偏离状态
  -> Expert 接管或纠正
  -> 记录当前观测和 Expert 应执行的 v, omega
  -> 加入训练集并重新训练
```

DAgger 数据必须记录接管原因、接管时刻和安全状态。可以先用横向扰动生成近似 intervention 数据，再进行真实人工接管采集。

## 9. 传统灰度 PID baseline

按线路极性得到每路响应 `q_i`，再计算：

```text
position_m = sum(sensor_position_m[i]*q_i) / max(sum(q_i), epsilon)
e = position_m - center_m
s_v = speed_policy(desired_linear_speed_mps, line_confidence)
v = s_v * desired_linear_speed_mps
omega = Kp*e + Ki*integral(e) + Kd*de/dt
```

灰度 PID 输出 `v, omega`，再由 Mixer 使用 `track_width_m` 转成左右轮速度。低置信度/丢线使用确定性状态机：短暂保持、按最后方向搜索、超时减速或停车。积分、输出和速度 ramp 均限幅。

## 10. 训练和评估

首版先做监督回归，按完整 run/车辆/轨道切分数据，禁止随机打散相邻帧。后续 Temporal Model 按完整序列或带 burn-in 的窗口训练，episode 边界必须 reset。

至少比较：

```text
灰度 PID / 状态机
MLP
GRU
LSTM
CfC/LNN（后续）
```

测试 clean、单/多传感器故障、噪声/增益偏差、延迟、不同速度和未见轨道。

横向误差必须区分：

- `gray_estimated_lateral_error`：由灰度阵列估算的 proxy，只用于运行时诊断；
- `ground_truth_lateral_error`：由上方相机、AprilTag、视觉定位或已知轨道数字模型得到，只用于测试评价。

如果没有独立 GT，指标名称必须写成 gray-estimated，不能声称是真实横向误差。

指标还包括完成率、出线率、重新捕获成功率、恢复时间、控制平滑度、左右轮抖动、推理延迟、参数量、RAM 和 Flash。测试 clean、单/多传感器故障、噪声/增益偏差、延迟、不同速度和未见车辆/轨道。
