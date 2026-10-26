# MotorController 网络设计

## 1. 职责

MotorController 是低层单轮闭环模块：让电机实际速度跟踪上层给出的目标速度。左右轮使用相同拓扑但不同权重。

```text
TargetSpeed + MotorObservation -> MotorController -> PWM
```

它不知道目标来自循迹、遥控还是导航，也不读取灰度、相机或路径规划数据。

## 2. 输入接口

核心字段：

```text
target_speed_mps : float32       # 必须存在
actual_speed_mps : SensorField  # 编码器估计的实际轮速
previous_applied_pwm : float32  # safe_pwm_applied[t-1]
```

可选字段均使用 `value/exist/valid`：

```text
battery_voltage, motor_current, motor_temperature,
driver_temperature, acceleration, wheel_acceleration,
bus_voltage, bus_current, load_estimate
```

`previous_applied_pwm` 是运行时状态，复位为 0，不是可缺失外部传感器。它必须严格满足：

```text
previous_applied_pwm[t] = safe_pwm_applied[t-1]
```

`actual_speed` 是 Core Input。若 `actual_speed.exist=0` 或有效性检查失败，wrapper 必须 bypass NN；若没有另一个有效的速度估计 fallback，则进入确定性 ramp-down/stop。NN 不训练“如何处理编码器失效”。

`actual_speed_mps` 不直接由 MotorController 计算，而由公共模块产生：

```text
Encoder -> VelocityEstimator -> actual_speed_mps
                                  ├──> PI/PID
                                  └──> MotorController NN
```

PI、PID 和 NN 必须使用完全相同的速度估计结果。低速时不能简单使用固定 2 ms 周期的 `delta_count/dt`；VelocityEstimator 应在 profile 中选择 count/window、pulse-period、互补估计和低通滤波等方法，并记录估计延迟和有效性。

## 3. 输入输出张量

MVP 使用 6 维核心/电池输入。actual speed 的 exist/valid 属于 wrapper health interface，不进入正常 NN 输入：

```text
x_motor_mvp = [
  target_speed_norm, actual_speed_norm,
  previous_applied_pwm_norm,
  battery_voltage_norm, battery_voltage_exist, battery_voltage_valid
]
```

完整输入固定为 30 维：先放上述 3 个连续核心字段，再按以下顺序追加每个 optional 字段的 value、exist、valid：

```text
battery_voltage, motor_current, motor_temperature,
driver_temperature, acceleration, wheel_acceleration,
bus_voltage, bus_current, load_estimate
```

硬件配置只改变 mask，不改变网络维度。

输出为 `float32[1]`：

```text
y_motor = [pwm_norm]    # [-1, 1]
```

方向和 duty 映射由 Actuator Adapter 完成。

## 4. MLP baseline

```text
6 -> Linear(32) + ReLU
  -> Linear(16) + ReLU
  -> Linear(8)  + ReLU
  -> Linear(1)  + tanh
```

MVP 参数量为 `897`。左、右电机分别加载一份权重，允许 `θ_L != θ_R`。

## 5. Temporal Model 版本

后续可将电机输入先编码为 latent，再替换时序模块：

```text
Motor Sensor Encoder -> latent z_t
                     -> None / GRU / LSTM / CfC
                     -> PWM Head
```

统一接口：

```text
reset()
step(z_t, dt_s) -> h_t
get_state() -> h_t
set_state(h_t)
```

`dt_s` 以秒为单位。时序状态用于表达电机惯性、加减速趋势、负载变化、摩擦、延迟和电池状态；MVP 先使用无状态 MLP。

## 6. 数据格式和采集

Motor episode 至少包括：

```text
t_us, dt_s, target_speed_mps
actual_speed_mps, actual_speed_exist, actual_speed_valid
previous_applied_pwm
所有 optional 字段及其 exist/valid
expert_pwm_raw
pi_pwm_raw
nn_delta_raw
controller_pwm_raw
safe_pwm_applied
safety_reason, safety_clamp_flags
```

其中：

- `expert_pwm_raw` 是专家/标签命令；
- `pi_pwm_raw` 是未经过 Safety 的 PI 命令；
- `nn_delta_raw` 是 NN 补偿；
- `controller_pwm_raw` 是 PI 与 NN 合成后、Safety 之前的命令；
- `safe_pwm_applied` 是 Safety 限幅后真正施加的命令。

Pure NN/Residual NN 的监督标签只能使用 raw controller 语义，不能用 `safe_pwm_applied` 反推 NN 标签。Safety 行为只作为状态、指标和故障分析记录，不能偷偷变成 NN 的学习目标。

采集速度阶跃、斜坡、正反转、负载变化、左右电机差异和不同电池电压。扫频和阶跃必须受电流、温度、堵转和急停阈值限制。

训练/验证按完整 run、车辆、负载和电池条件切分，不能随机打散相邻帧。数据还必须记录 VelocityEstimator 的 method/profile、估计延迟和 stale/fault 原因。

## 7. 传统 PI/PID baseline

每个轮子独立维护误差积分：

```text
e_t = target_speed - actual_speed
u_PI = saturate(Kff*target_speed + Kp*e_t + Ki*integral(e_t))
```

必须实现 anti-windup、积分限幅、输出限幅和故障处理。PID 版本额外加入带滤波的微分项。PI 是默认 fallback，PID 是实验比较项。

## 8. Pure NN

```text
u = NN(target_speed, actual_speed, optional motor state)
```

Pure NN 用专家 PWM 作为监督标签，用于研究神经网络是否可以完全替代传统控制器；它不是首版默认安全路径。

## 9. Residual Neural Controller

工程默认优先研究 Residual 方案：

```text
u_pi       = PI(input)
delta_u_nn = alpha * NN(input)       # NN ∈ [-1,1]
u_command  = saturate(u_pi + delta_u_nn)
```

训练标签是专家 PWM 相对 PI 的补偿，补偿幅度受 `alpha` 限制。模型未加载、输入无效、推理超时、输出非有限或 watchdog fault 时令 `delta_u_nn=0`，系统退化为 PI；若 actual speed core input 无效，则连 PI 也必须遵守速度反馈 fault policy，不得伪装成正常闭环。

### 9.1 Expert 定义与实验路径

`expert_pwm_raw` 必须明确来源，不能默认等于 PI：

| 模式 | Teacher / 优化目标 |
|---|---|
| Pure NN imitation | 高质量 Expert Controller 的 raw PWM |
| Residual imitation | `expert_pwm_raw - pi_pwm_raw` |
| Closed-loop NN（后期） | 直接优化速度 tracking、平滑度和约束后的闭环性能 |

第一阶段 Expert 推荐使用经过台架验证的 `feedforward + PID` 或高质量 PID；后续可使用系统辨识模型上的 MPC 作为 teacher。若 `expert=PI`，Residual 标签恒为 0，不能作为 Residual NN 的有效训练数据。

### 9.2 Previous Applied PWM 的因果一致性

虽然 MLP 没有显式 hidden state，但 `previous_applied_pwm` 使整个闭环成为隐式递归系统。训练数据中专家的上一 applied PWM 与部署时模型/Safety 产生的上一 applied PWM 分布不同，会产生 teacher-forcing 问题。

训练必须同时覆盖：

```text
previous_applied_pwm_train = previous_applied_pwm + epsilon
epsilon ~ bounded_noise
previous_applied_pwm_train = clip(previous_applied_pwm_train, -1, 1)
```

并在训练后进行 free-running closed-loop rollout：使用上一周期实际 `safe_pwm_applied` 作为下一周期 `previous_applied_pwm`，配合下一帧观测作为后续输入。验证不能只看逐帧离线 RMSE；必须记录漂移、发散、恢复时间和 Safety 触发率。Residual 版本尤其要覆盖“上一周期被 Safety 限幅”的状态。

NN 主要学习：

- 电池电压补偿；
- PWM 死区和电机非线性；
- 摩擦、负载和惯性；
- 左右电机差异；
- 老化和机械误差。

## 10. 训练增强

主动随机化：

- 电池电压、PWM 死区、编码器噪声和比例误差；
- 负载、摩擦、电机常数、惯性和左右参数差异；
- PWM-to-torque 非线性、机械阻力、通信延迟和控制周期抖动；
- battery/current/temperature 等 optional sensor 缺失。

硬件缺失使用 `exist=0, valid=0`；瞬时故障使用 `exist=1, valid=0`。校准和归一化后，最终模型 value 清零并保留 mask。

## 11. 训练和评估

至少比较：

```text
PI
PID
Pure NN MLP
Residual PI + NN
GRU/CfC（后续）
```

测试速度阶跃、斜坡、负载突变、左右电机差异、不同电池电压和传感器缺失。

指标包括 rise time、settling time、overshoot、steady-state error、速度 RMSE、PWM 抖动、负载突变恢复时间、不同电压误差、故障条件误差、推理延迟、参数量、RAM 和 Flash。
