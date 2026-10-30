# SmartLine R4 实验路径

本文是 R3 冻结接口之外的**可选实验实现**。默认构建、R3 权重和现有控制路径保持不变；R4 只替换巡线 tracking 控制器，Recovery、Mixer、Safety 和电机 PI+FF 仍使用 R3。

## 输入与控制语义

R4 固件输入为 7 维：

```text
[mu, activation, variance, dmu, ddmu, speed_norm, omega_prev_norm]
```

- 灰度先从固件约定的 `[-1, 1]`（线为 `+1`）换算到 `[0, 1]`；缺失/无效通道按 0 参与。
- 空间坐标以当前配置探头的最大绝对坐标归一化。`activation` 按 `sensor_count` 求平均。
- `dmu` 按 100 Hz 归一化并裁剪到 `[-1, 1]`；`ddmu` 是相邻归一化差分。首次采样导数为 0。
- 速度按 `v_max_nom_mps` 归一化；上一拍角速度按 `omega_ref_radps` 归一化。

部署网络：`7 -> 16(tanh) -> 8(tanh) -> 2(tanh)`，282 个权重/偏置参数。输出 0 是缩放后的转向残差，输出 1 映射为安全速度区间的系数。运行时先计算置信度足够时的 PD 基线，再加 NN 残差，最后仍经过现有角速度包络。R4 权重缺失或输出非有限数时退回 PID。

训练时另有线性 privileged head 预测归一化真实横向误差和赛道曲率；该 head 不导出。损失包含控制标签 MSE、特权标签 MSE，以及利用 PD 基线与 `omega_prev` 的转向平滑项。

## 训练和启用

```sh
cmake -S . -B build
cmake --build build -j
build/nncar_line_ab --dump-r4-teacher build/line_r4_teacher.bin --episodes 240 --seed 1
python3 tools/python/train_line_r4.py \
  --data build/line_r4_teacher.bin \
  --out-json configs/line_r4_weights.json \
  --epochs 100
cmake -S . -B build-r4 -DNNCAR_LINE_R4_WEIGHTS="$PWD/configs/line_r4_weights.json"
cmake --build build-r4 -j
build-r4/nncar_line_ab --r4-check
build-r4/nncar_line_ab --r4-random-check 80 --seed 9001
```

`--dump-r4-teacher` 使用独立 magic/schema，不会改变原 `--dump-teacher` 数据格式。训练 episode 随机切换 5–8 路探头、探头间距/高斯光斑宽度、灰度噪声/偏移、供电、电机死区、轮系惯量、车体质量、偏航惯量和轮胎抓地系数。仿真中的**植物理参数独立于控制器 profile**：控制器只看到既有 profile 与传感器/编码器反馈。训练按 episode 切分验证集。

`line_ab` 的实验路径启用可选耦合车体模型：轮速与车体速度通过有饱和上限的轮胎纵向力耦合，车体质量/偏航惯量、滚阻和偏航阻尼影响轨迹；因轮胎滑移方程较刚，内部以 4 个子步积分；传感器使用高斯光斑响应和高斯噪声；赛道由基频与随机相位的低幅谐波叠加。固定 R3/R4 对比及同一训练 episode 的 oracle/PID/student 采样使用相同初态、噪声种子和植物理参数。oracle 的默认几何跟踪增益针对这套合成模型校准，不代表真实车辆参数。其他仿真入口默认仍是旧模型，以免更改已有 golden/replay 语义。

`--r4-random-check N` 在与训练 episode 不同的 seed 上生成 N 个随机探头/赛道/车辆实例，并对 PID、R3 MLP、R4 使用配对的相同场景与噪声，输出聚合 RMS、最大误差和 fault union。它不参与训练或 checkpoint 选择，适合作为额外 held-out 检查；不要用同一 seed 反复调参后再称为独立测试。

`--r4-check` 仅报告 R4 与 PID 的聚合横向 RMS 差异，不自动接受或替换默认控制器。应使用足量 episode 训练后再看 held-out 闭环；小样本训练只验证工具链。

## 已知边界

- 目前只实现 SmartLine R4；电机仍是 R3 PI+残差/既有模式。
- 新增车体/轮胎与高斯光斑模型仍是合成模型，尚未用实车 step-response、制动距离、偏航响应或传感器标定曲线拟合；它比旧的理想差速运动学更丰富，但不是经验证的数字孪生，也没有侧偏角/横向轮胎力、悬架、地面坡度或温度等模型。
- R4 输入仍依赖实例 profile 中的探头物理位置；它解耦了通道数量，不代表任意安装位置都无需 profile。
- 还没有实车跨车型验收。训练成功、离线误差或仿真小样本结果都不能单独证明 zero-shot 泛化。
