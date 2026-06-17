# MWC Soccer Control

`mwc_soccer_control` 是点球/守门主控包，接口消息定义在 `soccer_msgs`。

## 目录

- `mwc_soccer_control/src/soccer_brain_node.cpp`
- `mwc_soccer_control/config/soccer_brain.yaml`
- `mwc_soccer_control/config/kick3_v0_50000.onnx`
- `mwc_soccer_control/config/kick3.trajbin`
- `mwc_soccer_control/launch/soccer_brain.launch.py`
- `mwc_soccer_control/launch/pre_action_flow_test.launch.py`
- `mwc_soccer_control/scripts/pre_action_flow_test.py`

## 依赖与环境

启动前先加载 ROS2 和 Unitree/G1 工作区环境：

```bash
source /opt/ros/humble/setup.bash
source /home/zigzagson/Documents/Code/g1robot/install/setup.bash
source /home/zigzagson/Documents/Code/MWCsoccer/install/setup.bash
```

如果当前机器的 `~/.ros/log` 不可写，可以把日志目录切到临时目录：

```bash
export ROS_LOG_DIR=/tmp/ros_logs
```

## 构建

在工作区根目录执行：

```bash
colcon build
```

如果只想单独构建主控包：

```bash
colcon build --packages-select mwc_soccer_control soccer_msgs
```

## 正式启动

正式启动命令：

```bash
ros2 launch mwc_soccer_control soccer_brain.launch.py
```

这个 launch 会默认加载：

- [mwc_soccer_control/config/soccer_brain.yaml](./mwc_soccer_control/config/soccer_brain.yaml)

## 启动流程

1. 先确认底层机器人栈已经起来，并且这些接口可用：
   - `/soccer/nav_status`
   - `/soccer/perception`
   - `/whole_body/action_ctrl`
2. 启动 `mwc_soccer_control`。
3. 点球模式下，主控的状态流转为：
   `PENALTY_ATTACK -> START_BALL_TRACK -> ALIGN -> STOP_BALL_TRACK -> READY_KICK -> KICK -> FINISH`
4. `ALIGN` 阶段会临时把 G1 loco FSM 切到 `812`，结束后恢复到 `802`。

## 参数文件

主控参数统一放在 YAML 里，不需要改代码。正式启动默认读取 [mwc_soccer_control/config/soccer_brain.yaml](./mwc_soccer_control/config/soccer_brain.yaml)。模型和轨迹默认由 launch 根据包内 `config/` 自动拼接。

### 模式与射门目标

- `start_mode`：启动后进入的模式。当前主控实现的是 `PENALTY_ATTACK`。
- `auto_start`：是否启动节点后自动发布一次点球模式。现场联调通常保持 `true`；如果要外部上位机触发，可以改成 `false`。
- `goal_target`：点球目标横向位置，左门柱为 `-1`，右门柱为 `1`。例如 `0.3` 表示中间偏右。这个值会原样发给导航模块，由导航基于 map 系下的球和球门坐标反算定点导航点。
- `nav_ball_distance_m`：导航停在球前的距离。调大机器人离球更远，调小机器人更贴近球；太小会影响最后 ALIGN 和起脚空间。

### Unitree 运控模式

- `unitree_network_interface`：Unitree SDK2 DDS 绑定网卡。为空时走默认配置；如果 DDS 发现网卡不对，再填实际网卡名。
- `align_obstacle_fsm_id`：ALIGN 微调阶段切入的运控 FSM。当前使用 `812`，也就是越障模式踏步。
- `align_restore_fsm_id`：ALIGN 结束后的恢复 FSM。当前使用 `802`，常规走跑模式。
- `restore_fsm_after_align`：ALIGN 结束后是否恢复 FSM。实机建议保持 `true`。
- `require_unitree_align_mode`：切换到 `812` 失败时是否直接报错。实机建议保持 `true`；纯流程测试可改成 `false`。
- `align_use_continuous_gait`：ALIGN 微调阶段是否启用 `ContinuousGait(true)`。当前默认 `false`，避免 812 模式下速度为 `0` 时持续踏步；需要保持连续步态时可改成 `true`，退出 ALIGN 时会发送 `ContinuousGait(false)`。

### 状态机时间

- `control_rate_hz`：主控状态机循环频率。默认 `20 Hz`，一般不用改。
- `nav_timeout_s`：等待导航到点的最大时间。当前默认 `45.0s`，用于容忍导航绕行或场地较大。
- `nav_status_timeout_s`：`/soccer/nav_status` 超时阈值。当前默认 `2.0s`，用于容忍短时状态抖动。
- `align_timeout_s`：ALIGN 微调最大时间。当前默认 `20.0s`，用于避免视觉/踏步慢时过早失败。
- `perception_timeout_s`：视觉感知超时阈值。当前默认 `0.6s`，用于容忍相机、SDK 推理或 TF 短时延迟。
- `post_track_settle_s`：打开视觉跟踪后等待感知稳定的时间。当前默认 `1.0s`，避免刚打开跟踪时还没出有效球坐标就报错。
- `ball_perception_wait_timeout_s`：打开视觉跟踪后最长等待有效球感知的时间。当前默认 `8.0s`，超过后按 `kick_on_ball_perception_timeout` 决定直接开踢还是进入 `ERROR`。
- `kick_on_ball_perception_timeout`：打开视觉跟踪后一直没有有效球感知时是否跳过 ALIGN 直接开踢。当前默认 `true`。
- `pre_kick_pause_s`：READY_KICK 到真正发送射门 action 前的停顿。当前默认 `0.5s`，用于让机器人站稳。

### ALIGN 目标与稳定判定

- `enable_align`：是否启用到点后的视觉微调。当前默认 `true`；设为 `false` 时打开视觉跟踪并等待 `post_track_settle_s` 后直接开踢，不进入 ALIGN。
- `align_target_ball_x_m`：期望球在 dummy/body 坐标系前方的距离。默认 `0.22 m`。
- `align_target_ball_y_m`：期望球的横向偏移。默认 `0.0 m`，表示球在正前方。
- `align_x_tolerance_m`：前后方向容差。当前默认 `0.08m`，调大更容易进入射门，调小对位更严格。
- `align_y_tolerance_m`：左右方向容差。当前默认 `0.06m`，调大更快通过，调小能提高起脚横向一致性。
- `align_yaw_tolerance_rad`：朝向容差。当前默认 `0.12rad`，调小会要求机器人更正对球/门。
- `align_required_stable_frames`：连续多少帧都满足容差才认为 ALIGN 成功。当前默认 `3` 帧，调大更稳但更慢，调小更快但容易误判。
- `kick_on_align_ball_lost_near_feet`：ALIGN 中如果最后一次有效球坐标已经接近脚前盲区，随后丢球时是否直接开踢。当前默认 `true`。
- `align_lost_ball_kick_x_m`：近脚丢球开踢的前向距离阈值。当前默认 `0.45m`，用于覆盖相机约 `0.40m` 以内看不到球的情况。
- `align_lost_ball_kick_y_tolerance_m`：近脚丢球开踢时允许的最后横向误差。当前默认 `0.10m`。
- `align_lost_ball_kick_yaw_tolerance_rad`：近脚丢球开踢时允许的最后朝向误差。当前默认 `0.18rad`。
- `kick_on_align_timeout`：ALIGN 超时后是否走开踢 fallback。当前默认 `true`；触发前必须先成功恢复 FSM，默认恢复到 `802`，恢复失败则进入 `ERROR` 不踢。

### ALIGN 微调控制

- `align_kx`：前后误差到踏步前后速度的比例系数。调大前后修正更快，过大会晃。
- `align_ky`：横向误差到踏步横向速度的比例系数。右偏/左偏修正不够时优先调这个。
- `align_kw`：朝向误差到角速度的比例系数。转向慢就调大，转向过冲就调小。
- `align_max_vx`：ALIGN 前后速度上限。限制每次向前/后踏步速度。
- `align_max_vy`：ALIGN 横向速度上限。限制左右踏步速度。
- `align_max_wz`：ALIGN 角速度上限。限制原地转向速度。
- `align_step_duration_s`：每次 `SetVelocity` 指令持续时间。当前默认 `0.45s`，调大单次修正更明显。
- `align_min_step_period_s`：每次速度指令结束后的观测稳定等待时间。当前默认 `0.15s`；等待结束后还必须收到新的 `/soccer/perception`，才会计算并发送下一次修正。

### 感知有效性

- `min_ball_confidence`：球检测置信度下限。当前默认 `0.30`，误检多就调高，漏检多就调低。
- `min_goal_confidence`：球门检测置信度下限。当前默认 `0.25`。只有检测到球门且置信度足够时，主控才优先用球门和球的相对关系算朝向误差。

### 射门 action

- `kick_action_server`：whole-body action server 名称，默认 `/whole_body/action_ctrl`。
- `kick_action_name`：发送给 whole-body 的动作名，同时作为 JSON 里的 `uuid` 和 `state_name`。
- `enable_kick_action`：是否真的发送射门 action。闭环测试里会置为 `false`，实机射门保持 `true`。
- `kick_server_timeout_s`：等待 action server 出现的时间。
- `kick_result_timeout_s`：等待射门 action 返回结果的时间。
- `kick_action_params_json`：完整 action JSON。非空时优先使用它，适合临时试验完整配置。
- `kick_model_path` / `kick_trajectory_path`：模型和轨迹路径。正式 launch 默认从包内 `config/` 自动注入，不需要手写绝对路径。
- `kick_policy_type`：whole-body 动作使用的策略类型，默认 `lingshu`。
- `kick_end_behavior`：动作结束后的行为，默认 `switch_to_loco`。
- `kick_quat_comp`：自动生成 action JSON 时写入的 `quat_comp`，默认 `-0.2`。

## 闭环测试

这个 launch 只测触发射门前的闭环，不发真实 kick action：

```bash
ros2 launch mwc_soccer_control pre_action_flow_test.launch.py
```

它会：

- 启动主控
- 把 `enable_kick_action` 关掉
- 模拟 `/soccer/nav_status`
- 模拟 `/soccer/perception`
- 检查流程是否能走到 `FINISH`

默认测试的微调量大约是：

- 前 `10 cm`
- 右 `15 cm`

也可以覆盖：

```bash
ros2 launch mwc_soccer_control pre_action_flow_test.launch.py \
  sim_align_ball_x_m:=0.32 sim_align_ball_y_m:=-0.15
```

## 调参建议

- 如果要改微调力度，优先调 `align_kx`、`align_ky`、`align_step_duration_s`
- 如果要改 ALIGN 阶段的运控模式，改 `align_obstacle_fsm_id`
- 如果要改恢复模式，改 `align_restore_fsm_id`
- 如果要换真实射门动作，优先改 `kick_action_params_json`，或者用 `kick_model_path` / `kick_trajectory_path`

## 排错

- `ros2 launch` 里如果报 topic QoS 不匹配，先检查发布端和订阅端的 durability 设置是否一致
- 如果 DDS 起不来，先确认已经 `source /home/zigzagson/Documents/Code/g1robot/install/setup.bash`
- 如果测试脚本没有进入状态机，先检查 `/soccer/nav_status` 和 `/soccer/perception` 是否真的在发布
