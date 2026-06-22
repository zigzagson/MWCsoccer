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

## 实时可视化

构建并加载工作区后，另开一个终端启动可视化节点：

```bash
ros2 launch mwc_soccer_control soccer_visualizer.launch.py
```

本机浏览器打开：

```text
http://127.0.0.1:18080
```

可视化节点只订阅 ROS2 topic，不会发送模式或运动控制命令。它实时展示：

- 点球状态机、球在机器人坐标系中的位置、期望球位、对位误差和速度指令
- 守门图像归一化坐标、中心死区、球位置和机器人横移方向
- 导航、感知、行为状态和速度 topic 的数据新鲜度
- 最近的状态切换事件及球位置/横移速度趋势

默认监听 `0.0.0.0:18080`，同一局域网内可通过机器人电脑 IP 访问。只允许
本机访问时使用：

```bash
ros2 launch mwc_soccer_control soccer_visualizer.launch.py \
  bind_address:=127.0.0.1 port:=18080
```

面板订阅 `/soccer/behavior_state`、`/soccer/perception`、
`/soccer/nav_status`、`/soccer/game_mode_cmd` 和 `/nav/cmd_vel_nav`。
点球画布以点球点和足球为固定参考：机器人到球的目标距离与角度由
`align_target_ball_x_m`、`align_target_ball_y_m` 决定。视觉跟踪开始后，
面板根据实时球相对坐标反算机器人位置并绘制移动轨迹；导航阶段如果没有
`/odom` 或全局位姿，只显示导航状态和目标站位。
如果修改了主控的对位目标、容差或守门死区，应把相同参数传给可视化节点，
保证图形标记与主控配置一致，例如：

```bash
ros2 launch mwc_soccer_control soccer_visualizer.launch.py \
  align_target_ball_x_m:=0.8 align_target_ball_y_m:=-0.3 \
  align_x_tolerance_m:=0.1 align_y_tolerance_m:=0.07 \
  goalkeeper_center_deadband_x:=0.05
```

这个 launch 会默认加载：

- [mwc_soccer_control/config/soccer_brain.yaml](./mwc_soccer_control/config/soccer_brain.yaml)

正式配置默认从 `IDLE` 启动，并直接从 Unitree
`rt/lowstate.wireless_remote` 读取遥控器组合键：

- `L1 + R1 + A`：进入 `IDLE`
- `L1 + R1 + X`：使用基础参数启动点球
- `L1 + R1 + Y`：使用 `goalkeeper_default` 预设启动守门
- `L1 + R1 + 下`：循环选择下一套庆祝动作，仅 `IDLE` 时有效
- `L1 + R1 + B`：执行当前选中的庆祝动作，仅 `IDLE` 时有效
- `L1 + R1 + 左/上/右`：分别使用 `penalty_left`、
  `penalty_center`、`penalty_right` 点球预设

组合键需要持续按住 `remote_combo_hold_s`，触发后进入
`remote_combo_cooldown_s` 冷却期。模式切换会先统一进入 `IDLE`，
停止速度和视觉跟踪、取消尚未完成的射门 Action、恢复运控 FSM，
再启动目标模式。

庆祝动作与踢球动作使用相同类型的 whole-body action。正式 launch 会加载
`config/soccer_celebration_actions/` 中的四套动作，默认选择内马尔庆祝舞。
每次按住 `L1 + R1 + 下` 会依次切换为前跳庆祝、举手庆祝、伸展挥手，再回到
内马尔庆祝舞。当前选择会显示在网页面板并写入 ROS 日志。动作成功后自动
返回 `IDLE`；执行期间可用 `L1 + R1 + A` 取消并复位。

只测试四个动作的循环切换和网页显示时，不要启动正式主控，运行：

```bash
ros2 launch mwc_soccer_control celebration_selection_test.launch.py
```

浏览器打开 `http://127.0.0.1:18080`。测试节点每两秒切换一次显示状态，
不会连接或发送 whole-body Action。可用 `interval_s:=5.0` 修改切换间隔。

## 启动流程

1. 先确认底层机器人栈已经起来，并且这些接口可用：
   - `/soccer/nav_status`
   - `/soccer/perception`
   - `/whole_body/action_ctrl`
2. 启动 `mwc_soccer_control`。
3. 点球模式下，主控的状态流转为：
   `PENALTY_ATTACK -> START_BALL_TRACK -> ALIGN -> STOP_BALL_TRACK -> READY_KICK -> KICK -> FINISH`
4. 守门模式下，主控保持在 `GOALKEEPER -> TRACK_BALL`，持续根据球的图像横坐标闭环横移。
5. `ALIGN` 阶段会临时把 G1 loco FSM 切到 `812`，结束后恢复到 `802`。

## 参数文件

主控参数统一放在 YAML 里，不需要改代码。正式启动默认读取 [mwc_soccer_control/config/soccer_brain.yaml](./mwc_soccer_control/config/soccer_brain.yaml)。模型和轨迹默认由 launch 根据包内 `config/` 自动拼接。

### 模式与射门目标

- `start_mode`：启动后进入的模式，可设为 `PENALTY_ATTACK` 或 `GOALKEEPER`。
- `auto_start`：是否启动节点后自动发布一次点球模式。现场联调通常保持 `true`；如果要外部上位机触发，可以改成 `false`。
- `goal_target`：点球目标横向位置，左门柱为 `-1`，右门柱为 `1`。例如 `0.3` 表示中间偏右。这个值会原样发给导航模块，由导航基于 map 系下的球和球门坐标反算定点导航点。
- `nav_ball_distance_m`：导航停在球前的距离。调大机器人离球更远，调小机器人更贴近球；太小会影响最后 ALIGN 和起脚空间。

### Unitree 运控模式

- `unitree_network_interface`：Unitree SDK2 DDS 绑定网卡。为空时走默认配置；如果 DDS 发现网卡不对，再填实际网卡名。
- `velocity_command_topic`：点球微调和守门横移统一发布速度的 `Twist` topic，默认 `/nav/cmd_vel_nav`。速度使用 `linear.x`、`linear.y`、`angular.z`；立即停止时三项速度置零并设置 `linear.z=1`，由 `LegController::nav_cmd_callback` 调用 `StopMove()`。
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
- `align_required_stable_frames`：连续多少个不同的静止感知帧都满足容差才认为 ALIGN 成功。当前默认 `3` 帧；同一帧不会被控制定时器重复累计。
- `kick_on_align_ball_lost_near_feet`：ALIGN 中如果最后一次有效球坐标已经接近脚前盲区，随后丢球时是否直接开踢。当前默认 `true`。
- `align_lost_ball_kick_x_m`：近脚丢球开踢的前向距离阈值。当前默认 `0.45m`，用于覆盖相机约 `0.40m` 以内看不到球的情况。
- `align_lost_ball_kick_y_tolerance_m`：近脚丢球开踢时允许的最后横向误差。当前默认 `0.10m`。
- `align_lost_ball_kick_yaw_tolerance_rad`：近脚丢球开踢时允许的最后朝向误差。当前默认 `0.18rad`。
- `kick_on_align_timeout`：ALIGN 超时后是否走开踢 fallback。当前默认 `true`；触发前必须先成功恢复 FSM，默认恢复到 `802`，恢复失败则进入 `ERROR` 不踢。

### ALIGN 微调控制

- `align_kx`：前后误差到踏步前后速度的比例系数。调大前后修正更快，过大会晃。
- `align_ky`：横向误差到踏步横向速度的比例系数。右偏/左偏修正不够时优先调这个。
- `align_kw`：朝向误差到角速度的比例系数。转向慢就调大，转向过冲就调小。
- `align_max_vx`：ALIGN 前后速度上限。当前默认 `0.50`，限制每次向前/后踏步速度。
- `align_max_vy`：ALIGN 横向速度上限。当前默认 `0.50`，限制左右踏步速度。
- `align_max_wz`：ALIGN 角速度上限。当前默认 `0.50`，限制原地转向速度。
- `align_min_speed`：ALIGN 各方向的最小非零速度。当前默认 `0.20`；某方向误差超过对应阈值时，该方向速度绝对值不会低于此值，进入对应阈值后该方向速度为 `0`。
- `align_step_duration_s`：`/nav/cmd_vel_nav` 单次速度命令的实际生效时间。`LegController` 当前调用默认 `continous_move=false` 的三参数 `Move()`，因此固定按 `1.00s` 配置；到期后主控再确认站立并采样。
- `align_min_step_period_s`：`GetFsmMode` 确认站立后的观测稳定等待时间。当前默认 `0.15s`；等待结束后还必须收到站立确认之后的新 `/soccer/perception`，才会计算并发送下一次修正。
- `align_require_standing_for_sample`：是否要求 `GetFsmMode()==0` 后才采纳 ALIGN 坐标。默认 `true`；移动期间、停稳前和重复的感知帧都不参与误差计算及稳定计数。

### 守门横向闭环控制

守门模式复用 `/soccer/perception.ball`，但坐标含义是图像中心坐标：`x<0` 为左、`x>0` 为右、`y<0` 为下、`y>0` 为上，`z=0`。只有 `image_has_ball=true`、`transform_valid=true` 且球置信度达标的帧才参与控制；`image_has_goal` 和球门置信度不参与守门判断。主控持续调节 `vy` 使球保持在 `x=0` 附近，始终保持 `vx=0`、`wz=0`。

不驱动机器人、只测试消息和状态流程：

```bash
ros2 launch mwc_soccer_control goalkeeper_flow_test.launch.py
```

该测试依次模拟居中、右偏、重新居中、左偏和无效感知。测试 launch 固定覆盖 `goalkeeper_enable_motion=false`。

- `goalkeeper_enable_motion`：是否实际调用 `SetVelocity`。正式运行默认 `true`，守门流程测试 launch 会覆盖为 `false`。
- `goalkeeper_center_deadband_x`：图像中心死区，默认 `0.05`；球在该范围内时发送零速。
- `goalkeeper_lateral_kp`：图像横向误差到机器人横移速度的比例系数，默认 `2.0`。
- `goalkeeper_min_lateral_speed`：超出死区后的最小非零横移速度，默认 `0.20m/s`。
- `goalkeeper_lateral_speed`：机器人横移速度上限，默认最高速 `1.00m/s`。
- `goalkeeper_lateral_sign`：图像方向到机器人 `vy` 的符号映射，默认 `1.0`；实机方向相反时改为 `-1.0`。
- `goalkeeper_perception_timeout_s`：守门感知停止阈值，默认 `1.00s`。持续收到无效帧满 1 秒，或感知 topic 连续 1 秒未更新时，主控发送零速停止。

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
- `kick_vendor_name_en` / `kick_vendor_name_cn`：动作供应商英文和中文名称，
  非空时写入自动生成的 Action params JSON，为空时省略对应字段。
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

- 从 `(0.32m, -0.15m)` 移动到当前目标 `(0.8m, -0.3m)`
- 纯流程测试不要求连接真实 Unitree FSM，也不会发送真实射门 action

也可以覆盖：

```bash
ros2 launch mwc_soccer_control pre_action_flow_test.launch.py \
  sim_align_ball_x_m:=0.32 sim_align_ball_y_m:=-0.15 \
  sim_settle_ball_x_m:=0.8 sim_settle_ball_y_m:=-0.3
```

## 调参建议

- 如果要改微调力度，优先调 `align_kx`、`align_ky` 和速度上下限；当前 topic 接口的单条 `Move()` 固定生效 1 秒。
- 如果要改 ALIGN 阶段的运控模式，改 `align_obstacle_fsm_id`
- 如果要改恢复模式，改 `align_restore_fsm_id`
- 如果要换真实射门动作，优先改 `kick_action_params_json`，或者用 `kick_model_path` / `kick_trajectory_path`

## 排错

- `ros2 launch` 里如果报 topic QoS 不匹配，先检查发布端和订阅端的 durability 设置是否一致
- 如果 DDS 起不来，先确认已经 `source /home/zigzagson/Documents/Code/g1robot/install/setup.bash`
- 如果测试脚本没有进入状态机，先检查 `/soccer/nav_status` 和 `/soccer/perception` 是否真的在发布
