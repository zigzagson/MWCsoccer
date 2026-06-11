# MWC Soccer Control

`mwc_soccer_control` 是点球/守门主控包，接口消息定义在 `soccer_msgs`。

## 目录

- `mwc_soccer_control/src/soccer_brain_node.cpp`
- `mwc_soccer_control/config/soccer_brain.yaml`
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

主控参数统一放在 YAML 里，不需要改代码。模型和轨迹默认由 launch 根据包内 `config/` 自动拼接。

关键参数如下：

- `align_obstacle_fsm_id: 812`
- `align_restore_fsm_id: 802`
- `kick_action_server: /whole_body/action_ctrl`
- `enable_kick_action: true`

射门动作有两种配置方式：

1. 直接写完整 JSON 到 `kick_action_params_json`
2. 留空 `kick_action_params_json`，让 launch 从包内 `config/` 自动注入 `kick_model_path` 和 `kick_trajectory_path`

示例：

```yaml
由 launch 自动注入，不需要在 YAML 里手写
```

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
