#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <message_center/action/execution_unit.hpp>
#include <soccer_msgs/msg/behavior_state.hpp>
#include <soccer_msgs/msg/game_mode_command.hpp>
#include <soccer_msgs/msg/nav_status.hpp>
#include <soccer_msgs/msg/soccer_perception.hpp>
#include <soccer_msgs/msg/vision_track_command.hpp>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/g1/loco/g1_loco_api.hpp>
#include <unitree/robot/g1/loco/g1_loco_client.hpp>

using namespace std::chrono_literals;

namespace mwc_soccer {

namespace {

double clampAbs(double value, double limit) {
  return std::clamp(value, -std::abs(limit), std::abs(limit));
}

std::string nowDetail(double timestamp_seconds, const std::string& detail) {
  return detail + " t=" + std::to_string(timestamp_seconds);
}

const char* boolText(bool value) {
  return value ? "true" : "false";
}

std::string jsonEscape(const std::string& value) {
  std::ostringstream out;
  for (const char c : value) {
    switch (c) {
      case '\\':
        out << "\\\\";
        break;
      case '"':
        out << "\\\"";
        break;
      case '\n':
        out << "\\n";
        break;
      case '\r':
        out << "\\r";
        break;
      case '\t':
        out << "\\t";
        break;
      default:
        out << c;
        break;
    }
  }
  return out.str();
}

}  // namespace

class SoccerBrainNode final : public rclcpp::Node {
 public:
  using SoccerPerception = soccer_msgs::msg::SoccerPerception;
  using NavStatus = soccer_msgs::msg::NavStatus;
  using GameModeCommand = soccer_msgs::msg::GameModeCommand;
  using VisionTrackCommand = soccer_msgs::msg::VisionTrackCommand;
  using BehaviorState = soccer_msgs::msg::BehaviorState;
  using ExecutionUnit = message_center::action::ExecutionUnit;
  using GoalHandleExecutionUnit = rclcpp_action::ClientGoalHandle<ExecutionUnit>;

  explicit SoccerBrainNode(const rclcpp::NodeOptions& options)
      : Node("soccer_brain_node", options) {
    declareParameters();
    loadParameters();
    initUnitreeClient();
    initRosIo();

    state_enter_time_ = now();
    publishBehavior("IDLE", "boot");

    if (auto_start_) {
      startPenaltyAttack();
    }
  }

 private:
  enum class State {
    IDLE,
    NAVIGATE_TO_POINT,
    START_BALL_TRACK,
    ALIGN,
    STOP_BALL_TRACK,
    READY_KICK,
    KICK,
    FINISH,
    ERROR,
  };

  struct AlignCommand {
    double vx = 0.0;
    double vy = 0.0;
    double wz = 0.0;
    double x_error = 0.0;
    double y_error = 0.0;
    double yaw_error = 0.0;
    bool within_tolerance = false;
  };

  void declareParameters() {
    declare_parameter<std::string>("start_mode", "PENALTY_ATTACK");
    declare_parameter<bool>("auto_start", true);
    declare_parameter<double>("goal_target", 0.0);
    declare_parameter<double>("nav_ball_distance_m", 0.75);

    declare_parameter<std::string>("unitree_network_interface", "");
    declare_parameter<int>("align_obstacle_fsm_id", 812);
    declare_parameter<int>("align_restore_fsm_id", 802);
    declare_parameter<bool>("restore_fsm_after_align", true);
    declare_parameter<bool>("require_unitree_align_mode", true);
    declare_parameter<bool>("align_use_continuous_gait", true);

    declare_parameter<double>("control_rate_hz", 20.0);
    declare_parameter<double>("nav_timeout_s", 25.0);
    declare_parameter<double>("nav_status_timeout_s", 1.0);
    declare_parameter<double>("align_timeout_s", 8.0);
    declare_parameter<double>("perception_timeout_s", 0.2);
    declare_parameter<double>("post_track_settle_s", 0.3);
    declare_parameter<double>("ball_perception_wait_timeout_s", 3.0);
    declare_parameter<bool>("kick_on_ball_perception_timeout", true);
    declare_parameter<double>("pre_kick_pause_s", 0.2);

    declare_parameter<double>("align_target_ball_x_m", 0.22);
    declare_parameter<double>("align_target_ball_y_m", 0.0);
    declare_parameter<bool>("enable_align", true);
    declare_parameter<double>("align_x_tolerance_m", 0.06);
    declare_parameter<double>("align_y_tolerance_m", 0.04);
    declare_parameter<double>("align_yaw_tolerance_rad", 0.08);
    declare_parameter<int>("align_required_stable_frames", 5);
    declare_parameter<bool>("kick_on_align_ball_lost_near_feet", true);
    declare_parameter<double>("align_lost_ball_kick_x_m", 0.45);
    declare_parameter<double>("align_lost_ball_kick_y_tolerance_m", 0.10);
    declare_parameter<double>("align_lost_ball_kick_yaw_tolerance_rad", 0.18);

    declare_parameter<double>("align_kx", 0.45);
    declare_parameter<double>("align_ky", 0.55);
    declare_parameter<double>("align_kw", 0.9);
    declare_parameter<double>("align_max_vx", 0.12);
    declare_parameter<double>("align_max_vy", 0.10);
    declare_parameter<double>("align_max_wz", 0.25);
    declare_parameter<double>("align_step_duration_s", 0.45);
    declare_parameter<double>("align_min_step_period_s", 0.15);

    declare_parameter<double>("min_ball_confidence", 0.45);
    declare_parameter<double>("min_goal_confidence", 0.35);
    declare_parameter<std::string>("kick_action_name", "PENALTY_KICK");
    declare_parameter<std::string>("kick_action_params_json", "");
    declare_parameter<std::string>("kick_model_path", "");
    declare_parameter<std::string>("kick_trajectory_path", "");
    declare_parameter<std::string>("kick_policy_type", "lingshu");
    declare_parameter<std::string>("kick_end_behavior", "switch_to_loco");
    declare_parameter<double>("kick_quat_comp", 0.0);
    declare_parameter<std::string>("kick_action_server", "/whole_body/action_ctrl");
    declare_parameter<bool>("enable_kick_action", true);
    declare_parameter<double>("kick_server_timeout_s", 2.0);
    declare_parameter<double>("kick_result_timeout_s", 8.0);
  }

  void loadParameters() {
    start_mode_ = get_parameter("start_mode").as_string();
    auto_start_ = get_parameter("auto_start").as_bool();
    goal_target_ = get_parameter("goal_target").as_double();
    nav_ball_distance_m_ = get_parameter("nav_ball_distance_m").as_double();

    unitree_network_interface_ =
        get_parameter("unitree_network_interface").as_string();
    align_obstacle_fsm_id_ =
        static_cast<int>(get_parameter("align_obstacle_fsm_id").as_int());
    align_restore_fsm_id_ =
        static_cast<int>(get_parameter("align_restore_fsm_id").as_int());
    restore_fsm_after_align_ =
        get_parameter("restore_fsm_after_align").as_bool();
    require_unitree_align_mode_ =
        get_parameter("require_unitree_align_mode").as_bool();
    align_use_continuous_gait_ =
        get_parameter("align_use_continuous_gait").as_bool();

    control_rate_hz_ = get_parameter("control_rate_hz").as_double();
    nav_timeout_s_ = get_parameter("nav_timeout_s").as_double();
    nav_status_timeout_s_ = get_parameter("nav_status_timeout_s").as_double();
    align_timeout_s_ = get_parameter("align_timeout_s").as_double();
    perception_timeout_s_ = get_parameter("perception_timeout_s").as_double();
    post_track_settle_s_ = get_parameter("post_track_settle_s").as_double();
    ball_perception_wait_timeout_s_ =
        get_parameter("ball_perception_wait_timeout_s").as_double();
    kick_on_ball_perception_timeout_ =
        get_parameter("kick_on_ball_perception_timeout").as_bool();
    pre_kick_pause_s_ = get_parameter("pre_kick_pause_s").as_double();

    align_target_ball_x_m_ =
        get_parameter("align_target_ball_x_m").as_double();
    align_target_ball_y_m_ =
        get_parameter("align_target_ball_y_m").as_double();
    enable_align_ = get_parameter("enable_align").as_bool();
    align_x_tolerance_m_ = get_parameter("align_x_tolerance_m").as_double();
    align_y_tolerance_m_ = get_parameter("align_y_tolerance_m").as_double();
    align_yaw_tolerance_rad_ =
        get_parameter("align_yaw_tolerance_rad").as_double();
    align_required_stable_frames_ =
        static_cast<int>(get_parameter("align_required_stable_frames").as_int());
    kick_on_align_ball_lost_near_feet_ =
        get_parameter("kick_on_align_ball_lost_near_feet").as_bool();
    align_lost_ball_kick_x_m_ =
        get_parameter("align_lost_ball_kick_x_m").as_double();
    align_lost_ball_kick_y_tolerance_m_ =
        get_parameter("align_lost_ball_kick_y_tolerance_m").as_double();
    align_lost_ball_kick_yaw_tolerance_rad_ =
        get_parameter("align_lost_ball_kick_yaw_tolerance_rad").as_double();

    align_kx_ = get_parameter("align_kx").as_double();
    align_ky_ = get_parameter("align_ky").as_double();
    align_kw_ = get_parameter("align_kw").as_double();
    align_max_vx_ = get_parameter("align_max_vx").as_double();
    align_max_vy_ = get_parameter("align_max_vy").as_double();
    align_max_wz_ = get_parameter("align_max_wz").as_double();
    align_step_duration_s_ =
        get_parameter("align_step_duration_s").as_double();
    align_min_step_period_s_ =
        get_parameter("align_min_step_period_s").as_double();

    min_ball_confidence_ = get_parameter("min_ball_confidence").as_double();
    min_goal_confidence_ = get_parameter("min_goal_confidence").as_double();
    kick_action_name_ = get_parameter("kick_action_name").as_string();
    kick_action_params_json_ =
        get_parameter("kick_action_params_json").as_string();
    kick_model_path_ = get_parameter("kick_model_path").as_string();
    kick_trajectory_path_ = get_parameter("kick_trajectory_path").as_string();
    kick_policy_type_ = get_parameter("kick_policy_type").as_string();
    kick_end_behavior_ = get_parameter("kick_end_behavior").as_string();
    kick_quat_comp_ = get_parameter("kick_quat_comp").as_double();
    kick_action_server_ = get_parameter("kick_action_server").as_string();
    enable_kick_action_ = get_parameter("enable_kick_action").as_bool();
    kick_server_timeout_s_ = get_parameter("kick_server_timeout_s").as_double();
    kick_result_timeout_s_ = get_parameter("kick_result_timeout_s").as_double();
  }

  void initUnitreeClient() {
    if (!unitree_network_interface_.empty()) {
      unitree::robot::ChannelFactory::Instance()->Init(
          0, unitree_network_interface_);
    } else {
      unitree::robot::ChannelFactory::Instance()->Init(0);
    }

    loco_client_ = std::make_unique<unitree::robot::g1::LocoClient>();
    loco_client_->Init();
  }

  void initRosIo() {
    const auto reliable_10 = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    const auto reliable_5 = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
    const auto transient_1 =
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    perception_sub_ = create_subscription<SoccerPerception>(
        "/soccer/perception", reliable_10,
        [this](SoccerPerception::SharedPtr msg) { onPerception(msg); });

    nav_status_sub_ = create_subscription<NavStatus>(
        "/soccer/nav_status", transient_1,
        [this](NavStatus::SharedPtr msg) { onNavStatus(msg); });

    game_mode_pub_ =
        create_publisher<GameModeCommand>("/soccer/game_mode_cmd", transient_1);
    vision_track_pub_ = create_publisher<VisionTrackCommand>(
        "/soccer/vision_track_cmd", reliable_5);
    behavior_pub_ =
        create_publisher<BehaviorState>("/soccer/behavior_state", transient_1);

    kick_client_ =
        rclcpp_action::create_client<ExecutionUnit>(this, kick_action_server_);

    const auto period =
        std::chrono::duration<double>(1.0 / std::max(1.0, control_rate_hz_));
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        [this]() { tick(); });
  }

  void onPerception(const SoccerPerception::SharedPtr msg) {
    last_perception_ = msg;
    last_perception_time_ = now();
  }

  void onNavStatus(const NavStatus::SharedPtr msg) {
    const bool changed =
        !last_nav_status_ ||
        last_nav_status_->mode != msg->mode ||
        last_nav_status_->nav_alive != msg->nav_alive ||
        last_nav_status_->perception_alive != msg->perception_alive ||
        last_nav_status_->navigating_to_point != msg->navigating_to_point ||
        last_nav_status_->target_reached != msg->target_reached ||
        last_nav_status_->detail != msg->detail;

    if (changed) {
      repeated_nav_status_count_ = 0;
      RCLCPP_INFO(
          get_logger(),
          "nav_status changed: mode=%s nav_alive=%s perception_alive=%s "
          "navigating_to_point=%s target_reached=%s detail=%s",
          msg->mode.c_str(),
          boolText(msg->nav_alive),
          boolText(msg->perception_alive),
          boolText(msg->navigating_to_point),
          boolText(msg->target_reached),
          msg->detail.c_str());
    } else {
      ++repeated_nav_status_count_;
      if (repeated_nav_status_count_ >= 10) {
        repeated_nav_status_count_ = 0;
        RCLCPP_INFO(
            get_logger(),
            "nav_status unchanged x10: mode=%s nav_alive=%s perception_alive=%s "
            "navigating_to_point=%s target_reached=%s detail=%s",
            msg->mode.c_str(),
            boolText(msg->nav_alive),
            boolText(msg->perception_alive),
            boolText(msg->navigating_to_point),
            boolText(msg->target_reached),
            msg->detail.c_str());
      }
    }

    last_nav_status_ = msg;
    last_nav_status_time_ = now();
  }

  void startPenaltyAttack() {
    if (start_mode_ != "PENALTY_ATTACK") {
      transitionToError("unsupported start_mode: " + start_mode_);
      return;
    }

    GameModeCommand cmd;
    cmd.header.stamp = now();
    cmd.mode = "PENALTY_ATTACK";
    cmd.goal_target = static_cast<float>(goal_target_);
    cmd.nav_ball_distance_m = static_cast<float>(nav_ball_distance_m_);
    cmd.reset_state = true;
    game_mode_pub_->publish(cmd);
    RCLCPP_INFO(
        get_logger(),
        "game_mode_cmd sent: mode=%s goal_target=%.3f nav_ball_distance_m=%.3f "
        "reset_state=%s",
        cmd.mode.c_str(), cmd.goal_target, cmd.nav_ball_distance_m,
        boolText(cmd.reset_state));

    transitionTo(State::NAVIGATE_TO_POINT, "sent PENALTY_ATTACK");
  }

  void tick() {
    switch (state_) {
      case State::IDLE:
        return;
      case State::NAVIGATE_TO_POINT:
        handleNavigateToPoint();
        return;
      case State::START_BALL_TRACK:
        handleStartBallTrack();
        return;
      case State::ALIGN:
        handleAlign();
        return;
      case State::STOP_BALL_TRACK:
        handleStopBallTrack();
        return;
      case State::READY_KICK:
        handleReadyKick();
        return;
      case State::KICK:
        handleKick();
        return;
      case State::FINISH:
      case State::ERROR:
        return;
    }
  }

  void handleNavigateToPoint() {
    if (elapsedInState() > nav_timeout_s_) {
      transitionToError("navigation timeout");
      return;
    }

    if (!last_nav_status_ || !last_nav_status_time_) {
      return;
    }

    if ((now() - *last_nav_status_time_).seconds() > nav_status_timeout_s_) {
      transitionToError("navigation status stale");
      return;
    }

    if (!last_nav_status_->nav_alive) {
      transitionToError("navigation module not alive");
      return;
    }

    if (last_nav_status_->target_reached) {
      transitionTo(State::START_BALL_TRACK, "target_reached");
    }
  }

  void handleStartBallTrack() {
    if (elapsedInState() >= post_track_settle_s_) {
      if (!enable_align_) {
        transitionTo(State::STOP_BALL_TRACK, "align disabled");
        return;
      }

      if (!validPerception()) {
        const double wait_timeout =
            std::max(post_track_settle_s_, ball_perception_wait_timeout_s_);
        if (elapsedInState() > wait_timeout) {
          if (kick_on_ball_perception_timeout_) {
            transitionTo(
                State::STOP_BALL_TRACK,
                "ball perception unavailable after track start; kick fallback");
          } else {
            transitionToError("ball perception unavailable after track start");
          }
        }
        return;
      }
      if (!enterObstacleSteppingMode()) {
        transitionToError("failed to enter obstacle stepping fsm");
        return;
      }
      transitionTo(State::ALIGN, "vision tracking ready");
    }
  }

  void handleAlign() {
    if (elapsedInState() > align_timeout_s_) {
      stopObstacleStep();
      restoreFsmAfterAlign();
      transitionToError("align timeout");
      return;
    }

    if (!validPerception()) {
      stable_align_frames_ = 0;
      stopObstacleStep();
      if (shouldKickAfterAlignBallLoss()) {
        restoreFsmAfterAlign();
        transitionTo(State::STOP_BALL_TRACK, "ball lost near feet; kick fallback");
      }
      return;
    }

    const auto command = computeAlignCommand(*last_perception_);
    last_valid_align_command_ = command;
    last_valid_align_ball_x_ = last_perception_->ball.point.x;
    publishBehavior(
        "ALIGN",
        "x_err=" + std::to_string(command.x_error) +
            " y_err=" + std::to_string(command.y_error) +
            " yaw_err=" + std::to_string(command.yaw_error));

    if (command.within_tolerance) {
      ++stable_align_frames_;
      stopObstacleStep();
      RCLCPP_INFO(
          get_logger(),
          "align within tolerance: stable_frames=%d/%d x_err=%.3f y_err=%.3f "
          "yaw_err=%.3f ball=(%.3f, %.3f, %.3f)",
          stable_align_frames_, align_required_stable_frames_, command.x_error,
          command.y_error, command.yaw_error, last_perception_->ball.point.x,
          last_perception_->ball.point.y, last_perception_->ball.point.z);
      if (stable_align_frames_ >= align_required_stable_frames_) {
        restoreFsmAfterAlign();
        transitionTo(State::STOP_BALL_TRACK, "align stable");
      }
      return;
    }

    stable_align_frames_ = 0;
    if (!readyForNextAlignCorrection()) {
      return;
    }

    if (sendObstacleStep(command.vx, command.vy, command.wz)) {
      RCLCPP_INFO(
          get_logger(),
          "align velocity sent: vx=%.3f vy=%.3f wz=%.3f duration=%.3f "
          "x_err=%.3f y_err=%.3f yaw_err=%.3f ball=(%.3f, %.3f, %.3f)",
          command.vx, command.vy, command.wz, align_step_duration_s_,
          command.x_error, command.y_error, command.yaw_error,
          last_perception_->ball.point.x, last_perception_->ball.point.y,
          last_perception_->ball.point.z);
    }
  }

  void handleStopBallTrack() {
    transitionTo(State::READY_KICK, "vision tracking stopped");
  }

  void handleReadyKick() {
    if (elapsedInState() >= pre_kick_pause_s_) {
      if (!enable_kick_action_) {
        transitionTo(State::FINISH, "kick action disabled");
        return;
      }
      transitionTo(State::KICK, "ready to kick");
    }
  }

  void handleKick() {
    if (kick_goal_sent_) {
      if (elapsedInState() > kick_result_timeout_s_) {
        transitionToError("kick action result timeout");
      }
      return;
    }
    kick_goal_sent_ = true;

    if (!sendKickGoal()) {
      transitionToError("failed to send kick action");
      return;
    }
  }

  bool enterObstacleSteppingMode() {
    if (!loco_client_) {
      return !require_unitree_align_mode_;
    }

    int current_fsm = -1;
    const int32_t get_ret = loco_client_->GetFsmId(current_fsm);
    if (get_ret == 0) {
      saved_fsm_id_ = current_fsm;
    } else {
      RCLCPP_WARN(get_logger(), "GetFsmId failed: %d", get_ret);
    }

    const int32_t ret = loco_client_->SetFsmId(align_obstacle_fsm_id_);
    if (ret != 0) {
      RCLCPP_ERROR(
          get_logger(), "SetFsmId(%d) for obstacle stepping failed: %d",
          align_obstacle_fsm_id_, ret);
      return !require_unitree_align_mode_;
    }

    obstacle_mode_active_ = true;
    RCLCPP_INFO(
        get_logger(), "align control mode sent: SetFsmId(%d) success",
        align_obstacle_fsm_id_);

    if (align_use_continuous_gait_) {
      const int32_t gait_ret = loco_client_->ContinuousGait(true);
      if (gait_ret != 0) {
        RCLCPP_WARN(
            get_logger(), "ContinuousGait(true) for ALIGN failed: %d",
            gait_ret);
        return !require_unitree_align_mode_;
      }
      continuous_gait_active_ = true;
      RCLCPP_INFO(
          get_logger(), "align continuous gait sent: ContinuousGait(true) success");
    }
    return true;
  }

  void restoreFsmAfterAlign() {
    if (!restore_fsm_after_align_ || !obstacle_mode_active_ || !loco_client_) {
      disableContinuousGait();
      obstacle_mode_active_ = false;
      return;
    }

    disableContinuousGait();

    const int restore_id = saved_fsm_id_.value_or(align_restore_fsm_id_);
    const int32_t ret = loco_client_->SetFsmId(restore_id);
    if (ret != 0) {
      RCLCPP_WARN(
          get_logger(), "SetFsmId(%d) restore after ALIGN failed: %d",
          restore_id, ret);
    } else {
      RCLCPP_INFO(
          get_logger(), "align control mode restore sent: SetFsmId(%d) success",
          restore_id);
    }
    obstacle_mode_active_ = false;
  }

  void disableContinuousGait() {
    if (!continuous_gait_active_ || !loco_client_) {
      continuous_gait_active_ = false;
      return;
    }

    const int32_t ret = loco_client_->ContinuousGait(false);
    if (ret != 0) {
      RCLCPP_WARN(
          get_logger(), "ContinuousGait(false) after ALIGN failed: %d", ret);
    } else {
      RCLCPP_INFO(
          get_logger(),
          "align continuous gait sent: ContinuousGait(false) success");
    }
    continuous_gait_active_ = false;
  }

  void stopObstacleStep() {
    if (!loco_client_ || (!obstacle_mode_active_ && !continuous_gait_active_)) {
      return;
    }
    const int32_t ret = loco_client_->SetVelocity(0.0f, 0.0f, 0.0f, 0.1f);
    if (ret != 0) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "SetVelocity stop failed during ALIGN: %d", ret);
    } else {
      RCLCPP_INFO(
          get_logger(),
          "align velocity sent: vx=0.000 vy=0.000 wz=0.000 duration=0.100 stop");
    }
  }

  bool sendObstacleStep(double vx, double vy, double wz) {
    if (!loco_client_) {
      return false;
    }

    const int32_t ret = loco_client_->SetVelocity(
        static_cast<float>(vx), static_cast<float>(vy),
        static_cast<float>(wz), static_cast<float>(align_step_duration_s_));
    if (ret != 0) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "SetVelocity step failed during ALIGN: %d", ret);
      return false;
    }

    last_step_time_ = now();
    last_step_perception_time_ = last_perception_time_;
    return true;
  }

  bool readyForNextAlignCorrection() const {
    if (!last_step_time_) {
      return true;
    }

    const double wait_s = align_step_duration_s_ + align_min_step_period_s_;
    if ((now() - *last_step_time_).seconds() < wait_s) {
      return false;
    }

    if (!last_step_perception_time_ || !last_perception_time_) {
      return true;
    }

    return (*last_perception_time_ - *last_step_perception_time_).seconds() >
           0.0;
  }

  AlignCommand computeAlignCommand(const SoccerPerception& perception) const {
    AlignCommand command;
    const auto& ball = perception.ball.point;

    command.x_error = ball.x - align_target_ball_x_m_;
    command.y_error = ball.y - align_target_ball_y_m_;
    command.yaw_error = computeYawError(perception);

    command.within_tolerance =
        std::abs(command.x_error) <= align_x_tolerance_m_ &&
        std::abs(command.y_error) <= align_y_tolerance_m_ &&
        std::abs(command.yaw_error) <= align_yaw_tolerance_rad_;

    command.vx = clampAbs(align_kx_ * command.x_error, align_max_vx_);
    command.vy = clampAbs(align_ky_ * command.y_error, align_max_vy_);
    command.wz = clampAbs(align_kw_ * command.yaw_error, align_max_wz_);
    return command;
  }

  bool shouldKickAfterAlignBallLoss() const {
    if (!kick_on_align_ball_lost_near_feet_ ||
        !last_valid_align_command_ ||
        !last_valid_align_ball_x_) {
      return false;
    }

    return *last_valid_align_ball_x_ <= align_lost_ball_kick_x_m_ &&
           std::abs(last_valid_align_command_->y_error) <=
               align_lost_ball_kick_y_tolerance_m_ &&
           std::abs(last_valid_align_command_->yaw_error) <=
               align_lost_ball_kick_yaw_tolerance_rad_;
  }

  double computeYawError(const SoccerPerception& perception) const {
    const auto& ball = perception.ball.point;
    if (perception.image_has_goal &&
        perception.goal_confidence >= min_goal_confidence_) {
      const auto& goal = perception.goal_center.point;
      return std::atan2(goal.y - ball.y, goal.x - ball.x);
    }
    return std::atan2(ball.y, std::max(0.05, ball.x));
  }

  bool validPerception() const {
    if (!last_perception_ || !last_perception_time_) {
      return false;
    }
    if ((now() - *last_perception_time_).seconds() > perception_timeout_s_) {
      return false;
    }
    return last_perception_->image_has_ball &&
           last_perception_->transform_valid &&
           last_perception_->ball_confidence >= min_ball_confidence_;
  }

  void publishVisionTrackCommand(const std::string& command) {
    VisionTrackCommand msg;
    msg.header.stamp = now();
    msg.command = command;
    vision_track_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "vision_track_cmd sent: %s", command.c_str());
  }

  void publishBehavior(const std::string& state, const std::string& detail) {
    BehaviorState msg;
    msg.header.stamp = now();
    msg.mode = mode_;
    msg.state = state;
    msg.detail = nowDetail(now().seconds(), detail);
    msg.progress = progressForState();
    msg.active = state_ != State::IDLE && state_ != State::FINISH &&
                 state_ != State::ERROR;
    behavior_pub_->publish(msg);
  }

  std::string navStatusSummary() const {
    if (!last_nav_status_ || !last_nav_status_time_) {
      return "nav_status=none";
    }

    std::ostringstream out;
    out << "nav_status_age=" << (now() - *last_nav_status_time_).seconds()
        << "s mode=" << last_nav_status_->mode
        << " nav_alive=" << boolText(last_nav_status_->nav_alive)
        << " perception_alive="
        << boolText(last_nav_status_->perception_alive)
        << " navigating_to_point="
        << boolText(last_nav_status_->navigating_to_point)
        << " target_reached=" << boolText(last_nav_status_->target_reached)
        << " detail=" << last_nav_status_->detail;
    return out.str();
  }

  std::string perceptionSummary() const {
    if (!last_perception_ || !last_perception_time_) {
      return "perception=none";
    }

    std::ostringstream out;
    out << "perception_age=" << (now() - *last_perception_time_).seconds()
        << "s image_has_ball=" << boolText(last_perception_->image_has_ball)
        << " transform_valid=" << boolText(last_perception_->transform_valid)
        << " ball_conf=" << last_perception_->ball_confidence
        << " ball=(" << last_perception_->ball.point.x
        << "," << last_perception_->ball.point.y
        << "," << last_perception_->ball.point.z
        << ") image_has_goal=" << boolText(last_perception_->image_has_goal)
        << " goal_conf=" << last_perception_->goal_confidence
        << " detail=" << last_perception_->detail;
    return out.str();
  }

  float progressForState() const {
    switch (state_) {
      case State::IDLE:
        return 0.0f;
      case State::NAVIGATE_TO_POINT:
        return 0.15f;
      case State::START_BALL_TRACK:
        return 0.35f;
      case State::ALIGN:
        return 0.55f;
      case State::STOP_BALL_TRACK:
        return 0.75f;
      case State::READY_KICK:
        return 0.85f;
      case State::KICK:
        return 0.95f;
      case State::FINISH:
        return 1.0f;
      case State::ERROR:
        return 0.0f;
    }
    return 0.0f;
  }

  bool sendKickGoal() {
    if (!kick_client_->wait_for_action_server(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(kick_server_timeout_s_)))) {
      RCLCPP_ERROR(
          get_logger(), "Kick action server unavailable: %s",
          kick_action_server_.c_str());
      return false;
    }

    auto goal = makeKickGoal();
    if (goal.params.empty()) {
      return false;
    }

    auto options = rclcpp_action::Client<ExecutionUnit>::SendGoalOptions();
    options.goal_response_callback =
        [this](const GoalHandleExecutionUnit::SharedPtr& handle) {
          if (!handle) {
            transitionToError("kick goal rejected");
          }
        };
    options.result_callback =
        [this](const GoalHandleExecutionUnit::WrappedResult& result) {
          if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
            transitionToError("kick action failed");
            return;
          }
          if (result.result &&
              result.result->result == ExecutionUnit::Result::SUCCESS) {
            transitionTo(State::FINISH, "kick succeeded");
          } else {
            const auto message =
                result.result ? result.result->error_message : "empty result";
            transitionToError("kick action failed: " + message);
          }
        };

    kick_client_->async_send_goal(goal, options);
    publishBehavior("KICK", "kick goal sent");
    return true;
  }

  ExecutionUnit::Goal makeKickGoal() const {
    ExecutionUnit::Goal goal;

    if (!kick_action_params_json_.empty()) {
      goal.params = kick_action_params_json_;
      return goal;
    }

    if (kick_model_path_.empty() || kick_trajectory_path_.empty()) {
      RCLCPP_ERROR(
          get_logger(),
          "kick_action_params_json is empty and kick_model_path/"
          "kick_trajectory_path are not both set. whole_body/action_ctrl "
          "expects a DynamicDanceManager JSON in ExecutionUnit.params.");
      return goal;
    }

    std::ostringstream params;
    params << "{"
           << "\"uuid\":\"" << jsonEscape(kick_action_name_) << "\","
           << "\"state_name\":\"" << jsonEscape(kick_action_name_) << "\","
           << "\"policy_type\":\"" << jsonEscape(kick_policy_type_) << "\","
           << "\"model\":\"" << jsonEscape(kick_model_path_) << "\","
           << "\"trajectory\":\"" << jsonEscape(kick_trajectory_path_) << "\","
           << "\"end_behavior\":\"" << jsonEscape(kick_end_behavior_) << "\","
           << "\"quat_comp\":" << kick_quat_comp_ << ","
           << "\"allowed_from\":[\"PASSIVE\",\"LOCO\",\"FIXEDSTAND\"]"
           << "}";
    goal.params = params.str();
    return goal;
  }

  void transitionTo(State next, const std::string& detail) {
    const auto previous = state_;
    const double previous_elapsed = elapsedInState();
    state_ = next;
    state_enter_time_ = now();
    stable_align_frames_ = 0;

    if (next == State::START_BALL_TRACK || next == State::ALIGN) {
      last_valid_align_command_.reset();
      last_valid_align_ball_x_.reset();
      last_step_time_.reset();
      last_step_perception_time_.reset();
    }

    if (next == State::NAVIGATE_TO_POINT) {
      mode_ = "PENALTY_ATTACK";
      kick_goal_sent_ = false;
    }
    if (next == State::START_BALL_TRACK) {
      publishVisionTrackCommand("START_BALL_TRACK");
    } else if (next == State::STOP_BALL_TRACK) {
      publishVisionTrackCommand("STOP_BALL_TRACK");
    }

    publishBehavior(stateName(next), detail);
    RCLCPP_INFO(
        get_logger(),
        "state transition: %s -> %s after %.3fs detail=%s | %s | %s",
        stateName(previous).c_str(), stateName(next).c_str(), previous_elapsed,
        detail.c_str(), navStatusSummary().c_str(), perceptionSummary().c_str());
  }

  void transitionToError(const std::string& detail) {
    const auto previous = state_;
    const double previous_elapsed = elapsedInState();
    stopObstacleStep();
    restoreFsmAfterAlign();
    state_ = State::ERROR;
    state_enter_time_ = now();
    publishVisionTrackCommand("STOP_BALL_TRACK");
    publishBehavior("ERROR", detail);
    RCLCPP_ERROR(
        get_logger(),
        "state transition: %s -> ERROR after %.3fs detail=%s | %s | %s",
        stateName(previous).c_str(), previous_elapsed, detail.c_str(),
        navStatusSummary().c_str(), perceptionSummary().c_str());
  }

  std::string stateName(State state) const {
    switch (state) {
      case State::IDLE:
        return "IDLE";
      case State::NAVIGATE_TO_POINT:
        return "NAVIGATE_TO_POINT";
      case State::START_BALL_TRACK:
        return "START_BALL_TRACK";
      case State::ALIGN:
        return "ALIGN";
      case State::STOP_BALL_TRACK:
        return "STOP_BALL_TRACK";
      case State::READY_KICK:
        return "READY_KICK";
      case State::KICK:
        return "KICK";
      case State::FINISH:
        return "FINISH";
      case State::ERROR:
        return "ERROR";
    }
    return "UNKNOWN";
  }

  double elapsedInState() const {
    return (now() - state_enter_time_).seconds();
  }

  std::string start_mode_;
  bool auto_start_ = true;
  std::string mode_ = "IDLE";

  double goal_target_ = 0.0;
  double nav_ball_distance_m_ = 0.75;

  std::string unitree_network_interface_;
  int align_obstacle_fsm_id_ = 812;
  int align_restore_fsm_id_ = 802;
  bool restore_fsm_after_align_ = true;
  bool require_unitree_align_mode_ = true;
  bool align_use_continuous_gait_ = true;

  double control_rate_hz_ = 20.0;
  double nav_timeout_s_ = 25.0;
  double nav_status_timeout_s_ = 1.0;
  double align_timeout_s_ = 8.0;
  double perception_timeout_s_ = 0.2;
  double post_track_settle_s_ = 0.3;
  double ball_perception_wait_timeout_s_ = 3.0;
  bool kick_on_ball_perception_timeout_ = true;
  double pre_kick_pause_s_ = 0.2;

  double align_target_ball_x_m_ = 0.22;
  double align_target_ball_y_m_ = 0.0;
  bool enable_align_ = true;
  double align_x_tolerance_m_ = 0.06;
  double align_y_tolerance_m_ = 0.04;
  double align_yaw_tolerance_rad_ = 0.08;
  int align_required_stable_frames_ = 5;
  bool kick_on_align_ball_lost_near_feet_ = true;
  double align_lost_ball_kick_x_m_ = 0.45;
  double align_lost_ball_kick_y_tolerance_m_ = 0.10;
  double align_lost_ball_kick_yaw_tolerance_rad_ = 0.18;

  double align_kx_ = 0.45;
  double align_ky_ = 0.55;
  double align_kw_ = 0.9;
  double align_max_vx_ = 0.12;
  double align_max_vy_ = 0.10;
  double align_max_wz_ = 0.25;
  double align_step_duration_s_ = 0.45;
  double align_min_step_period_s_ = 0.15;

  double min_ball_confidence_ = 0.45;
  double min_goal_confidence_ = 0.35;
  std::string kick_action_name_ = "PENALTY_KICK";
  std::string kick_action_params_json_;
  std::string kick_model_path_;
  std::string kick_trajectory_path_;
  std::string kick_policy_type_ = "lingshu";
  std::string kick_end_behavior_ = "switch_to_loco";
  double kick_quat_comp_ = 0.0;
  std::string kick_action_server_ = "/whole_body/action_ctrl";
  bool enable_kick_action_ = true;
  double kick_server_timeout_s_ = 2.0;
  double kick_result_timeout_s_ = 8.0;

  State state_ = State::IDLE;
  rclcpp::Time state_enter_time_;
  std::optional<rclcpp::Time> last_step_time_;
  std::optional<rclcpp::Time> last_step_perception_time_;
  std::optional<AlignCommand> last_valid_align_command_;
  std::optional<double> last_valid_align_ball_x_;
  int stable_align_frames_ = 0;
  bool obstacle_mode_active_ = false;
  bool continuous_gait_active_ = false;
  bool kick_goal_sent_ = false;
  std::optional<int> saved_fsm_id_;

  std::unique_ptr<unitree::robot::g1::LocoClient> loco_client_;

  rclcpp::Subscription<SoccerPerception>::SharedPtr perception_sub_;
  rclcpp::Subscription<NavStatus>::SharedPtr nav_status_sub_;
  rclcpp::Publisher<GameModeCommand>::SharedPtr game_mode_pub_;
  rclcpp::Publisher<VisionTrackCommand>::SharedPtr vision_track_pub_;
  rclcpp::Publisher<BehaviorState>::SharedPtr behavior_pub_;
  rclcpp_action::Client<ExecutionUnit>::SharedPtr kick_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  SoccerPerception::SharedPtr last_perception_;
  NavStatus::SharedPtr last_nav_status_;
  std::optional<rclcpp::Time> last_perception_time_;
  std::optional<rclcpp::Time> last_nav_status_time_;
  int repeated_nav_status_count_ = 0;
};

}  // namespace mwc_soccer

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mwc_soccer::SoccerBrainNode>(
      rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}
