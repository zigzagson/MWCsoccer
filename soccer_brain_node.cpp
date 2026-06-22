#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <message_center/action/execution_unit.hpp>
#include <soccer_msgs/msg/behavior_state.hpp>
#include <soccer_msgs/msg/game_mode_command.hpp>
#include <soccer_msgs/msg/nav_status.hpp>
#include <soccer_msgs/msg/soccer_perception.hpp>
#include <soccer_msgs/msg/vision_track_command.hpp>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/dds_wrapper/common/unitree_joystick.hpp>
#include <unitree/robot/g1/loco/g1_loco_api.hpp>
#include <unitree/robot/g1/loco/g1_loco_client.hpp>

using namespace std::chrono_literals;

namespace mwc_soccer {

namespace {

double correctionVelocity(
    double error,
    double tolerance,
    double gain,
    double min_speed,
    double max_speed) {
  if (std::abs(error) <= std::abs(tolerance)) {
    return 0.0;
  }

  const double max_magnitude = std::abs(max_speed);
  const double min_magnitude =
      std::min(std::abs(min_speed), max_magnitude);
  const double raw_velocity = gain * error;
  const double magnitude = std::clamp(
      std::abs(raw_velocity), min_magnitude, max_magnitude);
  return std::copysign(
      magnitude, raw_velocity != 0.0 ? raw_velocity : error);
}

std::string nowDetail(double timestamp_seconds, const std::string& detail) {
  return detail + " t=" + std::to_string(timestamp_seconds);
}

const char* boolText(bool value) {
  return value ? "true" : "false";
}

const char* resultCodeName(rclcpp_action::ResultCode code) {
  switch (code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      return "SUCCEEDED";
    case rclcpp_action::ResultCode::ABORTED:
      return "ABORTED";
    case rclcpp_action::ResultCode::CANCELED:
      return "CANCELED";
    case rclcpp_action::ResultCode::UNKNOWN:
      return "UNKNOWN";
  }
  return "UNRECOGNIZED";
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
      startConfiguredMode();
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
    CELEBRATE,
    TRACK_BALL,
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

  enum class RemoteRequest : int {
    NONE = 0,
    IDLE,
    PENALTY_DEFAULT,
    GOALKEEPER_DEFAULT,
    PENALTY_LEFT,
    PENALTY_CENTER,
    PENALTY_RIGHT,
    SELECT_NEXT_CELEBRATION,
    CELEBRATE,
  };

  struct PenaltyProfile {
    double goal_target = 0.0;
    double nav_ball_distance_m = 0.8;
    double align_target_ball_x_m = 0.8;
    double align_target_ball_y_m = -0.3;
  };

  struct GoalkeeperProfile {
    double lateral_kp = 3.0;
    double min_lateral_speed = 0.8;
    double lateral_speed = 1.0;
    double center_deadband_x = 0.05;
  };

  struct CelebrationAction {
    std::string id;
    std::string action_name;
    std::string model_path;
    std::string trajectory_path;
  };

  void declareParameters() {
    declare_parameter<std::string>("start_mode", "PENALTY_ATTACK");
    declare_parameter<bool>("auto_start", true);
    declare_parameter<double>("goal_target", 0.0);
    declare_parameter<double>("nav_ball_distance_m", 0.75);
    declare_parameter<bool>("remote_mode_control_enabled", true);
    declare_parameter<double>("remote_combo_hold_s", 0.45);
    declare_parameter<double>("remote_combo_cooldown_s", 1.0);

    declare_parameter<double>("profiles.penalty_left.goal_target", -0.65);
    declare_parameter<double>(
        "profiles.penalty_left.nav_ball_distance_m", 0.8);
    declare_parameter<double>(
        "profiles.penalty_left.align_target_ball_x_m", 0.8);
    declare_parameter<double>(
        "profiles.penalty_left.align_target_ball_y_m", -0.3);
    declare_parameter<double>("profiles.penalty_center.goal_target", 0.0);
    declare_parameter<double>(
        "profiles.penalty_center.nav_ball_distance_m", 0.8);
    declare_parameter<double>(
        "profiles.penalty_center.align_target_ball_x_m", 0.8);
    declare_parameter<double>(
        "profiles.penalty_center.align_target_ball_y_m", -0.3);
    declare_parameter<double>("profiles.penalty_right.goal_target", 0.65);
    declare_parameter<double>(
        "profiles.penalty_right.nav_ball_distance_m", 0.8);
    declare_parameter<double>(
        "profiles.penalty_right.align_target_ball_x_m", 0.8);
    declare_parameter<double>(
        "profiles.penalty_right.align_target_ball_y_m", -0.3);
    declare_parameter<double>(
        "profiles.goalkeeper_default.lateral_kp", 3.0);
    declare_parameter<double>(
        "profiles.goalkeeper_default.min_lateral_speed", 0.8);
    declare_parameter<double>(
        "profiles.goalkeeper_default.lateral_speed", 1.0);
    declare_parameter<double>(
        "profiles.goalkeeper_default.center_deadband_x", 0.05);

    declare_parameter<std::string>("unitree_network_interface", "");
    declare_parameter<std::string>("velocity_command_topic", "/nav/cmd_vel_nav");
    declare_parameter<int>("align_obstacle_fsm_id", 812);
    declare_parameter<int>("align_restore_fsm_id", 802);
    declare_parameter<bool>("restore_fsm_after_align", true);
    declare_parameter<bool>("require_unitree_align_mode", true);
    declare_parameter<bool>("align_use_continuous_gait", false);

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
    declare_parameter<bool>("kick_on_align_timeout", true);

    declare_parameter<double>("align_kx", 0.45);
    declare_parameter<double>("align_ky", 0.55);
    declare_parameter<double>("align_kw", 0.9);
    declare_parameter<double>("align_max_vx", 0.50);
    declare_parameter<double>("align_max_vy", 0.50);
    declare_parameter<double>("align_max_wz", 0.50);
    declare_parameter<double>("align_min_speed", 0.20);
    declare_parameter<double>("align_step_duration_s", 1.00);
    declare_parameter<double>("align_min_step_period_s", 0.15);
    declare_parameter<bool>("align_require_standing_for_sample", true);

    declare_parameter<double>("goalkeeper_min_ball_confidence", 0.30);
    declare_parameter<bool>("goalkeeper_enable_motion", true);
    declare_parameter<double>("goalkeeper_center_deadband_x", 0.05);
    declare_parameter<double>("goalkeeper_lateral_kp", 2.0);
    declare_parameter<double>("goalkeeper_min_lateral_speed", 0.20);
    declare_parameter<double>("goalkeeper_lateral_speed", 1.00);
    declare_parameter<double>("goalkeeper_lateral_sign", 1.0);
    declare_parameter<double>("goalkeeper_perception_timeout_s", 1.00);

    declare_parameter<double>("min_ball_confidence", 0.45);
    declare_parameter<double>("min_goal_confidence", 0.35);
    declare_parameter<std::string>("kick_action_name", "PENALTY_KICK");
    declare_parameter<std::string>("kick_action_params_json", "");
    declare_parameter<std::string>("kick_model_path", "");
    declare_parameter<std::string>("kick_trajectory_path", "");
    declare_parameter<std::string>("kick_policy_type", "lingshu");
    declare_parameter<std::string>("kick_vendor_name_en", "");
    declare_parameter<std::string>("kick_vendor_name_cn", "");
    declare_parameter<std::string>("kick_end_behavior", "switch_to_loco");
    declare_parameter<double>("kick_quat_comp", -0.2);
    declare_parameter<std::string>("kick_action_server", "/whole_body/action_ctrl");
    declare_parameter<bool>("enable_kick_action", true);
    declare_parameter<double>("kick_server_timeout_s", 2.0);
    declare_parameter<double>("kick_result_timeout_s", 8.0);

    declare_parameter<std::vector<std::string>>(
        "celebration_action_ids",
        {"neymar_victory_dance", "forward_jump", "raised_hand_taunt",
         "stretch_wave"});
    declare_parameter<std::vector<std::string>>(
        "celebration_action_names",
        {"NEYMAR_VICTORY_DANCE", "FORWARD_JUMP", "RAISED_HAND_TAUNT",
         "STRETCH_WAVE"});
    declare_parameter<std::string>("celebration_action_params_json", "");
    declare_parameter<std::vector<std::string>>(
        "celebration_model_paths", {"", "", "", ""});
    declare_parameter<std::vector<std::string>>(
        "celebration_trajectory_paths", {"", "", "", ""});
    declare_parameter<std::string>("celebration_policy_type", "lingshu");
    declare_parameter<std::string>("celebration_vendor_name_en", "");
    declare_parameter<std::string>("celebration_vendor_name_cn", "");
    declare_parameter<std::string>(
        "celebration_end_behavior", "switch_to_loco");
    declare_parameter<double>("celebration_quat_comp", -0.2);
    declare_parameter<std::string>(
        "celebration_action_server", "/whole_body/action_ctrl");
    declare_parameter<bool>("enable_celebration_action", true);
    declare_parameter<double>("celebration_server_timeout_s", 2.0);
    declare_parameter<double>("celebration_result_timeout_s", 15.0);
  }

  void loadParameters() {
    start_mode_ = get_parameter("start_mode").as_string();
    auto_start_ = get_parameter("auto_start").as_bool();
    goal_target_ = get_parameter("goal_target").as_double();
    nav_ball_distance_m_ = get_parameter("nav_ball_distance_m").as_double();
    penalty_default_profile_.goal_target = goal_target_;
    penalty_default_profile_.nav_ball_distance_m = nav_ball_distance_m_;
    remote_mode_control_enabled_ =
        get_parameter("remote_mode_control_enabled").as_bool();
    remote_combo_hold_s_ = get_parameter("remote_combo_hold_s").as_double();
    remote_combo_cooldown_s_ =
        get_parameter("remote_combo_cooldown_s").as_double();

    penalty_left_profile_ = loadPenaltyProfile("penalty_left");
    penalty_center_profile_ = loadPenaltyProfile("penalty_center");
    penalty_right_profile_ = loadPenaltyProfile("penalty_right");
    goalkeeper_default_profile_.lateral_kp =
        get_parameter("profiles.goalkeeper_default.lateral_kp").as_double();
    goalkeeper_default_profile_.min_lateral_speed =
        get_parameter(
            "profiles.goalkeeper_default.min_lateral_speed").as_double();
    goalkeeper_default_profile_.lateral_speed =
        get_parameter(
            "profiles.goalkeeper_default.lateral_speed").as_double();
    goalkeeper_default_profile_.center_deadband_x =
        get_parameter(
            "profiles.goalkeeper_default.center_deadband_x").as_double();

    unitree_network_interface_ =
        get_parameter("unitree_network_interface").as_string();
    velocity_command_topic_ =
        get_parameter("velocity_command_topic").as_string();
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
    penalty_default_profile_.align_target_ball_x_m =
        align_target_ball_x_m_;
    penalty_default_profile_.align_target_ball_y_m =
        align_target_ball_y_m_;
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
    kick_on_align_timeout_ = get_parameter("kick_on_align_timeout").as_bool();

    align_kx_ = get_parameter("align_kx").as_double();
    align_ky_ = get_parameter("align_ky").as_double();
    align_kw_ = get_parameter("align_kw").as_double();
    align_max_vx_ = get_parameter("align_max_vx").as_double();
    align_max_vy_ = get_parameter("align_max_vy").as_double();
    align_max_wz_ = get_parameter("align_max_wz").as_double();
    align_min_speed_ = get_parameter("align_min_speed").as_double();
    align_step_duration_s_ =
        get_parameter("align_step_duration_s").as_double();
    align_min_step_period_s_ =
        get_parameter("align_min_step_period_s").as_double();
    align_require_standing_for_sample_ =
        get_parameter("align_require_standing_for_sample").as_bool();

    goalkeeper_min_ball_confidence_ =
        get_parameter("goalkeeper_min_ball_confidence").as_double();
    goalkeeper_enable_motion_ =
        get_parameter("goalkeeper_enable_motion").as_bool();
    goalkeeper_center_deadband_x_ =
        get_parameter("goalkeeper_center_deadband_x").as_double();
    goalkeeper_lateral_kp_ =
        get_parameter("goalkeeper_lateral_kp").as_double();
    goalkeeper_min_lateral_speed_ =
        get_parameter("goalkeeper_min_lateral_speed").as_double();
    goalkeeper_lateral_speed_ =
        get_parameter("goalkeeper_lateral_speed").as_double();
    goalkeeper_lateral_sign_ =
        get_parameter("goalkeeper_lateral_sign").as_double();
    goalkeeper_perception_timeout_s_ =
        get_parameter("goalkeeper_perception_timeout_s").as_double();

    min_ball_confidence_ = get_parameter("min_ball_confidence").as_double();
    min_goal_confidence_ = get_parameter("min_goal_confidence").as_double();
    kick_action_name_ = get_parameter("kick_action_name").as_string();
    kick_action_params_json_ =
        get_parameter("kick_action_params_json").as_string();
    kick_model_path_ = get_parameter("kick_model_path").as_string();
    kick_trajectory_path_ = get_parameter("kick_trajectory_path").as_string();
    kick_policy_type_ = get_parameter("kick_policy_type").as_string();
    kick_vendor_name_en_ =
        get_parameter("kick_vendor_name_en").as_string();
    kick_vendor_name_cn_ =
        get_parameter("kick_vendor_name_cn").as_string();
    kick_end_behavior_ = get_parameter("kick_end_behavior").as_string();
    kick_quat_comp_ = get_parameter("kick_quat_comp").as_double();
    kick_action_server_ = get_parameter("kick_action_server").as_string();
    enable_kick_action_ = get_parameter("enable_kick_action").as_bool();
    kick_server_timeout_s_ = get_parameter("kick_server_timeout_s").as_double();
    kick_result_timeout_s_ = get_parameter("kick_result_timeout_s").as_double();

    celebration_action_params_json_ =
        get_parameter("celebration_action_params_json").as_string();
    const auto celebration_ids =
        get_parameter("celebration_action_ids").as_string_array();
    const auto celebration_names =
        get_parameter("celebration_action_names").as_string_array();
    const auto celebration_models =
        get_parameter("celebration_model_paths").as_string_array();
    const auto celebration_trajectories =
        get_parameter("celebration_trajectory_paths").as_string_array();
    const size_t celebration_count = celebration_ids.size();
    if (celebration_count == 0 ||
        celebration_names.size() != celebration_count ||
        celebration_models.size() != celebration_count ||
        celebration_trajectories.size() != celebration_count) {
      throw std::runtime_error(
          "celebration action id/name/model/trajectory arrays must be "
          "non-empty and have equal lengths");
    }
    celebration_actions_.clear();
    celebration_actions_.reserve(celebration_count);
    for (size_t i = 0; i < celebration_count; ++i) {
      celebration_actions_.push_back(
          {celebration_ids[i], celebration_names[i], celebration_models[i],
           celebration_trajectories[i]});
    }
    celebration_policy_type_ =
        get_parameter("celebration_policy_type").as_string();
    celebration_vendor_name_en_ =
        get_parameter("celebration_vendor_name_en").as_string();
    celebration_vendor_name_cn_ =
        get_parameter("celebration_vendor_name_cn").as_string();
    celebration_end_behavior_ =
        get_parameter("celebration_end_behavior").as_string();
    celebration_quat_comp_ =
        get_parameter("celebration_quat_comp").as_double();
    celebration_action_server_ =
        get_parameter("celebration_action_server").as_string();
    enable_celebration_action_ =
        get_parameter("enable_celebration_action").as_bool();
    celebration_server_timeout_s_ =
        get_parameter("celebration_server_timeout_s").as_double();
    celebration_result_timeout_s_ =
        get_parameter("celebration_result_timeout_s").as_double();
  }

  PenaltyProfile loadPenaltyProfile(const std::string& name) const {
    const std::string prefix = "profiles." + name + ".";
    PenaltyProfile profile;
    profile.goal_target =
        get_parameter(prefix + "goal_target").as_double();
    profile.nav_ball_distance_m =
        get_parameter(prefix + "nav_ball_distance_m").as_double();
    profile.align_target_ball_x_m =
        get_parameter(prefix + "align_target_ball_x_m").as_double();
    profile.align_target_ball_y_m =
        get_parameter(prefix + "align_target_ball_y_m").as_double();
    return profile;
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

    if (remote_mode_control_enabled_) {
      low_state_subscriber_ = std::make_shared<
          unitree::robot::ChannelSubscriber<
              unitree_hg::msg::dds_::LowState_>>("rt/lowstate");
      low_state_subscriber_->InitChannel(
          [this](const void* message) { onLowState(message); });
      RCLCPP_INFO(
          get_logger(),
          "remote mode control enabled on rt/lowstate: "
          "L1+R1+A=IDLE, L1+R1+X=PENALTY, L1+R1+Y=GOALKEEPER, "
          "L1+R1+DOWN=select celebration, "
          "L1+R1+B=run celebration (IDLE only), "
          "L1+R1+LEFT/UP/RIGHT=penalty profiles");
    }
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
    velocity_command_pub_ =
        create_publisher<geometry_msgs::msg::Twist>(
            velocity_command_topic_, reliable_10);

    kick_client_ =
        rclcpp_action::create_client<ExecutionUnit>(this, kick_action_server_);
    celebration_client_ = rclcpp_action::create_client<ExecutionUnit>(
        this, celebration_action_server_);

    const auto period =
        std::chrono::duration<double>(1.0 / std::max(1.0, control_rate_hz_));
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        [this]() { tick(); });
  }

  void onPerception(const SoccerPerception::SharedPtr msg) {
    last_perception_ = msg;
    last_perception_time_ = now();
    if (mode_ == "GOALKEEPER" &&
        (!msg->image_has_ball || !msg->transform_valid ||
         msg->ball_confidence < goalkeeper_min_ball_confidence_)) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "goalkeeper perception unavailable: image_has_ball=%s "
          "transform_valid=%s confidence=%.3f required=%.3f detail=%s",
          boolText(msg->image_has_ball), boolText(msg->transform_valid),
          msg->ball_confidence,
          goalkeeper_min_ball_confidence_, msg->detail.c_str());
    }
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

  void onLowState(const void* message) {
    const auto* state =
        static_cast<const unitree_hg::msg::dds_::LowState_*>(message);
    if (!state || state->wireless_remote().size() < 40) {
      return;
    }

    unitree::common::REMOTE_DATA_RX remote{};
    std::memcpy(
        remote.buff, state->wireless_remote().data(), sizeof(remote.buff));
    const auto& buttons = remote.RF_RX.btn.components;

    RemoteRequest candidate = RemoteRequest::NONE;
    if (buttons.L1 && buttons.R1) {
      if (buttons.A) {
        candidate = RemoteRequest::IDLE;
      } else if (buttons.X) {
        candidate = RemoteRequest::PENALTY_DEFAULT;
      } else if (buttons.Y) {
        candidate = RemoteRequest::GOALKEEPER_DEFAULT;
      } else if (buttons.B) {
        candidate = RemoteRequest::CELEBRATE;
      } else if (buttons.down) {
        candidate = RemoteRequest::SELECT_NEXT_CELEBRATION;
      } else if (buttons.left) {
        candidate = RemoteRequest::PENALTY_LEFT;
      } else if (buttons.up) {
        candidate = RemoteRequest::PENALTY_CENTER;
      } else if (buttons.right) {
        candidate = RemoteRequest::PENALTY_RIGHT;
      }
    }

    const auto steady_now = std::chrono::steady_clock::now();
    if (candidate == RemoteRequest::NONE) {
      remote_candidate_ = RemoteRequest::NONE;
      remote_candidate_fired_ = false;
      return;
    }
    if (candidate != remote_candidate_) {
      remote_candidate_ = candidate;
      remote_candidate_since_ = steady_now;
      remote_candidate_fired_ = false;
      return;
    }
    if (remote_candidate_fired_) {
      return;
    }

    const double held_s =
        std::chrono::duration<double>(
            steady_now - remote_candidate_since_).count();
    const bool cooldown_ready =
        !last_remote_trigger_time_ ||
        std::chrono::duration<double>(
            steady_now - *last_remote_trigger_time_).count() >=
            remote_combo_cooldown_s_;
    if (held_s < remote_combo_hold_s_ || !cooldown_ready) {
      return;
    }

    int expected = static_cast<int>(RemoteRequest::NONE);
    if (pending_remote_request_.compare_exchange_strong(
            expected, static_cast<int>(candidate))) {
      remote_candidate_fired_ = true;
      last_remote_trigger_time_ = steady_now;
    }
  }

  void startConfiguredMode() {
    if (start_mode_ == "IDLE") {
      enterIdle("configured startup");
      return;
    }
    if (start_mode_ == "PENALTY_ATTACK") {
      startPenaltyAttack();
      return;
    }
    if (start_mode_ == "GOALKEEPER") {
      startGoalkeeper();
      return;
    }
    transitionToError("unsupported start_mode: " + start_mode_);
  }

  void applyPenaltyProfile(
      const PenaltyProfile& profile, const std::string& name) {
    goal_target_ = profile.goal_target;
    nav_ball_distance_m_ = profile.nav_ball_distance_m;
    align_target_ball_x_m_ = profile.align_target_ball_x_m;
    align_target_ball_y_m_ = profile.align_target_ball_y_m;
    active_profile_ = name;
  }

  void applyGoalkeeperProfile(
      const GoalkeeperProfile& profile, const std::string& name) {
    goalkeeper_lateral_kp_ = profile.lateral_kp;
    goalkeeper_min_lateral_speed_ = profile.min_lateral_speed;
    goalkeeper_lateral_speed_ = profile.lateral_speed;
    goalkeeper_center_deadband_x_ = profile.center_deadband_x;
    active_profile_ = name;
  }

  void enterIdle(const std::string& reason) {
    ++mode_generation_;
    publishVelocityCommand(0.0, 0.0, 0.0, true);
    publishVisionTrackCommand("STOP_BALL_TRACK");
    stopObstacleStep();
    if (!restoreFsmAfterAlign()) {
      RCLCPP_WARN(get_logger(), "failed to restore FSM while entering IDLE");
    }
    if (kick_goal_handle_) {
      kick_client_->async_cancel_goal(kick_goal_handle_);
      kick_goal_handle_.reset();
    }
    if (celebration_goal_handle_) {
      celebration_client_->async_cancel_goal(celebration_goal_handle_);
      celebration_goal_handle_.reset();
    }

    state_ = State::IDLE;
    mode_ = "IDLE";
    state_enter_time_ = now();
    active_profile_.clear();
    stable_align_frames_ = 0;
    kick_goal_sent_ = false;
    celebration_goal_sent_ = false;
    goalkeeper_motion_active_ = false;
    goalkeeper_invalid_since_.reset();
    last_goalkeeper_processed_perception_time_.reset();
    last_valid_align_command_.reset();
    last_valid_align_ball_x_.reset();
    last_step_time_.reset();
    last_step_perception_time_.reset();
    last_processed_align_perception_time_.reset();
    align_stand_confirmed_time_.reset();
    align_stand_perception_time_.reset();
    last_logged_align_lost_perception_time_.reset();
    publishGameMode("IDLE");
    publishBehavior("IDLE", reason);
    RCLCPP_WARN(get_logger(), "soccer mode entered IDLE: %s", reason.c_str());
  }

  void switchMode(RemoteRequest request) {
    if (request == RemoteRequest::SELECT_NEXT_CELEBRATION) {
      if (state_ != State::IDLE) {
        RCLCPP_WARN(
            get_logger(),
            "celebration selection ignored: current state is %s, IDLE required",
            stateName(state_).c_str());
        return;
      }
      selectNextCelebration();
      return;
    }
    if (request == RemoteRequest::CELEBRATE) {
      if (state_ != State::IDLE) {
        RCLCPP_WARN(
            get_logger(),
            "celebration shortcut ignored: current state is %s, IDLE required",
            stateName(state_).c_str());
        return;
      }
      startCelebration();
      return;
    }

    enterIdle("remote mode switch");
    switch (request) {
      case RemoteRequest::IDLE:
      case RemoteRequest::NONE:
        return;
      case RemoteRequest::PENALTY_DEFAULT:
        applyPenaltyProfile(penalty_default_profile_, "penalty_default");
        startPenaltyAttack();
        return;
      case RemoteRequest::GOALKEEPER_DEFAULT:
        applyGoalkeeperProfile(
            goalkeeper_default_profile_, "goalkeeper_default");
        startGoalkeeper();
        return;
      case RemoteRequest::PENALTY_LEFT:
        applyPenaltyProfile(penalty_left_profile_, "penalty_left");
        startPenaltyAttack();
        return;
      case RemoteRequest::PENALTY_CENTER:
        applyPenaltyProfile(penalty_center_profile_, "penalty_center");
        startPenaltyAttack();
        return;
      case RemoteRequest::PENALTY_RIGHT:
        applyPenaltyProfile(penalty_right_profile_, "penalty_right");
        startPenaltyAttack();
        return;
      case RemoteRequest::SELECT_NEXT_CELEBRATION:
      case RemoteRequest::CELEBRATE:
        return;
    }
  }

  bool consumeRemoteRequest() {
    const int raw = pending_remote_request_.exchange(
        static_cast<int>(RemoteRequest::NONE));
    const auto request = static_cast<RemoteRequest>(raw);
    if (request == RemoteRequest::NONE) {
      return false;
    }
    switchMode(request);
    return true;
  }

  void publishGameMode(const std::string& mode) {
    GameModeCommand cmd;
    cmd.header.stamp = now();
    cmd.mode = mode;
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
  }

  void startPenaltyAttack() {
    mode_ = "PENALTY_ATTACK";
    publishGameMode(mode_);
    transitionTo(State::NAVIGATE_TO_POINT, "sent PENALTY_ATTACK");
  }

  void startGoalkeeper() {
    mode_ = "GOALKEEPER";
    last_goalkeeper_processed_perception_time_.reset();
    goalkeeper_invalid_since_.reset();
    goalkeeper_motion_active_ = false;
    publishGameMode(mode_);
    transitionTo(State::TRACK_BALL, "sent GOALKEEPER");
  }

  void startCelebration() {
    if (!enable_celebration_action_) {
      RCLCPP_WARN(get_logger(), "celebration action is disabled");
      return;
    }
    ++mode_generation_;
    mode_ = "CELEBRATION";
    celebration_goal_sent_ = false;
    transitionTo(State::CELEBRATE, "remote celebration shortcut");
  }

  void selectNextCelebration() {
    selected_celebration_index_ =
        (selected_celebration_index_ + 1) % celebration_actions_.size();
    const auto& action = selectedCelebration();
    publishBehavior(
        "IDLE", "celebration selection changed to " + action.id);
    RCLCPP_INFO(
        get_logger(), "celebration selected: %s (%zu/%zu)",
        action.id.c_str(), selected_celebration_index_ + 1,
        celebration_actions_.size());
  }

  const CelebrationAction& selectedCelebration() const {
    return celebration_actions_.at(selected_celebration_index_);
  }

  void tick() {
    if (consumeRemoteRequest()) {
      return;
    }
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
      case State::CELEBRATE:
        handleCelebration();
        return;
      case State::TRACK_BALL:
        handleTrackBall();
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
        RCLCPP_INFO(
            get_logger(),
            "align skipped: enable_align=false | %s",
            perceptionSummary().c_str());
        transitionTo(State::STOP_BALL_TRACK, "align disabled");
        return;
      }

      if (!validPerception()) {
        const double wait_timeout =
            std::max(post_track_settle_s_, ball_perception_wait_timeout_s_);
        if (elapsedInState() > wait_timeout) {
          if (kick_on_ball_perception_timeout_) {
            RCLCPP_WARN(
                get_logger(),
                "align skipped: no valid ball perception after %.3fs "
                "wait_timeout=%.3fs | %s",
                elapsedInState(), wait_timeout, perceptionSummary().c_str());
            transitionTo(
                State::STOP_BALL_TRACK,
                "ball perception unavailable after track start; kick fallback");
          } else {
            transitionToError("ball perception unavailable after track start");
          }
        }
        return;
      }
      const auto start_command = computeAlignCommand(*last_perception_);
      RCLCPP_INFO(
          get_logger(),
          "align entry check: valid perception before ALIGN | %s",
          alignCommandSummary(start_command, *last_perception_).c_str());
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
      RCLCPP_WARN(
          get_logger(),
          "align ending: timeout after %.3fs | last_error=%s | %s",
          elapsedInState(), lastAlignCommandSummary().c_str(),
          perceptionSummary().c_str());
      if (kick_on_align_timeout_) {
        if (!restoreFsmAfterAlign()) {
          transitionToError("align timeout and failed to restore fsm");
          return;
        }
        transitionTo(State::STOP_BALL_TRACK, "align timeout; kick fallback");
      } else {
        restoreFsmAfterAlign();
        transitionToError("align timeout");
      }
      return;
    }

    if (!readyForStationaryAlignSample()) {
      return;
    }

    last_processed_align_perception_time_ = last_perception_time_;

    if (!validPerception()) {
      stable_align_frames_ = 0;
      if (!last_logged_align_lost_perception_time_ ||
          !last_perception_time_ ||
          (*last_perception_time_ -
           *last_logged_align_lost_perception_time_).seconds() > 0.0) {
        RCLCPP_WARN(
            get_logger(),
            "align perception lost: last_error=%s | %s",
            lastAlignCommandSummary().c_str(), perceptionSummary().c_str());
        last_logged_align_lost_perception_time_ = last_perception_time_;
      }
      if (shouldKickAfterAlignBallLoss()) {
        RCLCPP_WARN(
            get_logger(),
            "align ending: ball lost near feet; kick fallback | last_error=%s",
            lastAlignCommandSummary().c_str());
        if (!restoreFsmAfterAlign()) {
          transitionToError("ball lost near feet but failed to restore fsm");
          return;
        }
        transitionTo(State::STOP_BALL_TRACK, "ball lost near feet; kick fallback");
      }
      return;
    }

    const auto command = computeAlignCommand(*last_perception_);
    last_valid_align_command_ = command;
    last_valid_align_ball_x_ = last_perception_->ball.point.x;
    RCLCPP_INFO(
        get_logger(), "align stationary error: %s",
        alignCommandSummary(command, *last_perception_).c_str());
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
        RCLCPP_INFO(
            get_logger(),
            "align ending: stable within tolerance | %s",
            alignCommandSummary(command, *last_perception_).c_str());
        if (!restoreFsmAfterAlign()) {
          transitionToError("align stable but failed to restore fsm");
          return;
        }
        transitionTo(State::STOP_BALL_TRACK, "align stable");
      }
      return;
    }

    stable_align_frames_ = 0;
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

  void handleCelebration() {
    if (celebration_goal_sent_) {
      if (elapsedInState() > celebration_result_timeout_s_) {
        transitionToError("celebration action result timeout");
      }
      return;
    }
    celebration_goal_sent_ = true;

    if (!sendCelebrationGoal()) {
      transitionToError("failed to send celebration action");
    }
  }

  void handleTrackBall() {
    if (!last_perception_ || !last_perception_time_) {
      stopGoalkeeperMotion("no perception");
      return;
    }

    const double perception_age =
        (now() - *last_perception_time_).seconds();
    if (perception_age >
        goalkeeper_perception_timeout_s_) {
      stopGoalkeeperMotion("perception timeout");
      return;
    }

    if (last_goalkeeper_processed_perception_time_ &&
        (*last_perception_time_ -
         *last_goalkeeper_processed_perception_time_).seconds() <= 0.0) {
      return;
    }
    last_goalkeeper_processed_perception_time_ = last_perception_time_;

    if (!last_perception_->image_has_ball ||
        !last_perception_->transform_valid ||
        last_perception_->ball_confidence <
            goalkeeper_min_ball_confidence_) {
      if (!goalkeeper_invalid_since_) {
        goalkeeper_invalid_since_ = now();
      }
      const double invalid_duration =
          (now() - *goalkeeper_invalid_since_).seconds();
      if (invalid_duration >= goalkeeper_perception_timeout_s_) {
        stopGoalkeeperMotion("invalid ball perception timeout");
      }
      return;
    }
    goalkeeper_invalid_since_.reset();

    const double image_x = last_perception_->ball.point.x;
    double lateral_speed = 0.0;
    if (std::abs(image_x) > goalkeeper_center_deadband_x_) {
      const double min_speed = std::min(
          std::abs(goalkeeper_min_lateral_speed_),
          std::abs(goalkeeper_lateral_speed_));
      const double magnitude = std::clamp(
          std::abs(goalkeeper_lateral_kp_ * image_x),
          min_speed, std::abs(goalkeeper_lateral_speed_));
      lateral_speed = std::copysign(
          magnitude, goalkeeper_lateral_sign_ * image_x);
    }

    if (lateral_speed == 0.0) {
      stopGoalkeeperMotion("ball centered");
    } else {
      if (!sendGoalkeeperVelocity(lateral_speed)) {
        transitionToError("goalkeeper velocity publish failed");
        return;
      }
      goalkeeper_motion_active_ = true;
    }

    const std::string detail =
        "goalkeeper control image_x=" + std::to_string(image_x) +
        " image_y=" +
        std::to_string(last_perception_->ball.point.y) +
        " vy=" + std::to_string(lateral_speed);
    publishBehavior("TRACK_BALL", detail);
  }

  bool sendGoalkeeperVelocity(double vy) {
    if (goalkeeper_enable_motion_) {
      publishVelocityCommand(0.0, vy, 0.0, vy == 0.0);
    }

    RCLCPP_INFO(
        get_logger(),
        "goalkeeper velocity %s: topic=%s vx=0.000 vy=%.3f wz=0.000 "
        "linear_z=%d image_x=%.3f deadband=%.3f",
        goalkeeper_enable_motion_ ? "sent" : "dry-run",
        velocity_command_topic_.c_str(), vy, vy == 0.0 ? 1 : 0,
        last_perception_ ? last_perception_->ball.point.x : 0.0,
        goalkeeper_center_deadband_x_);
    return true;
  }

  void stopGoalkeeperMotion(const char* reason) {
    if (!goalkeeper_motion_active_) {
      return;
    }
    if (sendGoalkeeperVelocity(0.0)) {
      goalkeeper_motion_active_ = false;
      RCLCPP_WARN(
          get_logger(), "goalkeeper motion stopped: reason=%s", reason);
    } else {
      goalkeeper_motion_active_ = false;
      RCLCPP_WARN(
          get_logger(),
          "goalkeeper motion stop command failed: reason=%s", reason);
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

  bool restoreFsmAfterAlign() {
    if (!restore_fsm_after_align_ || !obstacle_mode_active_ || !loco_client_) {
      disableContinuousGait();
      obstacle_mode_active_ = false;
      return true;
    }

    disableContinuousGait();

    const int restore_id = saved_fsm_id_.value_or(align_restore_fsm_id_);
    const int32_t ret = loco_client_->SetFsmId(restore_id);
    if (ret != 0) {
      RCLCPP_WARN(
          get_logger(), "SetFsmId(%d) restore after ALIGN failed: %d",
          restore_id, ret);
      return false;
    } else {
      RCLCPP_INFO(
          get_logger(), "align control mode restore sent: SetFsmId(%d) success",
          restore_id);
    }

    if (!confirmFsmId(restore_id)) {
      RCLCPP_WARN(
          get_logger(), "SetFsmId(%d) restore was sent but fsm confirmation failed",
          restore_id);
      return false;
    }

    obstacle_mode_active_ = false;
    return true;
  }

  bool confirmFsmId(int expected_fsm_id) {
    if (!loco_client_) {
      return false;
    }

    for (int attempt = 0; attempt < 5; ++attempt) {
      int current_fsm = -1;
      const int32_t ret = loco_client_->GetFsmId(current_fsm);
      if (ret == 0 && current_fsm == expected_fsm_id) {
        RCLCPP_INFO(
            get_logger(), "align control mode confirmed: fsm_id=%d",
            current_fsm);
        return true;
      }
      rclcpp::sleep_for(50ms);
    }
    return false;
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
    if ((!obstacle_mode_active_ && !continuous_gait_active_) ||
        align_stop_sent_) {
      return;
    }
    publishVelocityCommand(0.0, 0.0, 0.0, true);
    align_stop_sent_ = true;
    RCLCPP_INFO(
        get_logger(),
        "align velocity topic sent: vx=0.000 vy=0.000 wz=0.000 "
        "linear_z=1 stop");
  }

  bool sendObstacleStep(double vx, double vy, double wz) {
    publishVelocityCommand(vx, vy, wz, false);

    last_step_time_ = now();
    last_step_perception_time_ = last_perception_time_;
    align_stand_confirmed_time_.reset();
    align_stop_sent_ = false;
    return true;
  }

  void publishVelocityCommand(
      double vx, double vy, double wz, bool stop) {
    geometry_msgs::msg::Twist command;
    command.linear.x = vx;
    command.linear.y = vy;
    command.linear.z = stop ? 1.0 : 0.0;
    command.angular.z = wz;
    velocity_command_pub_->publish(command);
  }

  bool readyForStationaryAlignSample() {
    if (!last_perception_time_) {
      return false;
    }

    if (last_processed_align_perception_time_ &&
        (*last_perception_time_ -
         *last_processed_align_perception_time_).seconds() <= 0.0) {
      return false;
    }

    if (!align_require_standing_for_sample_) {
      if (!last_step_time_) {
        return true;
      }
      const double wait_s =
          align_step_duration_s_ + align_min_step_period_s_;
      return (now() - *last_step_time_).seconds() >= wait_s &&
             (!last_step_perception_time_ ||
              (*last_perception_time_ -
               *last_step_perception_time_).seconds() > 0.0);
    }

    if (last_step_time_ &&
        (now() - *last_step_time_).seconds() < align_step_duration_s_) {
      return false;
    }

    stopObstacleStep();

    if (!align_stand_confirmed_time_) {
      if (!loco_client_) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "align stationary sample blocked: LocoClient unavailable");
        return false;
      }

      int fsm_mode = -1;
      const int32_t ret = loco_client_->GetFsmMode(fsm_mode);
      if (ret != 0) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "align stationary sample blocked: GetFsmMode failed ret=%d", ret);
        return false;
      }
      if (fsm_mode != 0) {
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "align waiting for stand: fsm_mode=%d (0=standing, 1=moving)",
            fsm_mode);
        return false;
      }

      align_stand_confirmed_time_ = now();
      align_stand_perception_time_ = last_perception_time_;
      RCLCPP_INFO(
          get_logger(),
          "align stand confirmed: fsm_mode=0; wait %.3fs then use a new "
          "perception frame",
          align_min_step_period_s_);
      return false;
    }

    if ((now() - *align_stand_confirmed_time_).seconds() <
        align_min_step_period_s_) {
      return false;
    }

    if (align_stand_perception_time_ &&
        (*last_perception_time_ -
         *align_stand_perception_time_).seconds() <= 0.0) {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "align waiting for fresh perception after stand confirmation");
      return false;
    }

    if ((*last_perception_time_ -
         *align_stand_confirmed_time_).seconds() <= 0.0) {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "align ignoring perception captured before stand confirmation");
      return false;
    }

    int fsm_mode = -1;
    const int32_t ret = loco_client_->GetFsmMode(fsm_mode);
    if (ret != 0 || fsm_mode != 0) {
      align_stand_confirmed_time_.reset();
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "align stationary sample rejected: GetFsmMode ret=%d mode=%d",
          ret, fsm_mode);
      return false;
    }

    return true;
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

    command.vx = correctionVelocity(
        command.x_error, align_x_tolerance_m_, align_kx_,
        align_min_speed_, align_max_vx_);
    command.vy = correctionVelocity(
        command.y_error, align_y_tolerance_m_, align_ky_,
        align_min_speed_, align_max_vy_);
    command.wz = correctionVelocity(
        command.yaw_error, align_yaw_tolerance_rad_, align_kw_,
        align_min_speed_, align_max_wz_);
    return command;
  }

  std::string alignCommandSummary(
      const AlignCommand& command,
      const SoccerPerception& perception) const {
    std::ostringstream out;
    out << "x_err=" << command.x_error
        << " y_err=" << command.y_error
        << " yaw_err=" << command.yaw_error
        << " within_tolerance=" << boolText(command.within_tolerance)
        << " stable_frames=" << stable_align_frames_ << "/"
        << align_required_stable_frames_
        << " cmd_v=(" << command.vx << "," << command.vy << ","
        << command.wz << ")"
        << " ball=(" << perception.ball.point.x << ","
        << perception.ball.point.y << "," << perception.ball.point.z << ")"
        << " target=(" << align_target_ball_x_m_ << ","
        << align_target_ball_y_m_ << ")"
        << " tolerances=(" << align_x_tolerance_m_ << ","
        << align_y_tolerance_m_ << "," << align_yaw_tolerance_rad_ << ")";
    return out.str();
  }

  std::string lastAlignCommandSummary() const {
    if (!last_valid_align_command_) {
      return "none";
    }

    std::ostringstream out;
    out << "x_err=" << last_valid_align_command_->x_error
        << " y_err=" << last_valid_align_command_->y_error
        << " yaw_err=" << last_valid_align_command_->yaw_error
        << " within_tolerance="
        << boolText(last_valid_align_command_->within_tolerance)
        << " stable_frames=" << stable_align_frames_ << "/"
        << align_required_stable_frames_
        << " last_ball_x="
        << (last_valid_align_ball_x_ ? std::to_string(*last_valid_align_ball_x_)
                                     : "unknown")
        << " cmd_v=(" << last_valid_align_command_->vx << ","
        << last_valid_align_command_->vy << ","
        << last_valid_align_command_->wz << ")";
    return out.str();
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
    msg.detail = nowDetail(
        now().seconds(),
        detail + " celebration_selected=" + selectedCelebration().id);
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
      case State::CELEBRATE:
        return 0.5f;
      case State::TRACK_BALL:
        return 0.35f;
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

    RCLCPP_INFO(
        get_logger(), "kick action goal sending: server=%s params=%s",
        kick_action_server_.c_str(), goal.params.c_str());

    const uint64_t generation = mode_generation_;
    auto options = rclcpp_action::Client<ExecutionUnit>::SendGoalOptions();
    options.goal_response_callback =
        [this, generation](const GoalHandleExecutionUnit::SharedPtr& handle) {
          if (!handle) {
            if (generation != mode_generation_) {
              return;
            }
            RCLCPP_ERROR(get_logger(), "kick action goal response: rejected");
            transitionToError("kick goal rejected");
            return;
          }
          if (generation != mode_generation_) {
            kick_client_->async_cancel_goal(handle);
            return;
          }
          kick_goal_handle_ = handle;
          RCLCPP_INFO(get_logger(), "kick action goal response: accepted");
        };
    options.result_callback =
        [this, generation](
            const GoalHandleExecutionUnit::WrappedResult& result) {
          if (generation != mode_generation_) {
            return;
          }
          kick_goal_handle_.reset();
          const int action_result =
              result.result ? static_cast<int>(result.result->result) : -1;
          const auto action_error =
              result.result ? result.result->error_message : "empty result";
          RCLCPP_INFO(
              get_logger(),
              "kick action result received: code=%s action_result=%d "
              "error_message=%s",
              resultCodeName(result.code), action_result,
              action_error.c_str());
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
           << "\"policy_type\":\"" << jsonEscape(kick_policy_type_) << "\",";
    if (!kick_vendor_name_en_.empty()) {
      params << "\"vendor_name_en\":\""
             << jsonEscape(kick_vendor_name_en_) << "\",";
    }
    if (!kick_vendor_name_cn_.empty()) {
      params << "\"vendor_name_cn\":\""
             << jsonEscape(kick_vendor_name_cn_) << "\",";
    }
    params << "\"model\":\"" << jsonEscape(kick_model_path_) << "\","
           << "\"trajectory\":\"" << jsonEscape(kick_trajectory_path_) << "\","
           << "\"end_behavior\":\"" << jsonEscape(kick_end_behavior_) << "\","
           << "\"quat_comp\":" << kick_quat_comp_ << ","
           << "\"allowed_from\":[\"PASSIVE\",\"LOCO\",\"FIXEDSTAND\"]"
           << "}";
    goal.params = params.str();
    return goal;
  }

  bool sendCelebrationGoal() {
    if (!celebration_client_->wait_for_action_server(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(
                    celebration_server_timeout_s_)))) {
      RCLCPP_ERROR(
          get_logger(), "Celebration action server unavailable: %s",
          celebration_action_server_.c_str());
      return false;
    }

    auto goal = makeCelebrationGoal();
    if (goal.params.empty()) {
      return false;
    }

    RCLCPP_INFO(
        get_logger(),
        "celebration action goal sending: selected=%s server=%s params=%s",
        selectedCelebration().id.c_str(), celebration_action_server_.c_str(),
        goal.params.c_str());

    const uint64_t generation = mode_generation_;
    auto options = rclcpp_action::Client<ExecutionUnit>::SendGoalOptions();
    options.goal_response_callback =
        [this, generation](const GoalHandleExecutionUnit::SharedPtr& handle) {
          if (!handle) {
            if (generation != mode_generation_) {
              return;
            }
            RCLCPP_ERROR(
                get_logger(), "celebration action goal response: rejected");
            transitionToError("celebration goal rejected");
            return;
          }
          if (generation != mode_generation_) {
            celebration_client_->async_cancel_goal(handle);
            return;
          }
          celebration_goal_handle_ = handle;
          RCLCPP_INFO(
              get_logger(), "celebration action goal response: accepted");
        };
    options.result_callback =
        [this, generation](
            const GoalHandleExecutionUnit::WrappedResult& result) {
          if (generation != mode_generation_) {
            return;
          }
          celebration_goal_handle_.reset();
          const int action_result =
              result.result ? static_cast<int>(result.result->result) : -1;
          const auto action_error =
              result.result ? result.result->error_message : "empty result";
          RCLCPP_INFO(
              get_logger(),
              "celebration action result received: code=%s action_result=%d "
              "error_message=%s",
              resultCodeName(result.code), action_result,
              action_error.c_str());
          if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
            transitionToError("celebration action failed");
            return;
          }
          if (result.result &&
              result.result->result == ExecutionUnit::Result::SUCCESS) {
            enterIdle("celebration succeeded");
          } else {
            transitionToError(
                "celebration action failed: " + action_error);
          }
        };

    celebration_client_->async_send_goal(goal, options);
    publishBehavior("CELEBRATE", "celebration goal sent");
    return true;
  }

  ExecutionUnit::Goal makeCelebrationGoal() const {
    ExecutionUnit::Goal goal;
    const auto& action = selectedCelebration();

    if (!celebration_action_params_json_.empty()) {
      goal.params = celebration_action_params_json_;
      return goal;
    }

    if (action.model_path.empty() || action.trajectory_path.empty()) {
      RCLCPP_ERROR(
          get_logger(),
          "celebration_action_params_json is empty and celebration_model_path/"
          "celebration_trajectory_path are not both set for %s",
          action.id.c_str());
      return goal;
    }

    std::ostringstream params;
    params << "{"
           << "\"uuid\":\"" << jsonEscape(action.action_name) << "\","
           << "\"state_name\":\""
           << jsonEscape(action.action_name) << "\","
           << "\"policy_type\":\""
           << jsonEscape(celebration_policy_type_) << "\",";
    if (!celebration_vendor_name_en_.empty()) {
      params << "\"vendor_name_en\":\""
             << jsonEscape(celebration_vendor_name_en_) << "\",";
    }
    if (!celebration_vendor_name_cn_.empty()) {
      params << "\"vendor_name_cn\":\""
             << jsonEscape(celebration_vendor_name_cn_) << "\",";
    }
    params << "\"model\":\"" << jsonEscape(action.model_path) << "\","
           << "\"trajectory\":\""
           << jsonEscape(action.trajectory_path) << "\","
           << "\"end_behavior\":\""
           << jsonEscape(celebration_end_behavior_) << "\","
           << "\"quat_comp\":" << celebration_quat_comp_ << ","
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
      last_processed_align_perception_time_.reset();
      align_stand_confirmed_time_.reset();
      align_stand_perception_time_.reset();
      last_logged_align_lost_perception_time_.reset();
      align_stop_sent_ = false;
    }

    if (next == State::ALIGN) {
      align_stand_perception_time_ = last_perception_time_;
    }

    if (next == State::NAVIGATE_TO_POINT) {
      mode_ = "PENALTY_ATTACK";
      kick_goal_sent_ = false;
    }
    if (next == State::TRACK_BALL) {
      mode_ = "GOALKEEPER";
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
    if (mode_ == "GOALKEEPER") {
      stopGoalkeeperMotion("state error");
    }
    stopObstacleStep();
    restoreFsmAfterAlign();
    state_ = State::ERROR;
    state_enter_time_ = now();
    if (mode_ == "PENALTY_ATTACK") {
      publishVisionTrackCommand("STOP_BALL_TRACK");
    }
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
      case State::CELEBRATE:
        return "CELEBRATE";
      case State::TRACK_BALL:
        return "TRACK_BALL";
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
  bool remote_mode_control_enabled_ = true;
  double remote_combo_hold_s_ = 0.45;
  double remote_combo_cooldown_s_ = 1.0;
  PenaltyProfile penalty_left_profile_;
  PenaltyProfile penalty_center_profile_;
  PenaltyProfile penalty_right_profile_;
  PenaltyProfile penalty_default_profile_;
  GoalkeeperProfile goalkeeper_default_profile_;
  std::string active_profile_;

  std::string unitree_network_interface_;
  std::string velocity_command_topic_ = "/nav/cmd_vel_nav";
  int align_obstacle_fsm_id_ = 812;
  int align_restore_fsm_id_ = 802;
  bool restore_fsm_after_align_ = true;
  bool require_unitree_align_mode_ = true;
  bool align_use_continuous_gait_ = false;

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
  bool kick_on_align_timeout_ = true;

  double align_kx_ = 0.45;
  double align_ky_ = 0.55;
  double align_kw_ = 0.9;
  double align_max_vx_ = 0.50;
  double align_max_vy_ = 0.50;
  double align_max_wz_ = 0.50;
  double align_min_speed_ = 0.20;
  double align_step_duration_s_ = 1.00;
  double align_min_step_period_s_ = 0.15;
  bool align_require_standing_for_sample_ = true;

  double goalkeeper_min_ball_confidence_ = 0.30;
  bool goalkeeper_enable_motion_ = true;
  double goalkeeper_center_deadband_x_ = 0.05;
  double goalkeeper_lateral_kp_ = 2.0;
  double goalkeeper_min_lateral_speed_ = 0.20;
  double goalkeeper_lateral_speed_ = 1.00;
  double goalkeeper_lateral_sign_ = 1.0;
  double goalkeeper_perception_timeout_s_ = 1.00;

  double min_ball_confidence_ = 0.45;
  double min_goal_confidence_ = 0.35;
  std::string kick_action_name_ = "PENALTY_KICK";
  std::string kick_action_params_json_;
  std::string kick_model_path_;
  std::string kick_trajectory_path_;
  std::string kick_policy_type_ = "lingshu";
  std::string kick_vendor_name_en_;
  std::string kick_vendor_name_cn_;
  std::string kick_end_behavior_ = "switch_to_loco";
  double kick_quat_comp_ = -0.2;
  std::string kick_action_server_ = "/whole_body/action_ctrl";
  bool enable_kick_action_ = true;
  double kick_server_timeout_s_ = 2.0;
  double kick_result_timeout_s_ = 8.0;
  std::string celebration_action_params_json_;
  std::vector<CelebrationAction> celebration_actions_;
  size_t selected_celebration_index_ = 0;
  std::string celebration_policy_type_ = "lingshu";
  std::string celebration_vendor_name_en_;
  std::string celebration_vendor_name_cn_;
  std::string celebration_end_behavior_ = "switch_to_loco";
  double celebration_quat_comp_ = -0.2;
  std::string celebration_action_server_ = "/whole_body/action_ctrl";
  bool enable_celebration_action_ = true;
  double celebration_server_timeout_s_ = 2.0;
  double celebration_result_timeout_s_ = 15.0;

  State state_ = State::IDLE;
  rclcpp::Time state_enter_time_;
  std::optional<rclcpp::Time> last_step_time_;
  std::optional<rclcpp::Time> last_step_perception_time_;
  std::optional<rclcpp::Time> last_processed_align_perception_time_;
  std::optional<rclcpp::Time> align_stand_confirmed_time_;
  std::optional<rclcpp::Time> align_stand_perception_time_;
  std::optional<rclcpp::Time> last_logged_align_lost_perception_time_;
  std::optional<AlignCommand> last_valid_align_command_;
  std::optional<double> last_valid_align_ball_x_;
  int stable_align_frames_ = 0;
  bool obstacle_mode_active_ = false;
  bool continuous_gait_active_ = false;
  bool align_stop_sent_ = false;
  bool kick_goal_sent_ = false;
  bool celebration_goal_sent_ = false;
  uint64_t mode_generation_ = 0;
  GoalHandleExecutionUnit::SharedPtr kick_goal_handle_;
  GoalHandleExecutionUnit::SharedPtr celebration_goal_handle_;
  std::optional<int> saved_fsm_id_;
  std::optional<rclcpp::Time> last_goalkeeper_processed_perception_time_;
  std::optional<rclcpp::Time> goalkeeper_invalid_since_;
  bool goalkeeper_motion_active_ = false;

  std::unique_ptr<unitree::robot::g1::LocoClient> loco_client_;
  std::shared_ptr<
      unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>>
      low_state_subscriber_;

  std::atomic<int> pending_remote_request_{
      static_cast<int>(RemoteRequest::NONE)};
  RemoteRequest remote_candidate_ = RemoteRequest::NONE;
  bool remote_candidate_fired_ = false;
  std::chrono::steady_clock::time_point remote_candidate_since_{};
  std::optional<std::chrono::steady_clock::time_point>
      last_remote_trigger_time_;

  rclcpp::Subscription<SoccerPerception>::SharedPtr perception_sub_;
  rclcpp::Subscription<NavStatus>::SharedPtr nav_status_sub_;
  rclcpp::Publisher<GameModeCommand>::SharedPtr game_mode_pub_;
  rclcpp::Publisher<VisionTrackCommand>::SharedPtr vision_track_pub_;
  rclcpp::Publisher<BehaviorState>::SharedPtr behavior_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
      velocity_command_pub_;
  rclcpp_action::Client<ExecutionUnit>::SharedPtr kick_client_;
  rclcpp_action::Client<ExecutionUnit>::SharedPtr celebration_client_;
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
