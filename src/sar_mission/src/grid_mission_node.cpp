// Grid coverage mission node for PX4 offboard control.
//
// Streams OffboardControlMode + TrajectorySetpoint (position, NED) at 10 Hz,
// flies takeoff -> boustrophedon sweep -> RTL. Based on the PX4 px4_ros_com
// offboard_control example patterns (QoS, heartbeat-before-arm, VehicleCommand
// usage). PX4 main topic names are versioned; all topic names are parameters.

#include <chrono>
#include <cmath>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include "coverage_planner.hpp"

using namespace std::chrono_literals;
using sar_mission::CoveragePlanner;
using sar_mission::GridConfig;
using sar_mission::Waypoint;

enum class State { kStreaming, kTakeoff, kSweep, kRtl, kDone, kAbort };

class GridMissionNode : public rclcpp::Node
{
public:
  GridMissionNode()
  : Node("grid_mission_node")
  {
    // ---- parameters ----
    declare_parameter("topic_local_position", "/fmu/out/vehicle_local_position_v1");
    declare_parameter("topic_vehicle_status", "/fmu/out/vehicle_status_v4");
    declare_parameter("topic_offboard_control_mode", "/fmu/in/offboard_control_mode");
    declare_parameter("topic_trajectory_setpoint", "/fmu/in/trajectory_setpoint");
    declare_parameter("topic_vehicle_command", "/fmu/in/vehicle_command");

    declare_parameter("aoi_north_min", -20.0);
    declare_parameter("aoi_north_max", 20.0);
    declare_parameter("aoi_east_min", 10.0);
    declare_parameter("aoi_east_max", 60.0);
    declare_parameter("altitude_m", 10.0);
    declare_parameter("hfov_deg", 99.7);
    declare_parameter("vfov_deg", 83.0);
    declare_parameter("side_overlap", 0.25);
    declare_parameter("lane_heading_deg", 90.0);
    declare_parameter("reach_tol_xy_m", 1.0);
    declare_parameter("reach_tol_z_m", 0.6);

    // ---- pubs/subs ----
    const auto qos_in = rclcpp::QoS(10);
    offboard_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
      get_parameter("topic_offboard_control_mode").as_string(), qos_in);
    setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      get_parameter("topic_trajectory_setpoint").as_string(), qos_in);
    command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
      get_parameter("topic_vehicle_command").as_string(), qos_in);

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    const auto qos_out = rclcpp::QoS(
      rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    local_pos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      get_parameter("topic_local_position").as_string(), qos_out,
      [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        last_local_pos_ = *msg;
        last_pos_time_ = now();
        have_position_ = true;
      });
    status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      get_parameter("topic_vehicle_status").as_string(), qos_out,
      [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
        last_status_ = *msg;
        have_status_ = true;
      });

    path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/sar/planned_path", rclcpp::QoS(1).transient_local());

    // ---- mission plan ----
    GridConfig cfg{
      get_parameter("aoi_north_min").as_double(), get_parameter("aoi_north_max").as_double(),
      get_parameter("aoi_east_min").as_double(), get_parameter("aoi_east_max").as_double(),
      get_parameter("altitude_m").as_double(),
      get_parameter("hfov_deg").as_double(), get_parameter("vfov_deg").as_double(),
      get_parameter("side_overlap").as_double(), get_parameter("lane_heading_deg").as_double()};
    CoveragePlanner planner(cfg);
    waypoints_ = planner.plan();
    altitude_ = cfg.altitude_m;
    tol_xy_ = get_parameter("reach_tol_xy_m").as_double();
    tol_z_ = get_parameter("reach_tol_z_m").as_double();

    RCLCPP_INFO(get_logger(), "planned %zu waypoints, lane spacing %.1f m",
                waypoints_.size(), planner.lane_spacing());
    publishPlannedPath();

    timer_ = create_wall_timer(100ms, [this]() { tick(); });
  }

private:
  uint64_t nowUs() { return static_cast<uint64_t>(now().nanoseconds() / 1000); }

  void publishPlannedPath()
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = "ned";
    path.header.stamp = now();
    for (const auto & wp : waypoints_) {
      geometry_msgs::msg::PoseStamped p;
      p.header = path.header;
      p.pose.position.x = wp.north;
      p.pose.position.y = wp.east;
      p.pose.position.z = wp.down;
      path.poses.push_back(p);
    }
    path_pub_->publish(path);
  }

  void publishOffboardMode()
  {
    px4_msgs::msg::OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = nowUs();
    offboard_mode_pub_->publish(msg);
  }

  void publishSetpoint(float north, float east, float down, float yaw)
  {
    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.position = {north, east, down};
    msg.velocity = {NAN, NAN, NAN};
    msg.acceleration = {NAN, NAN, NAN};
    msg.jerk = {NAN, NAN, NAN};
    msg.yaw = yaw;
    msg.yawspeed = NAN;
    msg.timestamp = nowUs();
    setpoint_pub_->publish(msg);
  }

  void sendCommand(uint16_t command, float p1 = 0.0f, float p2 = 0.0f)
  {
    px4_msgs::msg::VehicleCommand msg{};
    msg.param1 = p1;
    msg.param2 = p2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = nowUs();
    command_pub_->publish(msg);
  }

  bool reached(float north, float east, float down) const
  {
    if (!have_position_) return false;
    const float dn = last_local_pos_.x - north;
    const float de = last_local_pos_.y - east;
    const float dz = last_local_pos_.z - down;
    return (std::hypot(dn, de) < tol_xy_) && (std::fabs(dz) < tol_z_);
  }

  void tick()
  {
    // always stream offboard mode + current setpoint (PX4 needs >=2 Hz)
    publishOffboardMode();
    publishSetpoint(cur_sp_north_, cur_sp_east_, cur_sp_down_, cur_sp_yaw_);

    // abort only if we HAD telemetry and lost it mid-flight
    if (have_position_ &&
        state_ != State::kStreaming && state_ != State::kDone && state_ != State::kAbort &&
        (now() - last_pos_time_) > 2s) {
      RCLCPP_ERROR(get_logger(), "lost local position telemetry, aborting");
      state_ = State::kAbort;
    }

    switch (state_) {
      case State::kStreaming:
        if (!have_position_ || !have_status_) {
          break;  // wait for telemetry before arming
        }
        if (stream_count_++ == 10) {  // 1 s of setpoints before mode+arm
          RCLCPP_INFO(get_logger(), "requesting offboard mode and arming");
          sendCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
          sendCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1, 0);
          cur_sp_down_ = -static_cast<float>(altitude_);
          state_ = State::kTakeoff;
          state_enter_time_ = now();
        }
        break;

      case State::kTakeoff:
        if (have_status_ && last_status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
          offboard_confirmed_ = true;
        }
        if (!offboard_confirmed_ && (now() - state_enter_time_) > 8s) {
          RCLCPP_ERROR(get_logger(), "offboard mode not confirmed within 8 s, aborting");
          state_ = State::kAbort;
          break;
        }
        if (reached(0.0f, 0.0f, cur_sp_down_)) {
          RCLCPP_INFO(get_logger(), "takeoff complete, starting sweep (%zu waypoints)", waypoints_.size());
          state_ = State::kSweep;
          wp_index_ = 0;
        }
        break;

      case State::kSweep: {
        if (have_status_ && last_status_.nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
          RCLCPP_ERROR(get_logger(), "left offboard mode during sweep (failsafe?), aborting");
          state_ = State::kAbort;
          break;
        }
        if (wp_index_ >= waypoints_.size()) {
          RCLCPP_INFO(get_logger(), "sweep complete, sending RTL");
          sendCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH);
          state_ = State::kRtl;
          state_enter_time_ = now();
          break;
        }
        const auto & wp = waypoints_[wp_index_];
        cur_sp_north_ = wp.north; cur_sp_east_ = wp.east;
        cur_sp_down_ = wp.down; cur_sp_yaw_ = wp.yaw;
        if (reached(wp.north, wp.east, wp.down)) {
          RCLCPP_INFO(get_logger(), "waypoint %zu/%zu reached (N %.1f E %.1f)",
                      wp_index_ + 1, waypoints_.size(), wp.north, wp.east);
          ++wp_index_;
        }
        break;
      }

      case State::kRtl:
        // SITL quirk: RTL can hover a few cm above ground without the land
        // detector firing (home altitude bias). Escalate: RTL -> Land mode ->
        // forced disarm, whichever terminates first.
        if (have_status_ && have_position_ &&
            last_status_.arming_state != px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED &&
            last_local_pos_.z > -1.0f) {
          const double in_rtl = (now() - state_enter_time_).seconds();
          if (!land_forced_ && in_rtl > 25.0) {
            RCLCPP_WARN(get_logger(), "RTL did not disarm near ground, forcing Land mode");
            sendCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
            land_forced_ = true;
          } else if (land_forced_ && !disarm_forced_ && in_rtl > 45.0) {
            RCLCPP_WARN(get_logger(), "landed but not disarmed, forcing disarm");
            sendCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
                        0, 21196);  // param2 21196 = force disarm in air/ground
            disarm_forced_ = true;
          }
        }
        if (have_status_ &&
            last_status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED) {
          RCLCPP_INFO(get_logger(), "landed and disarmed, mission done");
          state_ = State::kDone;
        }
        break;

      case State::kDone:
      case State::kAbort:
        // hold last setpoint; offboard stream keeps running harmlessly
        break;
    }
  }

  // pubs/subs
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // state
  State state_{State::kStreaming};
  uint32_t stream_count_{0};
  std::vector<Waypoint> waypoints_;
  size_t wp_index_{0};
  double altitude_{10.0};
  double tol_xy_{1.0}, tol_z_{0.6};
  float cur_sp_north_{0}, cur_sp_east_{0}, cur_sp_down_{0}, cur_sp_yaw_{0};

  px4_msgs::msg::VehicleLocalPosition last_local_pos_{};
  px4_msgs::msg::VehicleStatus last_status_{};
  rclcpp::Time last_pos_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time state_enter_time_{0, 0, RCL_ROS_TIME};
  bool have_position_{false};
  bool have_status_{false};
  bool offboard_confirmed_{false};
  bool land_forced_{false};
  bool disarm_forced_{false};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GridMissionNode>());
  rclcpp::shutdown();
  return 0;
}
