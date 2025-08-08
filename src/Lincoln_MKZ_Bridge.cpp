#include <cmath>
#include <memory>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/joy.hpp"

/*────────── Autoware messages ──────────*/
#include "autoware_control_msgs/msg/control.hpp"
#include "autoware_vehicle_msgs/msg/gear_command.hpp"
#include "autoware_vehicle_msgs/msg/gear_report.hpp"
#include "autoware_vehicle_msgs/msg/turn_indicators_command.hpp"
#include "autoware_vehicle_msgs/msg/turn_indicators_report.hpp"
#include "autoware_vehicle_msgs/msg/hazard_lights_command.hpp"
#include "autoware_vehicle_msgs/msg/hazard_lights_report.hpp"
#include "autoware_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_vehicle_msgs/msg/velocity_report.hpp"
#include "autoware_vehicle_msgs/msg/control_mode_report.hpp"
#include "autoware_vehicle_msgs/srv/control_mode_command.hpp"

/*────────── Tier-IV messages ──────────*/
#include "tier4_vehicle_msgs/msg/actuation_command_stamped.hpp"
#include "tier4_vehicle_msgs/msg/actuation_status_stamped.hpp"
#include "tier4_vehicle_msgs/msg/steering_wheel_status_stamped.hpp"
#include "tier4_vehicle_msgs/msg/vehicle_emergency_stamped.hpp"

/*────────── Dataspeed DBW messages ──────────*/
#include "ds_dbw_msgs/msg/gear_cmd.hpp"
#include "ds_dbw_msgs/msg/gear_report.hpp"
#include "ds_dbw_msgs/msg/steering_cmd.hpp"
#include "ds_dbw_msgs/msg/steering_report.hpp"
#include "ds_dbw_msgs/msg/throttle_cmd.hpp"
#include "ds_dbw_msgs/msg/throttle_report.hpp"
#include "ds_dbw_msgs/msg/ulc_cmd.hpp"
#include "ds_dbw_msgs/msg/vehicle_velocity.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

/*============================================================================*/
class Lincoln_MKZ_Bridge : public rclcpp::Node
{
public:
  Lincoln_MKZ_Bridge() : Node("Lincoln_MKZ_Bridge")
  {
    /*──────────────── Parameters ────────────────*/
    declare_parameter<double>("Lincoln_Wheel_Base", 2.85);
    declare_parameter<double>("Lincoln_Steering", 14.8);

    wheel_base = get_parameter("Lincoln_Wheel_Base").as_double();
    steering_gain = get_parameter("Lincoln_Steering").as_double();

    RCLCPP_INFO(get_logger(), "Lincoln MKZ Bridge initialized with wheel_base=%.2f, steering_gain=%.2f", 
                wheel_base, steering_gain);

    /*──────────────── Publishers to DBW ────────────────*/
    gear_publisher_ = create_publisher<ds_dbw_msgs::msg::GearCmd>("/vehicle/gear/cmd", 10);
    steering_publisher_ = create_publisher<ds_dbw_msgs::msg::SteeringCmd>("/vehicle/steering/cmd", 10);
    ulc_publisher = create_publisher<ds_dbw_msgs::msg::UlcCmd>("/vehicle/ulc/cmd", 10);
    throttle_publisher_ = create_publisher<ds_dbw_msgs::msg::ThrottleCmd>("/vehicle/throttle/cmd", 10);

    /*──────────────── Publishers to Autoware ────────────────*/
    gear_report_publisher_ = create_publisher<autoware_vehicle_msgs::msg::GearReport>("/vehicle/status/gear_status", 10);
    steering_report_publisher_ = create_publisher<autoware_vehicle_msgs::msg::SteeringReport>("/vehicle/status/steering_status", 10);
    vehicle_speed_publisher_ = create_publisher<autoware_vehicle_msgs::msg::VelocityReport>("/vehicle/status/velocity_status", 10);
    control_mode_publisher_ = create_publisher<autoware_vehicle_msgs::msg::ControlModeReport>("/vehicle/status/control_mode", rclcpp::QoS{1});
    
    // NEW: Additional status publishers
    actuation_status_publisher_ = create_publisher<tier4_vehicle_msgs::msg::ActuationStatusStamped>("/vehicle/status/actuation_status", 10);
    steering_wheel_status_publisher_ = create_publisher<tier4_vehicle_msgs::msg::SteeringWheelStatusStamped>("/vehicle/status/steering_wheel_status", 10);
    turn_indicators_status_publisher_ = create_publisher<autoware_vehicle_msgs::msg::TurnIndicatorsReport>("/vehicle/status/turn_indicators_status", 10);
    hazard_lights_status_publisher_ = create_publisher<autoware_vehicle_msgs::msg::HazardLightsReport>("/vehicle/status/hazard_lights_status", 10);

    /*──────────────── Subscriptions from DBW ────────────────*/
    gear_report_subscription_ = create_subscription<ds_dbw_msgs::msg::GearReport>(
        "/vehicle/gear/report", 10,
        std::bind(&Lincoln_MKZ_Bridge::gearReportCallback, this, _1));
    
    steering_report_subscription_ = create_subscription<ds_dbw_msgs::msg::SteeringReport>(
        "/vehicle/steering/report", 10,
        std::bind(&Lincoln_MKZ_Bridge::steeringReportCallback, this, _1));
    
    vehicle_speed_subscription_ = create_subscription<ds_dbw_msgs::msg::VehicleVelocity>(
        "/vehicle/vehicle_velocity", 10,
        std::bind(&Lincoln_MKZ_Bridge::vehicleTwistCallback, this, _1));

    // DBW enabled status
    enable_subscription_ = create_subscription<std_msgs::msg::Bool>(
        "/vehicle/dbw_enabled", rclcpp::QoS(3),
        std::bind(&Lincoln_MKZ_Bridge::recvDbwEnabled, this, _1));

    /*──────────────── Subscriptions from Autoware ────────────────*/
    gear_subscription_ = create_subscription<autoware_vehicle_msgs::msg::GearCommand>(
        "/control/command/gear_cmd", 10,
        std::bind(&Lincoln_MKZ_Bridge::gearCallback, this, _1));
    
    control_subscription_ = create_subscription<autoware_control_msgs::msg::Control>(
        "/control/command/control_cmd", 10,
        std::bind(&Lincoln_MKZ_Bridge::controlCallback, this, _1));

    // NEW: Missing callback implementations
    turn_indicators_cmd_subscription_ = create_subscription<autoware_vehicle_msgs::msg::TurnIndicatorsCommand>(
        "/control/command/turn_indicators_cmd", rclcpp::QoS{1},
        std::bind(&Lincoln_MKZ_Bridge::callbackTurnIndicatorsCommand, this, _1));
    
    hazard_lights_cmd_sub_ = create_subscription<autoware_vehicle_msgs::msg::HazardLightsCommand>(
        "/control/command/hazard_lights_cmd", rclcpp::QoS{1},
        std::bind(&Lincoln_MKZ_Bridge::callbackHazardLightsCommand, this, _1));
    
    actuation_cmd_sub_ = create_subscription<tier4_vehicle_msgs::msg::ActuationCommandStamped>(
        "/control/command/actuation_cmd", 1,
        std::bind(&Lincoln_MKZ_Bridge::callbackActuationCmd, this, _1));
    
    emergency_subscription_ = create_subscription<tier4_vehicle_msgs::msg::VehicleEmergencyStamped>(
        "/control/command/emergency_cmd", 1,
        std::bind(&Lincoln_MKZ_Bridge::callbackEmergencyCmd, this, _1));

    /*──────────────── Services ────────────────*/
    control_mode_server_ = create_service<autoware_vehicle_msgs::srv::ControlModeCommand>(
        "/input/control_mode_request", 
        std::bind(&Lincoln_MKZ_Bridge::onControlModeRequest, this, _1, _2));

    /*──────────────── Optional joystick ────────────────*/
    sub_joy_ = create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, 
        std::bind(&Lincoln_MKZ_Bridge::recvJoy, this, _1));
  }

private:
  /*══════════════════════════════════════════════
   *  Member variables
   *══════════════════════════════════════════════*/
  
  // Vehicle parameters
  double wheel_base;
  double steering_gain;
  double curr_steer{0.0};
  
  // State variables
  bool is_clear_override_needed_{false};
  bool dbw_enabled_{false};
  rclcpp::Time ctrl_time;
  
  // Joystick state
  sensor_msgs::msg::Joy joy_;
  const size_t AXIS_THROTTLE = 1;
  const size_t AXIS_BRAKE = 2;
  const size_t AXIS_STEER = 3;
  bool joy_throttle_valid = false;
  bool joy_brake_valid = false;

  // Command storage for status reporting
  tier4_vehicle_msgs::msg::ActuationCommandStamped::ConstSharedPtr last_actuation_cmd_;
  autoware_vehicle_msgs::msg::TurnIndicatorsCommand::ConstSharedPtr last_turn_cmd_;
  autoware_vehicle_msgs::msg::HazardLightsCommand::ConstSharedPtr last_hazard_cmd_;

  /*──────────────── Publishers ────────────────*/
  // To DBW
  rclcpp::Publisher<ds_dbw_msgs::msg::GearCmd>::SharedPtr gear_publisher_;
  rclcpp::Publisher<ds_dbw_msgs::msg::SteeringCmd>::SharedPtr steering_publisher_;
  rclcpp::Publisher<ds_dbw_msgs::msg::UlcCmd>::SharedPtr ulc_publisher;
  rclcpp::Publisher<ds_dbw_msgs::msg::ThrottleCmd>::SharedPtr throttle_publisher_;

  // To Autoware
  rclcpp::Publisher<autoware_vehicle_msgs::msg::GearReport>::SharedPtr gear_report_publisher_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr steering_report_publisher_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr vehicle_speed_publisher_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::ControlModeReport>::SharedPtr control_mode_publisher_;
  rclcpp::Publisher<tier4_vehicle_msgs::msg::ActuationStatusStamped>::SharedPtr actuation_status_publisher_;
  rclcpp::Publisher<tier4_vehicle_msgs::msg::SteeringWheelStatusStamped>::SharedPtr steering_wheel_status_publisher_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::TurnIndicatorsReport>::SharedPtr turn_indicators_status_publisher_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::HazardLightsReport>::SharedPtr hazard_lights_status_publisher_;

  /*──────────────── Subscriptions ────────────────*/
  rclcpp::Subscription<ds_dbw_msgs::msg::GearReport>::SharedPtr gear_report_subscription_;
  rclcpp::Subscription<ds_dbw_msgs::msg::SteeringReport>::SharedPtr steering_report_subscription_;
  rclcpp::Subscription<ds_dbw_msgs::msg::VehicleVelocity>::SharedPtr vehicle_speed_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_subscription_;
  
  rclcpp::Subscription<autoware_vehicle_msgs::msg::GearCommand>::SharedPtr gear_subscription_;
  rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr control_subscription_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::TurnIndicatorsCommand>::SharedPtr turn_indicators_cmd_subscription_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::HazardLightsCommand>::SharedPtr hazard_lights_cmd_sub_;
  rclcpp::Subscription<tier4_vehicle_msgs::msg::ActuationCommandStamped>::SharedPtr actuation_cmd_sub_;
  rclcpp::Subscription<tier4_vehicle_msgs::msg::VehicleEmergencyStamped>::SharedPtr emergency_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;

  /*──────────────── Services ────────────────*/
  rclcpp::Service<autoware_vehicle_msgs::srv::ControlModeCommand>::SharedPtr control_mode_server_;

  /*══════════════════════════════════════════════
   *  Gear translation functions
   *══════════════════════════════════════════════*/
  uint8_t translate_gear(uint8_t autoware_gear)
  {
    using FordGear = ds_dbw_msgs::msg::Gear;
    switch(autoware_gear) {
      case autoware_vehicle_msgs::msg::GearCommand::NEUTRAL: return FordGear::NEUTRAL;
      case autoware_vehicle_msgs::msg::GearCommand::DRIVE:
      case autoware_vehicle_msgs::msg::GearCommand::DRIVE_2:
        return FordGear::DRIVE;
      case autoware_vehicle_msgs::msg::GearCommand::REVERSE:
      case autoware_vehicle_msgs::msg::GearCommand::REVERSE_2:
        return FordGear::REVERSE;
      case autoware_vehicle_msgs::msg::GearCommand::PARK: return FordGear::PARK;
      case autoware_vehicle_msgs::msg::GearCommand::LOW:
      case autoware_vehicle_msgs::msg::GearCommand::LOW_2:
        return FordGear::LOW;
      default: return FordGear::NONE;
    }
  }

  uint8_t translateFordGearToAutowareGear(uint8_t ford_gear_value)
  {
    using AutowareGear = autoware_vehicle_msgs::msg::GearReport;
    switch(ford_gear_value) {
      case ds_dbw_msgs::msg::Gear::PARK: return AutowareGear::PARK;
      case ds_dbw_msgs::msg::Gear::REVERSE: return AutowareGear::REVERSE;
      case ds_dbw_msgs::msg::Gear::NEUTRAL: return AutowareGear::NEUTRAL;
      case ds_dbw_msgs::msg::Gear::DRIVE: return AutowareGear::DRIVE;
      case ds_dbw_msgs::msg::Gear::LOW: return AutowareGear::LOW;
      default: return AutowareGear::NONE;
    }
  }

  /*══════════════════════════════════════════════
   *  NEW: Missing callback implementations
   *══════════════════════════════════════════════*/
  
  void callbackActuationCmd(const tier4_vehicle_msgs::msg::ActuationCommandStamped::SharedPtr msg)
  {
    RCLCPP_DEBUG(get_logger(), "Received actuation command: accel=%.3f, brake=%.3f, steer=%.3f", 
                 msg->actuation.accel_cmd, msg->actuation.brake_cmd, msg->actuation.steer_cmd);
    
    last_actuation_cmd_ = msg;
    
    // Publish actuation status (echo back the commanded values as status)
    tier4_vehicle_msgs::msg::ActuationStatusStamped status;
    status.header = msg->header;
    status.status.accel_status = msg->actuation.accel_cmd;
    status.status.brake_status = msg->actuation.brake_cmd;
    status.status.steer_status = msg->actuation.steer_cmd;
    actuation_status_publisher_->publish(status);
    
  }

  // TODO: Correct the logic so that this is passed down to DS DBW
  void callbackTurnIndicatorsCommand(const autoware_vehicle_msgs::msg::TurnIndicatorsCommand::SharedPtr msg)
  {
    // RCLCPP_DEBUG(get_logger(), "Received turn indicators command: %d", msg->command);
    
    last_turn_cmd_ = msg;
    
    // Publish turn indicators status
    autoware_vehicle_msgs::msg::TurnIndicatorsReport report;
    report.stamp = now();
    report.report = msg->command;
    turn_indicators_status_publisher_->publish(report);
    
  }

  void callbackHazardLightsCommand(const autoware_vehicle_msgs::msg::HazardLightsCommand::SharedPtr msg)
  {
    // RCLCPP_DEBUG(get_logger(), "Received hazard lights command: %d", msg->command);
    
    last_hazard_cmd_ = msg;
    
    // Publish hazard lights status
    autoware_vehicle_msgs::msg::HazardLightsReport report;
    report.stamp = now();
    report.report = msg->command;
    hazard_lights_status_publisher_->publish(report);
    
  }

  void recvDbwEnabled(const std_msgs::msg::Bool::SharedPtr msg)
  {
    dbw_enabled_ = msg->data;
    // RCLCPP_DEBUG(get_logger(), "DBW enabled status changed: %s", dbw_enabled_ ? "ENABLED" : "DISABLED");
    
    // Publish control mode based on DBW status
    autoware_vehicle_msgs::msg::ControlModeReport mode_report;
    mode_report.stamp = now();
    mode_report.mode = dbw_enabled_ ? 
        autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS :
        autoware_vehicle_msgs::msg::ControlModeReport::MANUAL;
    control_mode_publisher_->publish(mode_report);
  }

  /*══════════════════════════════════════════════
   *  Enhanced emergency handling
   *══════════════════════════════════════════════*/
  void callbackEmergencyCmd(const tier4_vehicle_msgs::msg::VehicleEmergencyStamped::SharedPtr msg) 
  {
    // RCLCPP_DEBUG(get_logger(), "Emergency command received: %s", msg->emergency ? "ACTIVE" : "INACTIVE");
    
    if (msg->emergency) {
      // Send emergency brake command via ULC
      ds_dbw_msgs::msg::UlcCmd emergency_cmd;
      emergency_cmd.header.stamp = now();
      emergency_cmd.cmd_type = ds_dbw_msgs::msg::UlcCmd::CMD_ACCEL;
      emergency_cmd.cmd = -0.5;  // Negative acceleration for braking
      emergency_cmd.enable = true;
      emergency_cmd.clear = false;
      ulc_publisher->publish(emergency_cmd);
      
      RCLCPP_WARN(get_logger(), "Emergency brake applied via ULC");
    }
  }

  /*══════════════════════════════════════════════
   *  Original callback implementations (enhanced)
   *══════════════════════════════════════════════*/
  
  void onControlModeRequest(
      const autoware_vehicle_msgs::srv::ControlModeCommand::Request::SharedPtr request,
      const autoware_vehicle_msgs::srv::ControlModeCommand::Response::SharedPtr response)
  {
    RCLCPP_INFO(get_logger(), "Control mode request: %d", request->mode);
    
    if (request->mode == autoware_vehicle_msgs::srv::ControlModeCommand::Request::AUTONOMOUS) {
      is_clear_override_needed_ = true;
      response->success = true;
      RCLCPP_INFO(get_logger(), "Switched to AUTONOMOUS mode");
      return;
    }

    if (request->mode == autoware_vehicle_msgs::srv::ControlModeCommand::Request::MANUAL) {
      is_clear_override_needed_ = true;
      response->success = true;
      RCLCPP_INFO(get_logger(), "Switched to MANUAL mode");
      return;
    }

    RCLCPP_ERROR(get_logger(), "Unsupported control mode: %d", request->mode);
    response->success = false;
  }

  void gearCallback(const autoware_vehicle_msgs::msg::GearCommand::SharedPtr msg)
  {
    RCLCPP_DEBUG(get_logger(), "Gear command: %d", msg->command);
    
    ds_dbw_msgs::msg::GearCmd ford_msg;
    ford_msg.cmd.value = translate_gear(msg->command);
    gear_publisher_->publish(ford_msg);
  }

  void controlCallback(const autoware_control_msgs::msg::Control::SharedPtr msg)
  {
    ctrl_time = now();
    
    // Longitudinal control via ULC
    ds_dbw_msgs::msg::UlcCmd ulc_cmd;
    ulc_cmd.header.stamp = ctrl_time;
    ulc_cmd.cmd_type = ds_dbw_msgs::msg::UlcCmd::CMD_VELOCITY;
    ulc_cmd.cmd = msg->longitudinal.velocity;
    ulc_cmd.clear = false;
    ulc_cmd.enable = true;
    ulc_cmd.limit_accel = 3;
    ulc_cmd.limit_decel = 3;
    ulc_cmd.limit_jerk_throttle = 0;
    ulc_cmd.limit_jerk_brake = 0;
    ulc_cmd.enable_shift = false;
    ulc_cmd.enable_shift_park = false;
    ulc_cmd.coast_decel = false;
    ulc_publisher->publish(ulc_cmd);

    // Lateral control via steering
    ds_dbw_msgs::msg::SteeringCmd steering_msg;
    double steering_angle_degrees = steering_gain * msg->lateral.steering_tire_angle * (180.0 / M_PI);
    steering_msg.cmd = steering_angle_degrees;
    
    // Add joystick input if available
    if (joy_.axes.size() > AXIS_STEER) {
      steering_msg.cmd += joy_.axes[AXIS_STEER];
    }

    steering_msg.cmd_type = ds_dbw_msgs::msg::SteeringCmd::CMD_ANGLE;
    steering_msg.cmd_rate = 0.0;
    steering_msg.cmd_accel = 0.0;
    steering_msg.enable = true;
    steering_publisher_->publish(steering_msg);
  }

  void gearReportCallback(const ds_dbw_msgs::msg::GearReport::SharedPtr msg)
  {
    autoware_vehicle_msgs::msg::GearReport aw_report;
    aw_report.stamp = msg->header.stamp;
    aw_report.report = translateFordGearToAutowareGear(msg->gear.value);
    gear_report_publisher_->publish(aw_report);
  }

  void steeringReportCallback(const ds_dbw_msgs::msg::SteeringReport::SharedPtr msg)
  {
    // Steering tire angle report
    autoware_vehicle_msgs::msg::SteeringReport aw_steering_report;
    aw_steering_report.stamp = msg->header.stamp;
    curr_steer = msg->steering_wheel_angle * (M_PI / 180.0);
    aw_steering_report.steering_tire_angle = curr_steer / steering_gain;
    steering_report_publisher_->publish(aw_steering_report);

    // NEW: Steering wheel status report
    tier4_vehicle_msgs::msg::SteeringWheelStatusStamped wheel_status;
    wheel_status.stamp = msg->header.stamp;
    wheel_status.data = curr_steer;  // Steering wheel angle in radians
    steering_wheel_status_publisher_->publish(wheel_status);
  }

  void vehicleTwistCallback(const ds_dbw_msgs::msg::VehicleVelocity::SharedPtr msg)
  {
    double longitudinal_velocity = msg->vehicle_velocity_propulsion;
    
    autoware_vehicle_msgs::msg::VelocityReport velocity_msg;
    velocity_msg.header.frame_id = "base_link";
    velocity_msg.header.stamp = msg->header.stamp;
    velocity_msg.longitudinal_velocity = longitudinal_velocity;
    velocity_msg.heading_rate = longitudinal_velocity * std::tan(curr_steer) / wheel_base;
    vehicle_speed_publisher_->publish(velocity_msg);
  }

  void recvJoy(const sensor_msgs::msg::Joy::ConstSharedPtr msg)
  {
    if (msg->axes.size() <= AXIS_STEER) return;
    
    joy_throttle_valid = msg->axes[AXIS_THROTTLE] != 0.0;
    joy_brake_valid = msg->axes[AXIS_BRAKE] != 0.0;
    joy_ = *msg;
  }
};

/*============================================================================*/
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Lincoln_MKZ_Bridge>();
  
  RCLCPP_INFO(node->get_logger(), "Lincoln MKZ Bridge node started");
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
