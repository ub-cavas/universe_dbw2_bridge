#include <cmath>
#include <memory>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

/*────────── Autoware messages ──────────*/
#include "autoware_control_msgs/msg/control.hpp"
#include "autoware_vehicle_msgs/srv/control_mode_command.hpp"
#include "autoware_vehicle_msgs/msg/control_mode_report.hpp"
#include "autoware_vehicle_msgs/msg/gear_command.hpp"
#include "autoware_vehicle_msgs/msg/gear_report.hpp"
#include "autoware_vehicle_msgs/msg/hazard_lights_command.hpp"
#include "autoware_vehicle_msgs/msg/hazard_lights_report.hpp"
#include "autoware_vehicle_msgs/msg/turn_indicators_command.hpp"
#include "autoware_vehicle_msgs/msg/turn_indicators_report.hpp"
#include "autoware_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_vehicle_msgs/msg/velocity_report.hpp"
#include "tier4_vehicle_msgs/msg/vehicle_emergency_stamped.hpp"

/*────────── Tier4 Vehicle Messages(for Calibrator) ──────────*/
#include "tier4_vehicle_msgs/msg/actuation_status_stamped.hpp"

/*────────── Dataspeed DBW messages ──────────*/
#include "ds_dbw_msgs/msg/gear_cmd.hpp"
#include "ds_dbw_msgs/msg/gear_report.hpp"
#include "ds_dbw_msgs/msg/steering_cmd.hpp"
#include "ds_dbw_msgs/msg/steering_report.hpp"
#include "ds_dbw_msgs/msg/turn_signal_cmd.hpp"
#include "ds_dbw_msgs/msg/turn_signal_report.hpp"
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
    gear_pub = create_publisher<ds_dbw_msgs::msg::GearCmd>("/vehicle/gear/cmd", 10);
    steering_pub = create_publisher<ds_dbw_msgs::msg::SteeringCmd>("/vehicle/steering/cmd", 10);
    turn_signal_pub = create_publisher<ds_dbw_msgs::msg::TurnSignalCmd>("/vehicle/turn_signal/cmd", 10);
    ulc_pub = create_publisher<ds_dbw_msgs::msg::UlcCmd>("/vehicle/ulc/cmd", 10);

    /*──────────────── Publishers to Autoware ────────────────*/
    control_mode_report_pub = create_publisher<autoware_vehicle_msgs::msg::ControlModeReport>("/vehicle/status/control_mode", rclcpp::QoS{1});
    gear_report_pub = create_publisher<autoware_vehicle_msgs::msg::GearReport>("/vehicle/status/gear_status", 10);
    hazard_lights_report_pub = create_publisher<autoware_vehicle_msgs::msg::HazardLightsReport>("/vehicle/status/hazard_lights_status", 10);
    steering_report_pub = create_publisher<autoware_vehicle_msgs::msg::SteeringReport>("/vehicle/status/steering_status", 10);
    turn_indicators_report_pub = create_publisher<autoware_vehicle_msgs::msg::TurnIndicatorsReport>("/vehicle/status/turn_indicators_status", 10);
    velocity_report_pub = create_publisher<autoware_vehicle_msgs::msg::VelocityReport>("/vehicle/status/velocity_status", 10);
    actuation_status_pub = create_publisher<tier4_vehicle_msgs::msg::ActuationStatusStamped>("/vehicle/status/actuation_status", 10);

    /*──────────────── Subscriptions from DBW ────────────────*/
    enable_sub = create_subscription<std_msgs::msg::Bool>(
        "/vehicle/dbw_enabled", rclcpp::QoS(3),
        std::bind(&Lincoln_MKZ_Bridge::callbackDbwEnabled, this, _1));
    
    gear_report_sub = create_subscription<ds_dbw_msgs::msg::GearReport>(
        "/vehicle/gear/report", 10,
        std::bind(&Lincoln_MKZ_Bridge::callbackGearReport, this, _1));
    
    steering_report_sub = create_subscription<ds_dbw_msgs::msg::SteeringReport>(
        "/vehicle/steering/report", 10,
        std::bind(&Lincoln_MKZ_Bridge::callbackSteeringReport, this, _1));
    
    turn_signal_report_sub = create_subscription<ds_dbw_msgs::msg::TurnSignalReport>(
        "/vehicle/turn_signal/report", 10,
        std::bind(&Lincoln_MKZ_Bridge::callbackTurnSignalReport, this, _1));
    
    vehicle_speed_sub = create_subscription<ds_dbw_msgs::msg::VehicleVelocity>(
        "/vehicle/vehicle_velocity", 10,
        std::bind(&Lincoln_MKZ_Bridge::callbackVehicleTwist, this, _1));

    /*──────────────── Subscriptions from Autoware ────────────────*/
    gear_sub = create_subscription<autoware_vehicle_msgs::msg::GearCommand>(
        "/control/command/gear_cmd", 10,
        std::bind(&Lincoln_MKZ_Bridge::callbackGearCmd, this, _1));
    
    control_sub = create_subscription<autoware_control_msgs::msg::Control>(
        "/control/command/control_cmd", 10,
        std::bind(&Lincoln_MKZ_Bridge::callbackControlCmd, this, _1));

    turn_indicators_cmd_sub = create_subscription<autoware_vehicle_msgs::msg::TurnIndicatorsCommand>(
        "/control/command/turn_indicators_cmd", rclcpp::QoS{1},
        std::bind(&Lincoln_MKZ_Bridge::callbackTurnIndicatorsCmd, this, _1));
    
    hazard_lights_cmd_sub = create_subscription<autoware_vehicle_msgs::msg::HazardLightsCommand>(
        "/control/command/hazard_lights_cmd", rclcpp::QoS{1},
        std::bind(&Lincoln_MKZ_Bridge::callbackHazardLightsCmd, this, _1));
    
    emergency_cmd_sub = create_subscription<tier4_vehicle_msgs::msg::VehicleEmergencyStamped>(
        "/control/command/emergency_cmd", 1,
        std::bind(&Lincoln_MKZ_Bridge::callbackEmergencyCmd, this, _1));

    /*──────────────── Services ────────────────*/
    control_mode_server = create_service<autoware_vehicle_msgs::srv::ControlModeCommand>(
        "/input/control_mode_request", 
        std::bind(&Lincoln_MKZ_Bridge::onControlModeRequest, this, _1, _2));

  }

private:
  // Vehicle parameters
  double wheel_base;
  double steering_gain;
  double curr_steer{0.0};
  
  // State variables
  bool is_clear_override_needed_{false};
  bool dbw_enabled_{false};

  // State variables(for calibration actuation tracking)
  double current_accel_cmd_{0.0};
  double current_brake_cmd_{0.0};
  double current_steer_cmd_{0.0};

  /*──────────────── Publishers ────────────────*/
  // To DBW
  rclcpp::Publisher<ds_dbw_msgs::msg::GearCmd>::SharedPtr gear_pub;
  rclcpp::Publisher<ds_dbw_msgs::msg::SteeringCmd>::SharedPtr steering_pub;
  rclcpp::Publisher<ds_dbw_msgs::msg::TurnSignalCmd>::SharedPtr turn_signal_pub;
  rclcpp::Publisher<ds_dbw_msgs::msg::UlcCmd>::SharedPtr ulc_pub;

  // To Autoware
  rclcpp::Publisher<autoware_vehicle_msgs::msg::ControlModeReport>::SharedPtr control_mode_report_pub;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::GearReport>::SharedPtr gear_report_pub;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::HazardLightsReport>::SharedPtr hazard_lights_report_pub;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr steering_report_pub;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::TurnIndicatorsReport>::SharedPtr turn_indicators_report_pub;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr velocity_report_pub;
  rclcpp::Publisher<tier4_vehicle_msgs::msg::ActuationStatusStamped>::SharedPtr actuation_status_pub;
  
  /*──────────────── Subscriptions ────────────────*/
  // From DBW
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub;
  rclcpp::Subscription<ds_dbw_msgs::msg::GearReport>::SharedPtr gear_report_sub;
  rclcpp::Subscription<ds_dbw_msgs::msg::SteeringReport>::SharedPtr steering_report_sub;
  rclcpp::Subscription<ds_dbw_msgs::msg::TurnSignalReport>::SharedPtr turn_signal_report_sub;
  rclcpp::Subscription<ds_dbw_msgs::msg::VehicleVelocity>::SharedPtr vehicle_speed_sub;
  
  // From Autoware
  rclcpp::Subscription<autoware_vehicle_msgs::msg::GearCommand>::SharedPtr gear_sub;
  rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr control_sub;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::TurnIndicatorsCommand>::SharedPtr turn_indicators_cmd_sub;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::HazardLightsCommand>::SharedPtr hazard_lights_cmd_sub;
  rclcpp::Subscription<tier4_vehicle_msgs::msg::VehicleEmergencyStamped>::SharedPtr emergency_cmd_sub;

  /*──────────────── Services ────────────────*/
  rclcpp::Service<autoware_vehicle_msgs::srv::ControlModeCommand>::SharedPtr control_mode_server;

  uint8_t translateGearAutowareToDS(uint8_t autoware_gear)
  {
    using AutowareGear = autoware_vehicle_msgs::msg::GearCommand;
    using DSGear = ds_dbw_msgs::msg::Gear;

    switch(autoware_gear) {
      case AutowareGear::NEUTRAL: 
        return DSGear::NEUTRAL;
      case AutowareGear::DRIVE:
      case AutowareGear::DRIVE_2:
        return DSGear::DRIVE;
      case AutowareGear::REVERSE:
      case AutowareGear::REVERSE_2:
        return DSGear::REVERSE;
      case AutowareGear::PARK: 
        return DSGear::PARK;
      case AutowareGear::LOW:
      case AutowareGear::LOW_2:
        return DSGear::LOW;
      default: 
        return DSGear::NONE;
    }
  }

  uint8_t translateGearDSToAutoware(uint8_t ds_gear)
  {
    using AutowareGear = autoware_vehicle_msgs::msg::GearReport;
    using DSGear = ds_dbw_msgs::msg::Gear;

    switch(ds_gear) {
      case DSGear::PARK:
        return AutowareGear::PARK;
      case DSGear::REVERSE:
        return AutowareGear::REVERSE;
      case DSGear::NEUTRAL:
        return AutowareGear::NEUTRAL;
      case DSGear::DRIVE:
        return AutowareGear::DRIVE;
      case DSGear::LOW:
        return AutowareGear::LOW;
      default:
        return AutowareGear::NONE;
    }
  }

  void callbackTurnIndicatorsCmd(const autoware_vehicle_msgs::msg::TurnIndicatorsCommand::SharedPtr msg)
  {
    RCLCPP_DEBUG(get_logger(), "Received turn indicators command: %d", msg->command);
    
    using autoware_vehicle_msgs::msg::TurnIndicatorsCommand;
    using ds_dbw_msgs::msg::TurnSignal;

    ds_dbw_msgs::msg::TurnSignalCmd tsc_msg;

    if (msg->command == TurnIndicatorsCommand::ENABLE_LEFT) {
      tsc_msg.cmd.value = TurnSignal::LEFT;
    } else if (msg->command == TurnIndicatorsCommand::ENABLE_RIGHT) {
      tsc_msg.cmd.value = TurnSignal::RIGHT;
    } else {
      tsc_msg.cmd.value = TurnSignal::NONE;
    }
  
    turn_signal_pub->publish(tsc_msg);
  }

  void callbackHazardLightsCmd(const autoware_vehicle_msgs::msg::HazardLightsCommand::SharedPtr msg)
  {
    RCLCPP_DEBUG(get_logger(), "Received hazard lights command: %d", msg->command);

    using autoware_vehicle_msgs::msg::HazardLightsCommand;
    using ds_dbw_msgs::msg::TurnSignal;

    ds_dbw_msgs::msg::TurnSignalCmd tsc_msg;

    if (msg->command == HazardLightsCommand::ENABLE) {
      tsc_msg.cmd.value = TurnSignal::HAZARD;
    } else {
      tsc_msg.cmd.value = TurnSignal::NONE;
    }
    
    turn_signal_pub->publish(tsc_msg);
  }

  void callbackDbwEnabled(const std_msgs::msg::Bool::SharedPtr msg)
  {
    RCLCPP_DEBUG(get_logger(), "DBW enabled status changed: %s", dbw_enabled_ ? "ENABLED" : "DISABLED");

    dbw_enabled_ = msg->data;
    
    // Publish control mode based on DBW status
    autoware_vehicle_msgs::msg::ControlModeReport mode_report;
    mode_report.stamp = now();
    mode_report.mode = dbw_enabled_ ? 
        autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS :
        autoware_vehicle_msgs::msg::ControlModeReport::MANUAL;
    control_mode_report_pub->publish(mode_report);
  }

  void callbackEmergencyCmd(const tier4_vehicle_msgs::msg::VehicleEmergencyStamped::SharedPtr msg) 
  {
    RCLCPP_DEBUG(get_logger(), "Emergency command received: %s", msg->emergency ? "ACTIVE" : "INACTIVE");
    
    if (msg->emergency) {
      // Send emergency brake command via ULC
      ds_dbw_msgs::msg::UlcCmd emergency_cmd;
      emergency_cmd.header.stamp = now();
      emergency_cmd.cmd_type = ds_dbw_msgs::msg::UlcCmd::CMD_ACCEL;
      emergency_cmd.cmd = -0.5;  // Negative acceleration for braking
      emergency_cmd.enable = true;
      emergency_cmd.clear = false;
      ulc_pub->publish(emergency_cmd);

      // Update actuation status (for emergency)
      current_accel_cmd_ = 0.0;
      current_brake_cmd_ = 0.5;
      
      RCLCPP_WARN(get_logger(), "Emergency brake applied via ULC");
    }
  }
  
  void onControlModeRequest(
      const autoware_vehicle_msgs::srv::ControlModeCommand::Request::SharedPtr request,
      const autoware_vehicle_msgs::srv::ControlModeCommand::Response::SharedPtr response)
  {
    RCLCPP_DEBUG(get_logger(), "Control mode request: %d", request->mode);
    
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

  void callbackGearCmd(const autoware_vehicle_msgs::msg::GearCommand::SharedPtr msg)
  {
    RCLCPP_DEBUG(get_logger(), "Gear command: %d", msg->command);
    
    ds_dbw_msgs::msg::GearCmd ford_msg;
    ford_msg.cmd.value = translateGearAutowareToDS(msg->command);
    gear_pub->publish(ford_msg);
  }

  void callbackControlCmd(const autoware_control_msgs::msg::Control::SharedPtr msg)
  {
    // Extract actuation info from control command
    // Store steering command
    current_steer_cmd_ = msg->lateral.steering_tire_angle;
    
    // Simple velocity-based accel/brake estimation
    static double prev_velocity_cmd = 0.0;
    double velocity_cmd = msg->longitudinal.velocity;
    double velocity_diff = velocity_cmd - prev_velocity_cmd;
    
    if (velocity_diff > 0.05) {
        // Accelerating
        current_accel_cmd_ = std::min(1.0, velocity_diff * 5.0);
        current_brake_cmd_ = 0.0;
    } else if (velocity_diff < -0.05) {
        // Braking
        current_accel_cmd_ = 0.0;
        current_brake_cmd_ = std::min(1.0, -velocity_diff * 5.0);
    } else {
        // Maintaining - gradual decay
        current_accel_cmd_ *= 0.95;
        current_brake_cmd_ *= 0.95;
    }
    prev_velocity_cmd = velocity_cmd;

    // Longitudinal control via ULC
    ds_dbw_msgs::msg::UlcCmd ulc_cmd;
    ulc_cmd.header.stamp = now();
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
    ulc_pub->publish(ulc_cmd);

    // Lateral control via steering
    ds_dbw_msgs::msg::SteeringCmd steering_msg;
    double steering_angle_degrees = steering_gain * msg->lateral.steering_tire_angle * (180.0 / M_PI);
    steering_msg.cmd = steering_angle_degrees;

    steering_msg.cmd_type = ds_dbw_msgs::msg::SteeringCmd::CMD_ANGLE;
    steering_msg.cmd_rate = 0.0;
    steering_msg.cmd_accel = 0.0;
    steering_msg.enable = true;
    steering_pub->publish(steering_msg);
  }

  void callbackGearReport(const ds_dbw_msgs::msg::GearReport::SharedPtr msg)
  {
    autoware_vehicle_msgs::msg::GearReport aw_report;
    aw_report.stamp = msg->header.stamp;
    aw_report.report = translateGearDSToAutoware(msg->gear.value);
    gear_report_pub->publish(aw_report);
  }

  void callbackSteeringReport(const ds_dbw_msgs::msg::SteeringReport::SharedPtr msg)
  {
    autoware_vehicle_msgs::msg::SteeringReport aw_steering_report;
    aw_steering_report.stamp = msg->header.stamp;
    curr_steer = msg->steering_wheel_angle * (M_PI / 180.0);
    aw_steering_report.steering_tire_angle = curr_steer / steering_gain;
    steering_report_pub->publish(aw_steering_report);

    // Publish Actuation Status
    publishActuationStatus(msg->header.stamp);    
  }

  void callbackTurnSignalReport(const ds_dbw_msgs::msg::TurnSignalReport::SharedPtr msg)
  {
    using ds_dbw_msgs::msg::TurnSignal;
    using autoware_vehicle_msgs::msg::HazardLightsReport;
    using autoware_vehicle_msgs::msg::TurnIndicatorsReport;

    autoware_vehicle_msgs::msg::HazardLightsReport hazardReport;
    if (msg->cmd.value == TurnSignal::HAZARD) {
      hazardReport.report = HazardLightsReport::ENABLE;
    } else {
      hazardReport.report = HazardLightsReport::DISABLE;
    }
    
    autoware_vehicle_msgs::msg::TurnIndicatorsReport turnIndicatorsReport;
    if (msg->cmd.value == TurnSignal::LEFT) {
      turnIndicatorsReport.report = TurnIndicatorsReport::ENABLE_LEFT;
    } else if (msg->cmd.value == TurnSignal::RIGHT) {
      turnIndicatorsReport.report = TurnIndicatorsReport::ENABLE_RIGHT;
    } else {
      turnIndicatorsReport.report = TurnIndicatorsReport::DISABLE;
    }
    
    hazardReport.stamp = now();
    hazard_lights_report_pub->publish(hazardReport);

    turnIndicatorsReport.stamp = now();
    turn_indicators_report_pub->publish(turnIndicatorsReport);
  }

  void callbackVehicleTwist(const ds_dbw_msgs::msg::VehicleVelocity::SharedPtr msg)
  {
    double longitudinal_velocity = msg->vehicle_velocity_propulsion;
    
    autoware_vehicle_msgs::msg::VelocityReport velocity_msg;
    velocity_msg.header.frame_id = "base_link";
    velocity_msg.header.stamp = msg->header.stamp;
    velocity_msg.longitudinal_velocity = longitudinal_velocity;
    velocity_msg.heading_rate = longitudinal_velocity * std::tan(curr_steer) / wheel_base;
    velocity_report_pub->publish(velocity_msg);

    // Publish Actuation Status
    publishActuationStatus(msg->header.stamp);
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
