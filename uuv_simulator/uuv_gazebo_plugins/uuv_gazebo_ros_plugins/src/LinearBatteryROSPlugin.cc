// Copyright (c) 2016 The UUV Simulator Authors.
// Licensed under the Apache License, Version 2.0.

#include <uuv_gazebo_ros_plugins/LinearBatteryROSPlugin.hh>

#include <gz/sim/components/BatterySoC.hh>
#include <gz/sim/components/Name.hh>
#include <gz/plugin/Register.hh>

namespace gz::sim::systems
{

/////////////////////////////////////////////////
LinearBatteryROSPlugin::LinearBatteryROSPlugin() = default;
LinearBatteryROSPlugin::~LinearBatteryROSPlugin() = default;

/////////////////////////////////////////////////
void LinearBatteryROSPlugin::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &_ecm,
  gz::sim::EventManager &_eventMgr)
{
  // Call parent Configure (LinearBatteryPlugin)
  // Note: gz-sim8's LinearBatteryPlugin is a System; call its Configure here
  // if it exposes one, or duplicate SDF parsing as needed.
  modelEntity = _entity;

  if (!rclcpp::ok())
    rclcpp::init(0, nullptr);

  robotNamespace = _sdf->HasElement("namespace") ?
    _sdf->Get<std::string>("namespace") : "";

  rosNode = std::make_shared<rclcpp::Node>("linear_battery_ros_plugin",
    robotNamespace);

  double updateRate = 2.0;
  if (_sdf->HasElement("update_rate"))
    updateRate = _sdf->Get<double>("update_rate");
  if (updateRate <= 0.0)
  {
    RCLCPP_WARN(rosNode->get_logger(),
      "Invalid update rate %.2f, defaulting to 2 Hz", updateRate);
    updateRate = 2.0;
  }

  batteryStatePub = rosNode->create_publisher<sensor_msgs::msg::BatteryState>(
    "battery_state", 0);

  updateTimer = rosNode->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / updateRate)),
    [this]() { PublishBatteryState(); });

  gzmsg << "ROS Battery Plugin initialized\n"
        << "\t- Update rate [Hz]=" << updateRate << std::endl;
}

/////////////////////////////////////////////////
void LinearBatteryROSPlugin::Update(
  const gz::sim::UpdateInfo &,
  gz::sim::EntityComponentManager &_ecm)
{
  // Retrieve current SoC from ECM battery component if available
  if (batteryEntity != gz::sim::kNullEntity)
  {
    auto soc = _ecm.Component<gz::sim::components::BatterySoC>(batteryEntity);
    if (soc)
      batteryStateMsg.percentage = static_cast<float>(soc->Data());
  }
  // Spin ros callbacks (timer fires PublishBatteryState)
  if (rclcpp::ok())
    rclcpp::spin_some(rosNode);
}

/////////////////////////////////////////////////
void LinearBatteryROSPlugin::PublishBatteryState()
{
  batteryStateMsg.header.stamp = rosNode->now();
  batteryStateMsg.power_supply_status =
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
  batteryStateMsg.power_supply_health =
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
  batteryStateMsg.power_supply_technology =
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
  batteryStateMsg.present = true;

  batteryStatePub->publish(batteryStateMsg);
}

/////////////////////////////////////////////////
void LinearBatteryROSPlugin::Init() {}
void LinearBatteryROSPlugin::Reset() {}

} // namespace gz::sim::systems

GZ_ADD_PLUGIN(gz::sim::systems::LinearBatteryROSPlugin,
              gz::sim::System,
              gz::sim::systems::LinearBatteryROSPlugin::ISystemConfigure,
              gz::sim::systems::LinearBatteryROSPlugin::ISystemUpdate)
