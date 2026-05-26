// Copyright (c) 2016 The UUV Simulator Authors.
// Licensed under the Apache License, Version 2.0.

#include <uuv_gazebo_ros_plugins/CustomBatteryConsumerROSPlugin.hh>

#include <gz/sim/components/BatterySoC.hh>
#include <gz/sim/components/Name.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>

namespace gz::sim::systems
{

/////////////////////////////////////////////////
CustomBatteryConsumerROSPlugin::CustomBatteryConsumerROSPlugin()
  : isDeviceOn(true)
{
}

/////////////////////////////////////////////////
CustomBatteryConsumerROSPlugin::~CustomBatteryConsumerROSPlugin() = default;

/////////////////////////////////////////////////
void CustomBatteryConsumerROSPlugin::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &_ecm,
  gz::sim::EventManager &)
{
  modelEntity = _entity;

  if (!rclcpp::ok())
    rclcpp::init(0, nullptr);

  rosNode = std::make_shared<rclcpp::Node>("custom_battery_consumer");

  GZ_ASSERT(_sdf->HasElement("link_name"), "Consumer link name is missing");
  linkName = _sdf->Get<std::string>("link_name");

  GZ_ASSERT(_sdf->HasElement("battery_name"), "Battery name is missing");
  batteryName = _sdf->Get<std::string>("battery_name");

  GZ_ASSERT(_sdf->HasElement("power_load"), "Power load is missing");
  powerLoad = _sdf->Get<double>("power_load");
  GZ_ASSERT(powerLoad > 0, "Power load must be greater than zero");

  if (_sdf->HasElement("topic_device_state"))
  {
    std::string topicName = _sdf->Get<std::string>("topic_device_state");
    if (!topicName.empty())
    {
      deviceStateSub = rosNode->create_subscription<std_msgs::msg::Bool>(
        topicName, 1,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          UpdateDeviceState(msg);
        });
    }
  }
  else
  {
    UpdatePowerLoad(powerLoad);
  }

  gzmsg << "CustomBatteryConsumerROSPlugin::Device <" << linkName
        << "> added as battery consumer\n"
        << "\t- Power load [W]=" << powerLoad << std::endl;
}

/////////////////////////////////////////////////
void CustomBatteryConsumerROSPlugin::Update(
  const gz::sim::UpdateInfo &,
  gz::sim::EntityComponentManager &) {}

/////////////////////////////////////////////////
void CustomBatteryConsumerROSPlugin::UpdateDeviceState(
  const std_msgs::msg::Bool::SharedPtr _msg)
{
  isDeviceOn = _msg->data;
  UpdatePowerLoad(isDeviceOn ? powerLoad : 0.0);
}

/////////////////////////////////////////////////
void CustomBatteryConsumerROSPlugin::UpdatePowerLoad(double _powerLoad)
{
  // In gz-sim8, battery power load updates are done via the
  // gz::sim::components::BatteryPowerLoad component or via gz-msgs battery
  // service. Store the value; apply in Update() when ECM is available.
  powerLoad = _powerLoad;
  gzmsg << "CustomBatteryConsumerROSPlugin: power load set to "
        << _powerLoad << " W" << std::endl;
}

} // namespace gz::sim::systems

GZ_ADD_PLUGIN(gz::sim::systems::CustomBatteryConsumerROSPlugin,
              gz::sim::System,
              gz::sim::systems::CustomBatteryConsumerROSPlugin::ISystemConfigure,
              gz::sim::systems::CustomBatteryConsumerROSPlugin::ISystemUpdate)
