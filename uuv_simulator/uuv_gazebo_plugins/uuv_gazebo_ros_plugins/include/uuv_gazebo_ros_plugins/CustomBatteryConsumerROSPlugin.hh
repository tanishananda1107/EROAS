// Copyright (c) 2016 The UUV Simulator Authors.
// Licensed under the Apache License, Version 2.0.

#ifndef __LINEAR_BATTERY_CONSUMER_ROS_PLUGIN_HH__
#define __LINEAR_BATTERY_CONSUMER_ROS_PLUGIN_HH__

#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

namespace gz::sim::systems
{
class CustomBatteryConsumerROSPlugin :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemUpdate
{
public:
  CustomBatteryConsumerROSPlugin();
  virtual ~CustomBatteryConsumerROSPlugin();

  void Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &_eventMgr) override;

  void Update(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm) override;

protected:
  void UpdateDeviceState(const std_msgs::msg::Bool::SharedPtr _msg);
  void UpdatePowerLoad(double _powerLoad = 0.0);

  std::shared_ptr<rclcpp::Node> rosNode;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr deviceStateSub;

  bool isDeviceOn{false};
  double powerLoad{0.0};
  int consumerID{-1};

  std::string linkName;
  std::string batteryName;

  gz::sim::Entity modelEntity{gz::sim::kNullEntity};
};
}

#endif  // __LINEAR_BATTERY_CONSUMER_ROS_PLUGIN_HH__
