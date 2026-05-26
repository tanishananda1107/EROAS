// Copyright (c) 2016 The UUV Simulator Authors.
// Licensed under the Apache License, Version 2.0.

#ifndef __LINEAR_BATTERY_ROS_PLUGIN_HH__
#define __LINEAR_BATTERY_ROS_PLUGIN_HH__

#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <string>

namespace gz::sim::systems
{
class LinearBatteryROSPlugin :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemUpdate
{
public:
  LinearBatteryROSPlugin();
  virtual ~LinearBatteryROSPlugin();

  void Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &_eventMgr) override;

  void Update(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm) override;

  virtual void Init();
  virtual void Reset();

protected:
  void PublishBatteryState();

  std::shared_ptr<rclcpp::Node> rosNode;
  std::string robotNamespace;

  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr batteryStatePub;
  sensor_msgs::msg::BatteryState batteryStateMsg;

  rclcpp::TimerBase::SharedPtr updateTimer;
  gz::sim::Entity modelEntity{gz::sim::kNullEntity};
  gz::sim::Entity batteryEntity{gz::sim::kNullEntity};
};
}

#endif  // __LINEAR_BATTERY_ROS_PLUGIN_HH__
