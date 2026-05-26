// Copyright (c) 2016 The UUV Simulator Authors.
// Licensed under the Apache License, Version 2.0.

#ifndef __UUV_GAZEBO_PLUGINS_ACCELERATIONS_TEST_PLUGIN_H__
#define __UUV_GAZEBO_PLUGINS_ACCELERATIONS_TEST_PLUGIN_H__

#include <map>
#include <string>

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <uuv_gazebo_plugins/HydrodynamicModel.hh>
#include <uuv_gazebo_plugins/Def.hh>

namespace gz::sim::systems
{
class AccelerationsTestPlugin :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemUpdate
{
public:
  AccelerationsTestPlugin();
  virtual ~AccelerationsTestPlugin();

  void Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &_eventMgr) override;

  void PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm) override;

  void Update(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm) override;

private:
  std::shared_ptr<rclcpp::Node> rosNode;

  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_accel_b_gazebo;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_accel_b_numeric;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_accel_w_gazebo;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_accel_w_numeric;

  Eigen::Matrix<double, 6, 1> last_w_v_w_b;
  std::chrono::steady_clock::duration lastTime{0};

  gz::sim::Entity modelEntity{gz::sim::kNullEntity};
  gz::sim::Entity linkEntity{gz::sim::kNullEntity};
};
}

#endif  // __UUV_GAZEBO_PLUGINS_ACCELERATIONS_TEST_PLUGIN_H__
