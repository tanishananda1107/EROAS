// Copyright (c) 2016 The UUV Simulator Authors.
// Licensed under the Apache License, Version 2.0.

/// \file JointStatePublisher.hh  Gz-ROS2 plugin for publishing joint states.

#ifndef __JOINT_STATE_PUBLISHER_HH__
#define __JOINT_STATE_PUBLISHER_HH__

#include <string>
#include <vector>

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace uuv_simulator_ros
{
class JointStatePublisher :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPostUpdate
{
public:
  JointStatePublisher();
  ~JointStatePublisher();

  void Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &_eventMgr) override;

  void PostUpdate(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm) override;

  void PublishJointStates(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm);

private:
  bool IsIgnoredJoint(const std::string &_jointName) const;

  gz::sim::Entity modelEntity{gz::sim::kNullEntity};
  gz::sim::Model model;

  std::shared_ptr<rclcpp::Node> node;
  std::string robotNamespace;
  std::vector<std::string> movingJoints;

  double updateRate{0.0};
  std::chrono::steady_clock::duration updatePeriod{0};
  std::chrono::steady_clock::duration lastUpdate{0};

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointStatePub;
};
}

#endif  // __JOINT_STATE_PUBLISHER_HH__
