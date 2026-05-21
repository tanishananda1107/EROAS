// Copyright (c) 2016 The UUV Simulator Authors.
// All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// \file JointStatePublisher.hh A Gazebo ROS plugin for publishing the joint
/// states of a robot (position, velocity and effort). Build similar to the
/// class in the GazeboRosJointStatePublisher, but including more information

#ifndef __JOINT_STATE_PUBLISHER_HH__
#define __JOINT_STATE_PUBLISHER_HH__

#include <memory>
#include <string>
#include <vector>

#include <gz/sim/System.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/World.hh>
#include <gz/msgs/joint_state.pb.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace uuv_simulator_ros
{
class JointStatePublisher : public gz::sim::System,
                            public gz::sim::ISystemConfigure,
                            public gz::sim::ISystemUpdate
{
  public: JointStatePublisher();

  public: ~JointStatePublisher() override;

  public: void Configure(const gz::sim::Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         gz::sim::EntityComponentManager &_ecm,
                         gz::sim::EventManager &_eventManager) override;

  public: void Update(const gz::sim::UpdateInfo &_info,
                      gz::sim::EntityComponentManager &_ecm) override;

  public: void PublishJointStates(const gz::sim::EntityComponentManager &_ecm);

  private: bool IsIgnoredJoint(const std::string &_jointName);

  private: std::string robotNamespace;

  private: std::vector<std::string> movingJoints;

  private: double updateRate;

  private: double updatePeriod;

  private: rclcpp::Time lastUpdate;

  private: std::shared_ptr<rclcpp::Node> node;

  private: rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointStatePub;

  private: gz::sim::Entity modelEntity;
};
}

#endif  // __JOINT_STATE_PUBLISHER_HH__
