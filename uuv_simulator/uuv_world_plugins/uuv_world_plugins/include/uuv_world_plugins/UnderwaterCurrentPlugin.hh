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

/// \file UnderwaterCurrentPlugin.hh
/// \brief Plugin that for the underwater world

#ifndef UUV_WORLD_PLUGINS__UNDERWATER_CURRENT_PLUGIN_HH__
#define UUV_WORLD_PLUGINS__UNDERWATER_CURRENT_PLUGIN_HH__

#include <map>
#include <cmath>
#include <string>
#include <memory>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Component.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/Events.hh>
#include <sdf/sdf.hh>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <uuv_world_ros_plugins_msgs/srv/set_current_model.hpp>
#include <uuv_world_ros_plugins_msgs/srv/get_current_model.hpp>
#include <uuv_world_ros_plugins_msgs/srv/set_current_velocity.hpp>
#include <uuv_world_ros_plugins_msgs/srv/set_current_direction.hpp>

#include <uuv_world_plugins/GaussMarkovProcess.hh>

namespace uuv_world_plugins
{

/// \brief Class for the underwater current plugin
/// TODO: Add option to make the underwater current also a function of depth
///       to comply with DNV
class UnderwaterCurrentPlugin :
  public ignition::gazebo::SystemPlugin,
  public ignition::gazebo::SystemConfigure,
  public ignition::gazebo::SystemUpdate
{
  /// \brief Class constructor
  public: UnderwaterCurrentPlugin();

  /// \brief Class destructor
  public: ~UnderwaterCurrentPlugin() override;

  // Documentation inherited.
  public: void OnConfigure(const ignition::gazebo::ConfigureInfo &info) override;

  // Documentation inherited.
  public: void OnUpdate(const ignition::gazebo::UpdateInfo &info) override;

  /// \brief Update the simulation state.
  /// \param[in] _info Information used in the update event.
  protected: void Update(const ignition::gazebo::UpdateInfo &_info);

  /// \brief Publish current velocity and the pose of its frame
  protected: void PublishCurrentVelocity();

  /// \brief Update event
  protected: ignition::gazebo::SystemUpdateEvent lastUpdate;

  /// \brief Pointer to a node for communication
  protected: rclcpp::Node::SharedPtr rosNode;

  /// \brief Current velocity topic
  protected: std::string currentVelocityTopic;

  /// \brief Namespace for topics and services
  protected: std::string ns;

  /// \brief Gauss-Markov process instance for the current velocity
  protected: GaussMarkovProcess currentVelModel;

  /// \brief Gauss-Markov process instance for horizontal angle model
  protected: GaussMarkovProcess currentHorzAngleModel;

  /// \brief Gauss-Markov process instance for vertical angle model
  protected: GaussMarkovProcess currentVertAngleModel;

  /// \brief Last update time stamp
  protected: ignition::gazebo::Time lastRosPublishTime;

  /// \brief Current linear velocity vector
  protected: ignition::math::Vector3d currentVelocity;

  /// \brief Publisher for current velocity
  protected: rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr flowVelocityPub;

  /// \brief Service servers for current velocity model
  protected: std::map<std::string, rclcpp::ServiceBase::SharedPtr> worldServices;

  /// \brief ROS publish period
  protected: double rosPublishPeriod;

  /// \brief Connection for ROS publishing
  protected: ignition::gazebo::EventConnectionPtr rosPublishConnection;

  /// \brief Update current velocity model service callback
  protected: bool UpdateCurrentVelocityModel(
    const uuv_world_ros_plugins_msgs::srv::SetCurrentModel::Request::SharedPtr _req,
    uuv_world_ros_plugins_msgs::srv::SetCurrentModel::Response::SharedPtr _res);

  /// \brief Get current velocity model service callback
  protected: bool GetCurrentVelocityModel(
    const uuv_world_ros_plugins_msgs::srv::GetCurrentModel::Request::SharedPtr _req,
    uuv_world_ros_plugins_msgs::srv::GetCurrentModel::Response::SharedPtr _res);

  /// \brief Update horizontal angle model service callback
  protected: bool UpdateCurrentHorzAngleModel(
    const uuv_world_ros_plugins_msgs::srv::SetCurrentModel::Request::SharedPtr _req,
    uuv_world_ros_plugins_msgs::srv::SetCurrentModel::Response::SharedPtr _res);

  /// \brief Get horizontal angle model service callback
  protected: bool GetCurrentHorzAngleModel(
    const uuv_world_ros_plugins_msgs::srv::GetCurrentModel::Request::SharedPtr _req,
    uuv_world_ros_plugins_msgs::srv::GetCurrentModel::Response::SharedPtr _res);

  /// \brief Update vertical angle model service callback
  protected: bool UpdateCurrentVertAngleModel(
    const uuv_world_ros_plugins_msgs::srv::SetCurrentModel::Request::SharedPtr _req,
    uuv_world_ros_plugins_msgs::srv::SetCurrentModel::Response::SharedPtr _res);

  /// \brief Get vertical angle model service callback
  protected: bool GetCurrentVertAngleModel(
    const uuv_world_ros_plugins_msgs::srv::GetCurrentModel::Request::SharedPtr _req,
    uuv_world_ros_plugins_msgs::srv::GetCurrentModel::Response::SharedPtr _res);

  /// \brief Update current velocity service callback
  protected: bool UpdateCurrentVelocity(
    const uuv_world_ros_plugins_msgs::srv::SetCurrentVelocity::Request::SharedPtr _req,
    uuv_world_ros_plugins_msgs::srv::SetCurrentVelocity::Response::SharedPtr _res);

  /// \brief Update horizontal angle service callback
  protected: bool UpdateHorzAngle(
    const uuv_world_ros_plugins_msgs::srv::SetCurrentDirection::Request::SharedPtr _req,
    uuv_world_ros_plugins_msgs::srv::SetCurrentDirection::Response::SharedPtr _res);

  /// \brief Update vertical angle service callback
  protected: bool UpdateVertAngle(
    const uuv_world_ros_plugins_msgs::srv::SetCurrentDirection::Request::SharedPtr _req,
    uuv_world_ros_plugins_msgs::srv::SetCurrentDirection::Response::SharedPtr _res);

  /// \brief Publish current velocity to ROS
  protected: void OnUpdateCurrentVel();
};

}  // namespace uuv_world_plugins

#endif  // UUV_WORLD_PLUGINS__UNDERWATER_CURRENT_PLUGIN_HH__
