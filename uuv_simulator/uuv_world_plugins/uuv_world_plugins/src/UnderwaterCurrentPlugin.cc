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

/// \file UnderwaterCurrentPlugin.cc
/// \brief Plugin that for the underwater world

#include <uuv_world_plugins/UnderwaterCurrentPlugin.hh>

#include <ignition/physics/Vector3.hh>

#include <rclcpp/rclcpp.hpp>

namespace uuv_world_plugins
{

UnderwaterCurrentPlugin::UnderwaterCurrentPlugin()
: rosPublishPeriod(0.05)
{
}

UnderwaterCurrentPlugin::~UnderwaterCurrentPlugin()
{
  if (rosPublishConnection) {
    rosPublishConnection.reset();
  }
}

void UnderwaterCurrentPlugin::OnConfigure(const ignition::gazebo::ConfigureInfo &info)
{
  (void)info;
  this->currentVelocityTopic = "/world/current/flow_velocity";
  this->ns = "";

  this->rosNode = std::make_shared<rclcpp::Node>("underwater_current_plugin");

  // Initialize current velocity model
  this->currentVelModel.Init(
    this->rosNode,
    this->ns + "current_velocity_model",
    0.1,  // initial variance
    0.01,  // process noise
    0.001,  // measurement noise
    0.0,  // initial value
    0.0,  // initial mean
    0.0,  // initial std dev
    0.0,  // initial drift
    0.0,  // initial autoregressive coeff
    0.0,  // initial drift time constant
    0.0  // initial measurement noise
  );

  // Initialize horizontal angle model
  this->currentHorzAngleModel.Init(
    this->rosNode,
    this->ns + "current_horizontal_angle_model",
    0.1,  // initial variance
    0.01,  // process noise
    0.001,  // measurement noise
    0.0,  // initial value
    0.0,  // initial mean
    0.0,  // initial std dev
    0.0,  // initial drift
    0.0,  // initial autoregressive coeff
    0.0,  // initial drift time constant
    0.0  // initial measurement noise
  );

  // Initialize vertical angle model
  this->currentVertAngleModel.Init(
    this->rosNode,
    this->ns + "current_vertical_angle_model",
    0.1,  // initial variance
    0.01,  // process noise
    0.001,  // measurement noise
    0.0,  // initial value
    0.0,  // initial mean
    0.0,  // initial std dev
    0.0,  // initial drift
    0.0,  // initial autoregressive coeff
    0.0,  // initial drift time constant
    0.0  // initial measurement noise
  );

  // Initialize current velocity
  this->currentVelocity = ignition::math::Vector3d(0.0, 0.0, 0.0);

  // Create publisher for flow velocity
  this->flowVelocityPub = this->rosNode->create_publisher<geometry_msgs::msg::TwistStamped>(
    this->currentVelocityTopic, 10);

  // Create service servers
  this->worldServices["update_current_velocity_model"] =
    this->rosNode->create_service<uuv_world_ros_plugins_msgs::srv::SetCurrentModel>(
    "~/update_current_velocity_model",
    std::bind(&UnderwaterCurrentPlugin::UpdateCurrentVelocityModel, this,
      std::placeholders::_1, std::placeholders::_2));

  this->worldServices["get_current_velocity_model"] =
    this->rosNode->create_service<uuv_world_ros_plugins_msgs::srv::GetCurrentModel>(
    "~/get_current_velocity_model",
    std::bind(&UnderwaterCurrentPlugin::GetCurrentVelocityModel, this,
      std::placeholders::_1, std::placeholders::_2));

  this->worldServices["update_current_horizontal_angle_model"] =
    this->rosNode->create_service<uuv_world_ros_plugins_msgs::srv::SetCurrentModel>(
    "~/update_current_horizontal_angle_model",
    std::bind(&UnderwaterCurrentPlugin::UpdateCurrentHorzAngleModel, this,
      std::placeholders::_1, std::placeholders::_2));

  this->worldServices["get_current_horizontal_angle_model"] =
    this->rosNode->create_service<uuv_world_ros_plugins_msgs::srv::GetCurrentModel>(
    "~/get_current_horizontal_angle_model",
    std::bind(&UnderwaterCurrentPlugin::GetCurrentHorzAngleModel, this,
      std::placeholders::_1, std::placeholders::_2));

  this->worldServices["update_current_vertical_angle_model"] =
    this->rosNode->create_service<uuv_world_ros_plugins_msgs::srv::SetCurrentModel>(
    "~/update_current_vertical_angle_model",
    std::bind(&UnderwaterCurrentPlugin::UpdateCurrentVertAngleModel, this,
      std::placeholders::_1, std::placeholders::_2));

  this->worldServices["get_current_vertical_angle_model"] =
    this->rosNode->create_service<uuv_world_ros_plugins_msgs::srv::GetCurrentModel>(
    "~/get_current_vertical_angle_model",
    std::bind(&UnderwaterCurrentPlugin::GetCurrentVertAngleModel, this,
      std::placeholders::_1, std::placeholders::_2));

  this->worldServices["update_current_velocity"] =
    this->rosNode->create_service<uuv_world_ros_plugins_msgs::srv::SetCurrentVelocity>(
    "~/update_current_velocity",
    std::bind(&UnderwaterCurrentPlugin::UpdateCurrentVelocity, this,
      std::placeholders::_1, std::placeholders::_2));

  this->worldServices["update_current_horizontal_angle"] =
    this->rosNode->create_service<uuv_world_ros_plugins_msgs::srv::SetCurrentDirection>(
    "~/update_current_horizontal_angle",
    std::bind(&UnderwaterCurrentPlugin::UpdateHorzAngle, this,
      std::placeholders::_1, std::placeholders::_2));

  this->worldServices["update_current_vertical_angle"] =
    this->rosNode->create_service<uuv_world_ros_plugins_msgs::srv::SetCurrentDirection>(
    "~/update_current_vertical_angle",
    std::bind(&UnderwaterCurrentPlugin::UpdateVertAngle, this,
      std::placeholders::_1, std::placeholders::_2));

  // Set up ROS publishing
  this->rosPublishConnection =
    ignition::gazebo::event::Events::ConnectSimulationPeriod(
    std::bind(&UnderwaterCurrentPlugin::OnUpdateCurrentVel, this));
}

void UnderwaterCurrentPlugin::OnUpdate(const ignition::gazebo::UpdateInfo &info)
{
  // Update the current velocity model
  this->currentVelModel.Update(info.simTime.Double());
  this->currentHorzAngleModel.Update(info.simTime.Double());
  this->currentVertAngleModel.Update(info.simTime.Double());

  // Get the current velocity and direction
  double vel = this->currentVelModel.GetMean();
  double horzAngle = this->currentHorzAngleModel.GetMean();
  double vertAngle = this->currentVertAngleModel.GetMean();

  // Convert spherical coordinates to Cartesian
  this->currentVelocity.X() = vel * cos(vertAngle) * cos(horzAngle);
  this->currentVelocity.Y() = vel * cos(vertAngle) * sin(horzAngle);
  this->currentVelocity.Z() = vel * sin(vertAngle);
}

void UnderwaterCurrentPlugin::Update(const ignition::gazebo::UpdateInfo &_info)
{
  (void)_info;
}

void UnderwaterCurrentPlugin::PublishCurrentVelocity()
{
  geometry_msgs::msg::TwistStamped twistStamped;
  twistStamped.header.stamp = rclcpp::Clock().now();
  twistStamped.header.frame_id = "world";
  twistStamped.twist.linear.x = this->currentVelocity.X();
  twistStamped.twist.linear.y = this->currentVelocity.Y();
  twistStamped.twist.linear.z = this->currentVelocity.Z();
  twistStamped.twist.angular.x = 0.0;
  twistStamped.twist.angular.y = 0.0;
  twistStamped.twist.angular.z = 0.0;

  this->flowVelocityPub->publish(twistStamped);
}

void UnderwaterCurrentPlugin::OnUpdateCurrentVel()
{
  // Check if we should publish based on the period
  static ignition::gazebo::Time lastPublishTime = ignition::gazebo::Time(0);
  ignition::gazebo::Time now = ignition::gazebo::clock::get_clock()->now();

  if ((now - lastPublishTime).Double() >= this->rosPublishPeriod) {
    this->PublishCurrentVelocity();
    lastPublishTime = now;
  }
}

bool UnderwaterCurrentPlugin::UpdateCurrentVelocityModel(
  const uuv_world_ros_plugins_msgs::srv::SetCurrentModel::Request::SharedPtr _req,
  uuv_world_ros_plugins_msgs::srv::SetCurrentModel::Response::SharedPtr _res)
{
  RCLCPP_INFO(this->rosNode->get_logger(), "Updating current velocity model");

  // Update the current velocity model parameters
  this->currentVelModel.SetMean(_req->mean);
  this->currentVelModel.SetStdDev(_req->std_dev);
  this->currentVelModel.SetDrift(_req->drift);
  this->currentVelModel.SetAutoregressiveCoeff(_req->autoregressive_coeff);
  this->currentVelModel.SetDriftTimeConstant(_req->drift_time_constant);

  _res->success = true;
  return true;
}

bool UnderwaterCurrentPlugin::GetCurrentVelocityModel(
  const uuv_world_ros_plugins_msgs::srv::GetCurrentModel::Request::SharedPtr _req,
  uuv_world_ros_plugins_msgs::srv::GetCurrentModel::Response::SharedPtr _res)
{
  (void)_req;
  RCLCPP_INFO(this->rosNode->get_logger(), "Getting current velocity model");

  _res->mean = this->currentVelModel.GetMean();
  _res->std_dev = this->currentVelModel.GetStdDev();
  _res->drift = this->currentVelModel.GetDrift();
  _res->autoregressive_coeff = this->currentVelModel.GetAutoregressiveCoeff();
  _res->drift_time_constant = this->currentVelModel.GetDriftTimeConstant();
  _res->success = true;
  return true;
}

bool UnderwaterCurrentPlugin::UpdateCurrentHorzAngleModel(
  const uuv_world_ros_plugins_msgs::srv::SetCurrentModel::Request::SharedPtr _req,
  uuv_world_ros_plugins_msgs::srv::SetCurrentModel::Response::SharedPtr _res)
{
  RCLCPP_INFO(this->rosNode->get_logger(), "Updating current horizontal angle model");

  this->currentHorzAngleModel.SetMean(_req->mean);
  this->currentHorzAngleModel.SetStdDev(_req->std_dev);
  this->currentHorzAngleModel.SetDrift(_req->drift);
  this->currentHorzAngleModel.SetAutoregressiveCoeff(_req->autoregressive_coeff);
  this->currentHorzAngleModel.SetDriftTimeConstant(_req->drift_time_constant);

  _res->success = true;
  return true;
}

bool UnderwaterCurrentPlugin::GetCurrentHorzAngleModel(
  const uuv_world_ros_plugins_msgs::srv::GetCurrentModel::Request::SharedPtr _req,
  uuv_world_ros_plugins_msgs::srv::GetCurrentModel::Response::SharedPtr _res)
{
  (void)_req;
  RCLCPP_INFO(this->rosNode->get_logger(), "Getting current horizontal angle model");

  _res->mean = this->currentHorzAngleModel.GetMean();
  _res->std_dev = this->currentHorzAngleModel.GetStdDev();
  _res->drift = this->currentHorzAngleModel.GetDrift();
  _res->autoregressive_coeff = this->currentHorzAngleModel.GetAutoregressiveCoeff();
  _res->drift_time_constant = this->currentHorzAngleModel.GetDriftTimeConstant();
  _res->success = true;
  return true;
}

bool UnderwaterCurrentPlugin::UpdateCurrentVertAngleModel(
  const uuv_world_ros_plugins_msgs::srv::SetCurrentModel::Request::SharedPtr _req,
  uuv_world_ros_plugins_msgs::srv::SetCurrentModel::Response::SharedPtr _res)
{
  RCLCPP_INFO(this->rosNode->get_logger(), "Updating current vertical angle model");

  this->currentVertAngleModel.SetMean(_req->mean);
  this->currentVertAngleModel.SetStdDev(_req->std_dev);
  this->currentVertAngleModel.SetDrift(_req->drift);
  this->currentVertAngleModel.SetAutoregressiveCoeff(_req->autoregressive_coeff);
  this->currentVertAngleModel.SetDriftTimeConstant(_req->drift_time_constant);

  _res->success = true;
  return true;
}

bool UnderwaterCurrentPlugin::GetCurrentVertAngleModel(
  const uuv_world_ros_plugins_msgs::srv::GetCurrentModel::Request::SharedPtr _req,
  uuv_world_ros_plugins_msgs::srv::GetCurrentModel::Response::SharedPtr _res)
{
  (void)_req;
  RCLCPP_INFO(this->rosNode->get_logger(), "Getting current vertical angle model");

  _res->mean = this->currentVertAngleModel.GetMean();
  _res->std_dev = this->currentVertAngleModel.GetStdDev();
  _res->drift = this->currentVertAngleModel.GetDrift();
  _res->autoregressive_coeff = this->currentVertAngleModel.GetAutoregressiveCoeff();
  _res->drift_time_constant = this->currentVertAngleModel.GetDriftTimeConstant();
  _res->success = true;
  return true;
}

bool UnderwaterCurrentPlugin::UpdateCurrentVelocity(
  const uuv_world_ros_plugins_msgs::srv::SetCurrentVelocity::Request::SharedPtr _req,
  uuv_world_ros_plugins_msgs::srv::SetCurrentVelocity::Response::SharedPtr _res)
{
  RCLCPP_INFO(this->rosNode->get_logger(), "Updating current velocity");

  this->currentVelocity.X() = _req->linear.x;
  this->currentVelocity.Y() = _req->linear.y;
  this->currentVelocity.Z() = _req->linear.z;

  _res->success = true;
  return true;
}

bool UnderwaterCurrentPlugin::UpdateHorzAngle(
  const uuv_world_ros_plugins_msgs::srv::SetCurrentDirection::Request::SharedPtr _req,
  uuv_world_ros_plugins_msgs::srv::SetCurrentDirection::Response::SharedPtr _res)
{
  RCLCPP_INFO(this->rosNode->get_logger(), "Updating horizontal angle");

  this->currentHorzAngleModel.SetMean(_req->angle);

  _res->success = true;
  return true;
}

bool UnderwaterCurrentPlugin::UpdateVertAngle(
  const uuv_world_ros_plugins_msgs::srv::SetCurrentDirection::Request::SharedPtr _req,
  uuv_world_ros_plugins_msgs::srv::SetCurrentDirection::Response::SharedPtr _res)
{
  RCLCPP_INFO(this->rosNode->get_logger(), "Updating vertical angle");

  this->currentVertAngleModel.SetMean(_req->angle);

  _res->success = true;
  return true;
}

// Register the plugin
GZ_ADD_PLUGIN(uuv_world_plugins::UnderwaterCurrentPlugin,
              ignition::gazebo::SystemPlugin,
              ignition::gazebo::SystemConfigure,
              ignition::gazebo::SystemUpdate)

}  // namespace uuv_world_plugins
