// Copyright (c) 2016 The UUV Simulator Authors.
// Licensed under the Apache License, Version 2.0.

#ifndef __THRUSTER_ROS_PLUGIN_HH__
#define __THRUSTER_ROS_PLUGIN_HH__

#include <map>
#include <string>
#include <vector>

#include <uuv_gazebo_plugins/ThrusterPlugin.hh>

#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>

#include <rclcpp/rclcpp.hpp>
#include <uuv_gazebo_ros_plugins_msgs/msg/float_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

#include <uuv_gazebo_ros_plugins_msgs/srv/set_thruster_state.hpp>
#include <uuv_gazebo_ros_plugins_msgs/srv/get_thruster_state.hpp>
#include <uuv_gazebo_ros_plugins_msgs/srv/set_thruster_efficiency.hpp>
#include <uuv_gazebo_ros_plugins_msgs/srv/get_thruster_efficiency.hpp>
#include <uuv_gazebo_ros_plugins_msgs/srv/get_thruster_conversion_fcn.hpp>

namespace uuv_simulator_ros
{
class ThrusterROSPlugin : public gazebo::ThrusterPlugin
{
public:
  ThrusterROSPlugin();
  ~ThrusterROSPlugin();

  void Load(gz::sim::EntityComponentManager &_ecm,
            const std::shared_ptr<const sdf::Element> &_sdf);

  void RosPublishStates();

  void SetThrustReference(
    const uuv_gazebo_ros_plugins_msgs::msg::FloatStamped::SharedPtr &_msg);

  std::chrono::nanoseconds GetRosPublishPeriod();
  void SetRosPublishRate(double _hz);

  virtual void Init();
  virtual void Reset();

  bool SetThrustForceEfficiency(
    uuv_gazebo_ros_plugins_msgs::srv::SetThrusterEfficiency::Request::SharedPtr _req,
    uuv_gazebo_ros_plugins_msgs::srv::SetThrusterEfficiency::Response::SharedPtr _res);

  bool GetThrustForceEfficiency(
    uuv_gazebo_ros_plugins_msgs::srv::GetThrusterEfficiency::Request::SharedPtr _req,
    uuv_gazebo_ros_plugins_msgs::srv::GetThrusterEfficiency::Response::SharedPtr _res);

  bool SetDynamicStateEfficiency(
    uuv_gazebo_ros_plugins_msgs::srv::SetThrusterEfficiency::Request::SharedPtr _req,
    uuv_gazebo_ros_plugins_msgs::srv::SetThrusterEfficiency::Response::SharedPtr _res);

  bool GetDynamicStateEfficiency(
    uuv_gazebo_ros_plugins_msgs::srv::GetThrusterEfficiency::Request::SharedPtr _req,
    uuv_gazebo_ros_plugins_msgs::srv::GetThrusterEfficiency::Response::SharedPtr _res);

  bool SetThrusterState(
    uuv_gazebo_ros_plugins_msgs::srv::SetThrusterState::Request::SharedPtr _req,
    uuv_gazebo_ros_plugins_msgs::srv::SetThrusterState::Response::SharedPtr _res);

  bool GetThrusterState(
    uuv_gazebo_ros_plugins_msgs::srv::GetThrusterState::Request::SharedPtr _req,
    uuv_gazebo_ros_plugins_msgs::srv::GetThrusterState::Response::SharedPtr _res);

  bool GetThrusterConversionFcn(
    uuv_gazebo_ros_plugins_msgs::srv::GetThrusterConversionFcn::Request::SharedPtr _req,
    uuv_gazebo_ros_plugins_msgs::srv::GetThrusterConversionFcn::Response::SharedPtr _res);

private:
  std::map<std::string, rclcpp::ServiceBase::SharedPtr> services;

  std::shared_ptr<rclcpp::Node> rosNode;

  rclcpp::Subscription<uuv_gazebo_ros_plugins_msgs::msg::FloatStamped>::SharedPtr subThrustReference;

  rclcpp::Publisher<uuv_gazebo_ros_plugins_msgs::msg::FloatStamped>::SharedPtr pubThrust;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pubThrustWrench;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pubThrusterState;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pubThrustForceEff;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pubDynamicStateEff;

  std::chrono::nanoseconds rosPublishPeriod{0};
  std::chrono::steady_clock::time_point lastRosPublishTime;
};
}

#endif  // __THRUSTER_ROS_PLUGIN_HH__
