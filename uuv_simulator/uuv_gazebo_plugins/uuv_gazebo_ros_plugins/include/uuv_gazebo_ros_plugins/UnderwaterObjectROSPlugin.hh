// Copyright (c) 2016 The UUV Simulator Authors.
// Licensed under the Apache License, Version 2.0.

/// \file UnderwaterObjectROSPlugin.hh  Publishes underwater object's
/// Gazebo Harmonic topics and parameters into ROS2 standards.

#ifndef __UNDERWATER_OBJECT_ROS_PLUGIN_HH__
#define __UNDERWATER_OBJECT_ROS_PLUGIN_HH__

#include <string>
#include <map>

#include <uuv_gazebo_plugins/UnderwaterObjectPlugin.hh>

#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <uuv_gazebo_ros_plugins_msgs/srv/set_use_global_current_vel.hpp>
#include <uuv_gazebo_ros_plugins_msgs/msg/underwater_object_model.hpp>
#include <uuv_gazebo_ros_plugins_msgs/srv/get_model_properties.hpp>
#include <uuv_gazebo_ros_plugins_msgs/srv/set_float.hpp>
#include <uuv_gazebo_ros_plugins_msgs/srv/get_float.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

namespace uuv_simulator_ros
{
class UnderwaterObjectROSPlugin : public gazebo::UnderwaterObjectPlugin
{
public:
  UnderwaterObjectROSPlugin();
  virtual ~UnderwaterObjectROSPlugin();

  void Load(gz::sim::EntityComponentManager &_ecm,
            const std::shared_ptr<const sdf::Element> &_sdf);

  virtual void Init();
  virtual void Reset();
  virtual void Update(const gz::sim::UpdateInfo &_info,
                      gz::sim::EntityComponentManager &_ecm);

  void UpdateLocalCurrentVelocity(
    const geometry_msgs::msg::Vector3::SharedPtr &_msg);

  bool SetUseGlobalCurrentVel(
    uuv_gazebo_ros_plugins_msgs::srv::SetUseGlobalCurrentVel::Request::SharedPtr _req,
    uuv_gazebo_ros_plugins_msgs::srv::SetUseGlobalCurrentVel::Response::SharedPtr _res);

  bool GetModelProperties(
    uuv_gazebo_ros_plugins_msgs::srv::GetModelProperties::Request::SharedPtr _req,
    uuv_gazebo_ros_plugins_msgs::srv::GetModelProperties::Response::SharedPtr _res);

  // Scaling / offset setters and getters for added mass, damping, volume, fluid density
  bool SetScalingAddedMass(
    uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Request::SharedPtr _req,
    uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Response::SharedPtr _res);
  bool GetScalingAddedMass(
    uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Request::SharedPtr _req,
    uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Response::SharedPtr _res);

  bool SetScalingDamping(
    uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Request::SharedPtr _req,
    uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Response::SharedPtr _res);
  bool GetScalingDamping(
    uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Request::SharedPtr _req,
    uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Response::SharedPtr _res);

  bool SetScalingVolume(
    uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Request::SharedPtr _req,
    uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Response::SharedPtr _res);
  bool GetScalingVolume(
    uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Request::SharedPtr _req,
    uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Response::SharedPtr _res);

  bool SetFluidDensity(
    uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Request::SharedPtr _req,
    uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Response::SharedPtr _res);
  bool GetFluidDensity(
    uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Request::SharedPtr _req,
    uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Response::SharedPtr _res);

  bool SetOffsetVolume(
    uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Request::SharedPtr _req,
    uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Response::SharedPtr _res);
  bool GetOffsetVolume(
    uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Request::SharedPtr _req,
    uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Response::SharedPtr _res);

  bool SetOffsetAddedMass(
    uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Request::SharedPtr _req,
    uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Response::SharedPtr _res);
  bool GetOffsetAddedMass(
    uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Request::SharedPtr _req,
    uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Response::SharedPtr _res);

  bool SetOffsetLinearDamping(
    uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Request::SharedPtr _req,
    uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Response::SharedPtr _res);
  bool GetOffsetLinearDamping(
    uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Request::SharedPtr _req,
    uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Response::SharedPtr _res);

  bool SetOffsetLinearForwardSpeedDamping(
    uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Request::SharedPtr _req,
    uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Response::SharedPtr _res);
  bool GetOffsetLinearForwardSpeedDamping(
    uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Request::SharedPtr _req,
    uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Response::SharedPtr _res);

  bool SetOffsetNonLinearDamping(
    uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Request::SharedPtr _req,
    uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Response::SharedPtr _res);
  bool GetOffsetNonLinearDamping(
    uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Request::SharedPtr _req,
    uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Response::SharedPtr _res);

protected:
  virtual void PublishRestoringForce(gz::sim::Entity _linkEntity,
    gz::sim::EntityComponentManager &_ecm);

  virtual void PublishHydrodynamicWrenches(gz::sim::Entity _linkEntity,
    gz::sim::EntityComponentManager &_ecm);

  virtual void GenWrenchMsg(
    gz::math::Vector3d _force, gz::math::Vector3d _torque,
    geometry_msgs::msg::WrenchStamped &_output);

  virtual void InitDebug(gz::sim::Entity _linkEntity,
    gz::sim::EntityComponentManager &_ecm,
    gazebo::HydrodynamicModelPtr _hydro);

  virtual void PublishCurrentVelocityMarker();
  virtual void PublishIsSubmerged();

private:
  std::shared_ptr<rclcpp::Node> rosNode;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr subLocalCurVel;
  std::map<std::string, rclcpp::PublisherBase::SharedPtr> rosHydroPub;
  std::map<std::string, rclcpp::ServiceBase::SharedPtr> services;

  geometry_msgs::msg::TransformStamped nedTransform;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
};
}

#endif  // __UNDERWATER_OBJECT_ROS_PLUGIN_HH__
