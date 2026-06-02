#ifndef UUV_GAZEBO_ROS_PLUGINS_UNDERWATER_OBJECT_ROS_PLUGIN_HH_
#define UUV_GAZEBO_ROS_PLUGINS_UNDERWATER_OBJECT_ROS_PLUGIN_HH_

#include <map>
#include <memory>
#include <string>

#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <uuv_gazebo_plugins/UnderwaterObjectPlugin.hh>
#include <uuv_gazebo_ros_plugins_msgs/srv/get_float.hpp>
#include <uuv_gazebo_ros_plugins_msgs/srv/get_model_properties.hpp>
#include <uuv_gazebo_ros_plugins_msgs/srv/set_float.hpp>
#include <uuv_gazebo_ros_plugins_msgs/srv/set_use_global_current_vel.hpp>

namespace uuv_simulator_ros
{

class UnderwaterObjectROSPlugin : public uuv_gz_plugins::UnderwaterObjectPlugin
{
public:
  UnderwaterObjectROSPlugin();
  ~UnderwaterObjectROSPlugin() override;

  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &_eventMgr) override;

  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override;

private:
  void UpdateLocalCurrentVelocity(
      const geometry_msgs::msg::Vector3::SharedPtr &_msg);

  void SetUseGlobalCurrentVel(
      uuv_gazebo_ros_plugins_msgs::srv::SetUseGlobalCurrentVel::Request::
          SharedPtr _req,
      uuv_gazebo_ros_plugins_msgs::srv::SetUseGlobalCurrentVel::Response::
          SharedPtr _res);

  void GetModelProperties(
      uuv_gazebo_ros_plugins_msgs::srv::GetModelProperties::Request::SharedPtr,
      uuv_gazebo_ros_plugins_msgs::srv::GetModelProperties::Response::
          SharedPtr _res);

  void SetFluidDensity(
      uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Request::SharedPtr _req,
      uuv_gazebo_ros_plugins_msgs::srv::SetFloat::Response::SharedPtr _res);

  void GetFluidDensity(
      uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Request::SharedPtr,
      uuv_gazebo_ros_plugins_msgs::srv::GetFloat::Response::SharedPtr _res);

private:
  rclcpp::Node::SharedPtr rosNode;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr subLocalCurVel;
  std::map<std::string, rclcpp::ServiceBase::SharedPtr> services;
};

}  // namespace uuv_simulator_ros

#endif
