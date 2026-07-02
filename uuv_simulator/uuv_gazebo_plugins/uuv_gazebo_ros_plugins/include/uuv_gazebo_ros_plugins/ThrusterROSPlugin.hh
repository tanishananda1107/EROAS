#ifndef UUV_GAZEBO_ROS_PLUGINS_THRUSTER_ROS_PLUGIN_HH_
#define UUV_GAZEBO_ROS_PLUGINS_THRUSTER_ROS_PLUGIN_HH_

#include <chrono>
#include <map>
#include <memory>
#include <string>

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <uuv_gazebo_plugins/ThrusterPlugin.hh>
#include <uuv_gazebo_ros_plugins_msgs/msg/float_stamped.hpp>
#include <uuv_gazebo_ros_plugins_msgs/srv/get_thruster_conversion_fcn.hpp>
#include <uuv_gazebo_ros_plugins_msgs/srv/get_thruster_efficiency.hpp>
#include <uuv_gazebo_ros_plugins_msgs/srv/get_thruster_state.hpp>
#include <uuv_gazebo_ros_plugins_msgs/srv/set_thruster_efficiency.hpp>
#include <uuv_gazebo_ros_plugins_msgs/srv/set_thruster_state.hpp>

namespace uuv_simulator_ros
{

class ThrusterROSPlugin : public uuv_gz_plugins::ThrusterPlugin
{
public:
  ThrusterROSPlugin();
  ~ThrusterROSPlugin() override;

  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &_eventMgr) override;

  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override;

  void RosPublishStates();

  void SetThrustReference(
      const uuv_gazebo_ros_plugins_msgs::msg::FloatStamped::SharedPtr &_msg);

  std::chrono::nanoseconds GetRosPublishPeriod() const;
  void SetRosPublishRate(double _hz);

  bool SetThrustForceEfficiency(
      uuv_gazebo_ros_plugins_msgs::srv::SetThrusterEfficiency::Request::
          SharedPtr _req,
      uuv_gazebo_ros_plugins_msgs::srv::SetThrusterEfficiency::Response::
          SharedPtr _res);

  bool GetThrustForceEfficiency(
      uuv_gazebo_ros_plugins_msgs::srv::GetThrusterEfficiency::Request::
          SharedPtr _req,
      uuv_gazebo_ros_plugins_msgs::srv::GetThrusterEfficiency::Response::
          SharedPtr _res);

  bool SetDynamicStateEfficiency(
      uuv_gazebo_ros_plugins_msgs::srv::SetThrusterEfficiency::Request::
          SharedPtr _req,
      uuv_gazebo_ros_plugins_msgs::srv::SetThrusterEfficiency::Response::
          SharedPtr _res);

  bool GetDynamicStateEfficiency(
      uuv_gazebo_ros_plugins_msgs::srv::GetThrusterEfficiency::Request::
          SharedPtr _req,
      uuv_gazebo_ros_plugins_msgs::srv::GetThrusterEfficiency::Response::
          SharedPtr _res);

  bool SetThrusterState(
      uuv_gazebo_ros_plugins_msgs::srv::SetThrusterState::Request::SharedPtr
          _req,
      uuv_gazebo_ros_plugins_msgs::srv::SetThrusterState::Response::SharedPtr
          _res);

  bool GetThrusterState(
      uuv_gazebo_ros_plugins_msgs::srv::GetThrusterState::Request::SharedPtr
          _req,
      uuv_gazebo_ros_plugins_msgs::srv::GetThrusterState::Response::SharedPtr
          _res);

  bool GetThrusterConversionFcn(
      uuv_gazebo_ros_plugins_msgs::srv::GetThrusterConversionFcn::Request::
          SharedPtr _req,
      uuv_gazebo_ros_plugins_msgs::srv::GetThrusterConversionFcn::Response::
          SharedPtr _res);

private:
  std::map<std::string, rclcpp::ServiceBase::SharedPtr> services;
  rclcpp::Node::SharedPtr rosNode;

  rclcpp::Subscription<uuv_gazebo_ros_plugins_msgs::msg::FloatStamped>::
      SharedPtr subThrustReference;
  rclcpp::Publisher<uuv_gazebo_ros_plugins_msgs::msg::FloatStamped>::SharedPtr
      pubThrust;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr
      pubThrustWrench;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pubThrusterState;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pubThrustForceEff;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pubDynamicStateEff;

  std::chrono::nanoseconds rosPublishPeriod;
  std::chrono::steady_clock::time_point lastRosPublishTime;
};

}  // namespace uuv_simulator_ros

#endif
