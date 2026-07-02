#ifndef UUV_GAZEBO_ROS_PLUGINS_FIN_ROS_PLUGIN_HH_
#define UUV_GAZEBO_ROS_PLUGINS_FIN_ROS_PLUGIN_HH_

#include <chrono>
#include <map>
#include <memory>
#include <string>

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <uuv_gazebo_plugins/FinPlugin.hh>
#include <uuv_gazebo_ros_plugins_msgs/msg/float_stamped.hpp>
#include <uuv_gazebo_ros_plugins_msgs/srv/get_list_param.hpp>

namespace uuv_simulator_ros
{

class FinROSPlugin : public uuv_gz_plugins::FinPlugin
{
public:
  FinROSPlugin();
  ~FinROSPlugin() override;

  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &_eventMgr) override;

  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override;

  void RosPublishStates();

  void SetReference(
      const uuv_gazebo_ros_plugins_msgs::msg::FloatStamped::SharedPtr &_msg);

  bool GetLiftDragParams(
      uuv_gazebo_ros_plugins_msgs::srv::GetListParam::Request::SharedPtr _req,
      uuv_gazebo_ros_plugins_msgs::srv::GetListParam::Response::SharedPtr _res);

  std::chrono::nanoseconds GetRosPublishPeriod() const;
  void SetRosPublishRate(double _hz);

private:
  rclcpp::Node::SharedPtr rosNode;
  rclcpp::Subscription<uuv_gazebo_ros_plugins_msgs::msg::FloatStamped>::SharedPtr
      subReference;
  rclcpp::Publisher<uuv_gazebo_ros_plugins_msgs::msg::FloatStamped>::SharedPtr
      pubState;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pubFinForce;
  std::map<std::string,
           rclcpp::Service<uuv_gazebo_ros_plugins_msgs::srv::GetListParam>::
               SharedPtr> services;

  std::chrono::nanoseconds rosPublishPeriod;
  std::chrono::steady_clock::time_point lastRosPublishTime;
};

}  // namespace uuv_simulator_ros

#endif
