// Ported to ROS 2 / Gazebo Harmonic (gz-sim 8)
#ifndef __UUV_RPT_ROS_PLUGIN_HH__
#define __UUV_RPT_ROS_PLUGIN_HH__

#include <rclcpp/rclcpp.hpp>
#include <uuv_sensor_ros_plugins/ROSBaseModelPlugin.hh>
#include <uuv_sensor_ros_plugins_msgs/msg/position_with_covariance_stamped.hpp>
#include <gz/math/Vector3.hh>

namespace gz { namespace sim {

class RPTROSPlugin : public ROSBaseModelPlugin {
public:
  RPTROSPlugin();
  virtual ~RPTROSPlugin();
  void Configure(const Entity& _entity,
                 const std::shared_ptr<const sdf::Element>& _sdf,
                 EntityComponentManager& _ecm, EventManager& _eventMgr) override;

protected:
  bool OnUpdate(const UpdateInfo& _info, EntityComponentManager& _ecm) override;
  gz::math::Vector3d position;
  uuv_sensor_ros_plugins_msgs::msg::PositionWithCovarianceStamped rosMessage;
  rclcpp::Publisher
    uuv_sensor_ros_plugins_msgs::msg::PositionWithCovarianceStamped>::SharedPtr posPub;
};

}}  // namespace gz::sim
#endif
