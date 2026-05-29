// Ported to ROS 2 / Gazebo Harmonic (gz-sim 8)
#ifndef __UUV_POSE_GT_SENSOR_ROS_PLUGIN_HH__
#define __UUV_POSE_GT_SENSOR_ROS_PLUGIN_HH__

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <uuv_sensor_ros_plugins/ROSBaseModelPlugin.hh>
#include <memory>
#include <string>

namespace gz { namespace sim {

class PoseGTROSPlugin : public ROSBaseModelPlugin {
public:
  PoseGTROSPlugin();
  virtual ~PoseGTROSPlugin();
  void Configure(const Entity& _entity,
                 const std::shared_ptr<const sdf::Element>& _sdf,
                 EntityComponentManager& _ecm, EventManager& _eventMgr) override;

protected:
  bool OnUpdate(const UpdateInfo& _info, EntityComponentManager& _ecm) override;
  void PublishNEDOdomMessage(const rclcpp::Time& _time, const gz::math::Pose3d& _pose,
    const gz::math::Vector3d& _linVel, const gz::math::Vector3d& _angVel);
  void PublishOdomMessage(const rclcpp::Time& _time, const gz::math::Pose3d& _pose,
    const gz::math::Vector3d& _linVel, const gz::math::Vector3d& _angVel);
  void UpdateNEDTransform(EntityComponentManager& _ecm);

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr nedOdomPub, odomPub;
  gz::math::Pose3d offset, nedTransform;
  std::string nedFrameID;
  bool nedTransformIsInit{false}, publishNEDOdom{false};
  std::shared_ptr<tf2_ros::Buffer> tfBuffer;
  std::shared_ptr<tf2_ros::TransformListener> tfListener;
  gz::math::Vector3d lastLinVel, lastAngVel, linAcc, angAcc,
                     lastRefLinVel, lastRefAngVel, refLinAcc, refAngAcc;
};

}}  // namespace gz::sim
#endif
