// Ported to ROS 2 / Gazebo Harmonic (gz-sim 8)
#ifndef __UUV_DVL_ROS_PLUGIN_HH__
#define __UUV_DVL_ROS_PLUGIN_HH__

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <uuv_sensor_ros_plugins/ROSBaseModelPlugin.hh>
#include <uuv_sensor_ros_plugins_msgs/msg/dvl.hpp>
#include <uuv_sensor_ros_plugins_msgs/msg/dvl_beam.hpp>
#include <gz/math/Pose3.hh>
#include <memory>
#include <string>
#include <vector>

#define ALTITUDE_OUT_OF_RANGE -1.0

namespace gz { namespace sim {

class DVLROSPlugin : public ROSBaseModelPlugin {
public:
  DVLROSPlugin();
  virtual ~DVLROSPlugin();
  void Configure(const Entity& _entity,
                 const std::shared_ptr<const sdf::Element>& _sdf,
                 EntityComponentManager& _ecm, EventManager& _eventMgr) override;

protected:
  bool OnUpdate(const UpdateInfo& _info, EntityComponentManager& _ecm) override;
  void OnBeamCallback(
    const sensor_msgs::msg::Range::ConstSharedPtr& _range0,
    const sensor_msgs::msg::Range::ConstSharedPtr& _range1,
    const sensor_msgs::msg::Range::ConstSharedPtr& _range2,
    const sensor_msgs::msg::Range::ConstSharedPtr& _range3);
  bool UpdateBeamTransforms();

  bool beamTransformsInitialized{false};
  double altitude{ALTITUDE_OUT_OF_RANGE};
  uuv_sensor_ros_plugins_msgs::msg::DVL dvlROSMsg;
  std::vector<uuv_sensor_ros_plugins_msgs::msg::DVLBeam> dvlBeamMsgs;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twistPub;
  geometry_msgs::msg::TwistWithCovarianceStamped twistROSMsg;
  std::vector<std::string> beamsLinkNames, beamTopics;
  std::vector<gz::math::Pose3d> beamPoses;

  using RangeSub = message_filters::Subscriber<sensor_msgs::msg::Range>;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime
    sensor_msgs::msg::Range, sensor_msgs::msg::Range,
    sensor_msgs::msg::Range, sensor_msgs::msg::Range>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

  std::shared_ptr<RangeSub> beamSub0, beamSub1, beamSub2, beamSub3;
  std::shared_ptr<Synchronizer> syncBeamMessages;
  std::shared_ptr<tf2_ros::Buffer> tfBuffer;
  std::shared_ptr<tf2_ros::TransformListener> tfListener;
};

}}  // namespace gz::sim
#endif
