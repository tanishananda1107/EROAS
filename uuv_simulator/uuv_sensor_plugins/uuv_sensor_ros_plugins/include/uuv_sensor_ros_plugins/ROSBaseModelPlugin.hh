// Ported to ROS 2 / Gazebo Harmonic (gz-sim 8)
#ifndef __ROS_BASE_MODEL_PLUGIN_HH__
#define __ROS_BASE_MODEL_PLUGIN_HH__

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/math/Pose3.hh>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <uuv_sensor_ros_plugins/ROSBasePlugin.hh>
#include <memory>
#include <string>

namespace gz { namespace sim {

class ROSBaseModelPlugin
  : public ROSBasePlugin, public System,
    public ISystemConfigure, public ISystemUpdate
{
public:
  ROSBaseModelPlugin();
  virtual ~ROSBaseModelPlugin();

  void Configure(const Entity& _entity,
                 const std::shared_ptr<const sdf::Element>& _sdf,
                 EntityComponentManager& _ecm, EventManager& _eventMgr) override;
  void Update(const UpdateInfo& _info, EntityComponentManager& _ecm) override;

protected:
  virtual bool OnUpdate(const UpdateInfo& _info, EntityComponentManager& _ecm);

  Entity modelEntity{kNullEntity}, linkEntity{kNullEntity};
  Model model;
  Link link;
  bool enableLocalNEDFrame{false};
  std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
  math::Pose3d localNEDFrame;
  geometry_msgs::msg::TransformStamped tfLocalNEDFrame;
  void SendLocalNEDTransform();

private:
  EventManager* eventMgr_{nullptr};
};

}}  // namespace gz::sim
#endif  // __ROS_BASE_MODEL_PLUGIN_HH__
