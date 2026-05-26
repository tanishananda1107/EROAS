// Copyright (c) 2016 The UUV Simulator Authors.
// Licensed under the Apache License, Version 2.0.

#include <uuv_gazebo_ros_plugins/AccelerationsTestPlugin.hh>
#include <uuv_gazebo_plugins/Def.hh>

#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/LinearAcceleration.hh>
#include <gz/sim/components/AngularAcceleration.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/plugin/Register.hh>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

namespace gz::sim::systems
{

/////////////////////////////////////////////////
AccelerationsTestPlugin::AccelerationsTestPlugin()
{
  last_w_v_w_b.setZero();
}

/////////////////////////////////////////////////
AccelerationsTestPlugin::~AccelerationsTestPlugin() = default;

/////////////////////////////////////////////////
void AccelerationsTestPlugin::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &_ecm,
  gz::sim::EventManager &)
{
  modelEntity = _entity;

  if (!rclcpp::ok())
    rclcpp::init(0, nullptr);

  rosNode = std::make_shared<rclcpp::Node>("accelerations_test_plugin");

  std::string linkName;
  if (_sdf->HasElement("link_name"))
    linkName = _sdf->Get<std::string>("link_name");
  else
  {
    gzerr << "[AccelerationsTestPlugin] Please specify a link_name.\n";
    return;
  }

  gz::sim::Model model(_entity);
  linkEntity = model.LinkByName(_ecm, linkName);
  if (linkEntity == gz::sim::kNullEntity)
  {
    gzerr << "[AccelerationsTestPlugin] Could not find link \"" << linkName << "\".\n";
    return;
  }

  // Enable velocity/acceleration components
  gz::sim::Link link(linkEntity);
  link.EnableVelocityChecks(_ecm, true);
  link.EnableAccelerationChecks(_ecm, true);

  pub_accel_w_gazebo = rosNode->create_publisher<geometry_msgs::msg::Vector3Stamped>(
    "accel_w_gazebo", 10);
  pub_accel_w_numeric = rosNode->create_publisher<geometry_msgs::msg::Vector3Stamped>(
    "accel_w_numeric", 10);
  pub_accel_b_gazebo = rosNode->create_publisher<geometry_msgs::msg::Vector3Stamped>(
    "accel_b_gazebo", 10);
  pub_accel_b_numeric = rosNode->create_publisher<geometry_msgs::msg::Vector3Stamped>(
    "accel_b_numeric", 10);
}

/////////////////////////////////////////////////
void AccelerationsTestPlugin::PreUpdate(
  const gz::sim::UpdateInfo &,
  gz::sim::EntityComponentManager &) {}

/////////////////////////////////////////////////
static geometry_msgs::msg::Vector3Stamped vec3Msg(
  const gz::math::Vector3d &v, const rclcpp::Time &stamp, const std::string &frame)
{
  geometry_msgs::msg::Vector3Stamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame;
  msg.vector.x = v.X();
  msg.vector.y = v.Y();
  msg.vector.z = v.Z();
  return msg;
}

/////////////////////////////////////////////////
void AccelerationsTestPlugin::Update(
  const gz::sim::UpdateInfo &_info,
  gz::sim::EntityComponentManager &_ecm)
{
  if (linkEntity == gz::sim::kNullEntity)
    return;

  double dt = std::chrono::duration<double>(_info.simTime - lastTime).count();
  if (dt <= 0.0) return;

  gz::sim::Link link(linkEntity);

  auto w_lin_vel = link.WorldLinearVelocity(_ecm).value_or(gz::math::Vector3d::Zero);
  auto w_ang_vel = link.WorldAngularVelocity(_ecm).value_or(gz::math::Vector3d::Zero);
  auto b_lin_vel = link.RelativeLinearVelocity(_ecm).value_or(gz::math::Vector3d::Zero);
  auto b_ang_vel = link.RelativeAngularVelocity(_ecm).value_or(gz::math::Vector3d::Zero);

  auto w_lin_acc = link.WorldLinearAcceleration(_ecm).value_or(gz::math::Vector3d::Zero);
  auto w_ang_acc = link.WorldAngularAcceleration(_ecm).value_or(gz::math::Vector3d::Zero);
  auto b_lin_acc = link.RelativeLinearAcceleration(_ecm).value_or(gz::math::Vector3d::Zero);
  auto b_ang_acc = link.RelativeAngularAcceleration(_ecm).value_or(gz::math::Vector3d::Zero);

  // World-frame velocity as 6-vector
  Eigen::Matrix<double,6,1> w_v;
  w_v << w_lin_vel.X(), w_lin_vel.Y(), w_lin_vel.Z(),
         w_ang_vel.X(), w_ang_vel.Y(), w_ang_vel.Z();

  // Numeric differentiation in world frame
  Eigen::Matrix<double,6,1> num_w_a = (w_v - last_w_v_w_b) / dt;

  // Rotation to body frame
  auto pose = link.WorldPose(_ecm).value_or(gz::math::Pose3d::Zero);
  gz::math::Matrix3d R_b_w_gz(pose.Rot().Inverse());
  Eigen::Matrix3d R;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      R(i, j) = R_b_w_gz(i, j);

  Eigen::Matrix<double,6,1> num_b_a;
  num_b_a.head<3>() = R * num_w_a.head<3>();
  num_b_a.tail<3>() = R * num_w_a.tail<3>();

  rclcpp::Time stamp(_info.simTime.count());
  std::string world_frame = "world";
  std::string body_frame = _ecm.ComponentData<gz::sim::components::Name>(linkEntity)
    .value_or("link");

  pub_accel_w_gazebo->publish(vec3Msg(w_lin_acc, stamp, world_frame));
  pub_accel_b_gazebo->publish(vec3Msg(b_lin_acc, stamp, body_frame));
  pub_accel_w_numeric->publish(vec3Msg(
    gz::math::Vector3d(num_w_a[0], num_w_a[1], num_w_a[2]), stamp, world_frame));
  pub_accel_b_numeric->publish(vec3Msg(
    gz::math::Vector3d(num_b_a[0], num_b_a[1], num_b_a[2]), stamp, body_frame));

  last_w_v_w_b = w_v;
  lastTime = _info.simTime;
}

} // namespace gz::sim::systems

GZ_ADD_PLUGIN(gz::sim::systems::AccelerationsTestPlugin,
              gz::sim::System,
              gz::sim::systems::AccelerationsTestPlugin::ISystemConfigure,
              gz::sim::systems::AccelerationsTestPlugin::ISystemPreUpdate,
              gz::sim::systems::AccelerationsTestPlugin::ISystemUpdate)
