// Copyright (c) 2016 The UUV Simulator Authors.
// Licensed under the Apache License, Version 2.0.

#include <uuv_gazebo_ros_plugins/JointStatePublisher.hh>

#include <gz/sim/Model.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointForce.hh>
#include <gz/sim/components/JointAxis.hh>
#include <gz/sim/components/JointType.hh>
#include <gz/sim/components/Name.hh>
#include <gz/plugin/Register.hh>

#include <sdf/Joint.hh>

namespace uuv_simulator_ros
{

/////////////////////////////////////////////////
JointStatePublisher::JointStatePublisher() = default;
JointStatePublisher::~JointStatePublisher() = default;

/////////////////////////////////////////////////
void JointStatePublisher::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &_ecm,
  gz::sim::EventManager &)
{
  modelEntity = _entity;
  model = gz::sim::Model(_entity);

  if (!rclcpp::ok())
    rclcpp::init(0, nullptr);

  if (_sdf->HasElement("robotNamespace"))
    robotNamespace = _sdf->Get<std::string>("robotNamespace");
  else
    robotNamespace = _ecm.ComponentData<gz::sim::components::Name>(_entity)
      .value_or("robot");

  if (robotNamespace[0] != '/')
    robotNamespace = "/" + robotNamespace;

  node = std::make_shared<rclcpp::Node>("joint_state_publisher",
    robotNamespace.substr(1)); // strip leading slash for node namespace

  gzmsg << "JointStatePublisher::robotNamespace=" << robotNamespace << std::endl;

  updateRate = _sdf->HasElement("updateRate") ?
    _sdf->Get<double>("updateRate") : 50.0;

  GZ_ASSERT(updateRate > 0, "Update rate must be positive");
  updatePeriod = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(1.0 / updateRate));

  // Collect moving joints
  movingJoints.clear();
  model.ForEachJoint(_ecm, [&](const gz::sim::Entity &jointEntity) -> bool
  {
    auto jointType = _ecm.Component<gz::sim::components::JointType>(jointEntity);
    if (jointType && jointType->Data() == sdf::JointType::FIXED)
      return true;

    auto axis = _ecm.Component<gz::sim::components::JointAxis>(jointEntity);
    if (axis)
    {
      double lower = axis->Data().Lower();
      double upper = axis->Data().Upper();
      if (lower == 0.0 && upper == 0.0)
        return true;
    }

    auto name = _ecm.ComponentData<gz::sim::components::Name>(jointEntity);
    if (name)
    {
      movingJoints.push_back(*name);
      gzmsg << "\t- " << *name << std::endl;
      // Enable position/velocity/effort components
      _ecm.CreateComponent(jointEntity, gz::sim::components::JointPosition());
      _ecm.CreateComponent(jointEntity, gz::sim::components::JointVelocity());
      _ecm.CreateComponent(jointEntity, gz::sim::components::JointForce());
    }
    return true;
  });

  jointStatePub = node->create_publisher<sensor_msgs::msg::JointState>(
    robotNamespace + "/joint_states", 1);

  lastUpdate = std::chrono::steady_clock::duration::zero();
}

/////////////////////////////////////////////////
void JointStatePublisher::PostUpdate(
  const gz::sim::UpdateInfo &_info,
  const gz::sim::EntityComponentManager &_ecm)
{
  if (_info.simTime - lastUpdate >= updatePeriod)
  {
    PublishJointStates(_info, _ecm);
    lastUpdate = _info.simTime;
  }
}

/////////////////////////////////////////////////
void JointStatePublisher::PublishJointStates(
  const gz::sim::UpdateInfo &_info,
  const gz::sim::EntityComponentManager &_ecm)
{
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = rclcpp::Time(_info.simTime.count());

  model.ForEachJoint(_ecm, [&](const gz::sim::Entity &jointEntity) -> bool
  {
    auto nameComp = _ecm.ComponentData<gz::sim::components::Name>(jointEntity);
    if (!nameComp) return true;
    const std::string &jName = *nameComp;

    double pos = 0.0, vel = 0.0, eff = 0.0;
    if (!IsIgnoredJoint(jName))
    {
      auto posComp = _ecm.Component<gz::sim::components::JointPosition>(jointEntity);
      auto velComp = _ecm.Component<gz::sim::components::JointVelocity>(jointEntity);
      auto effComp = _ecm.Component<gz::sim::components::JointForce>(jointEntity);

      if (posComp && !posComp->Data().empty()) pos = posComp->Data()[0];
      if (velComp && !velComp->Data().empty()) vel = velComp->Data()[0];
      if (effComp && !effComp->Data().empty()) eff = effComp->Data()[0];
    }

    msg.name.push_back(jName);
    msg.position.push_back(pos);
    msg.velocity.push_back(vel);
    msg.effort.push_back(eff);
    return true;
  });

  jointStatePub->publish(msg);
}

/////////////////////////////////////////////////
bool JointStatePublisher::IsIgnoredJoint(const std::string &_jointName) const
{
  if (movingJoints.empty()) return true;
  for (const auto &j : movingJoints)
    if (_jointName == j) return false;
  return true;
}

} // namespace uuv_simulator_ros

GZ_ADD_PLUGIN(uuv_simulator_ros::JointStatePublisher,
              gz::sim::System,
              uuv_simulator_ros::JointStatePublisher::ISystemConfigure,
              uuv_simulator_ros::JointStatePublisher::ISystemPostUpdate)
