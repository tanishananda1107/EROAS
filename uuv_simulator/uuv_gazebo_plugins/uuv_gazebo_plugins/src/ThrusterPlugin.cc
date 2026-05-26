// Copyright (c) 2016 The UUV Simulator Authors.
// All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License")

// ============================================================
// ROS2 / Gazebo Harmonic (gz-sim 8) conversion notes:
//
//  - GZ_REGISTER_MODEL_PLUGIN   → GZ_ADD_PLUGIN
//  - transport::NodePtr         → gz::transport::Node (stack-allocated)
//  - node->Subscribe/Advertise  → node.Subscribe/Advertise
//  - ConstDoublePtr callback    → gz::msgs::Double callback
//  - msgs::Vector3d / msgs::Set → gz::msgs::Vector3d directly
//  - physics::ModelPtr / LinkPtr/ JointPtr → gz::sim::Entity + wrappers
//  - link->AddRelativeForce     → gz::sim::Link::AddWorldForce (rotated)
//  - joint->SetVelocity         → gz::sim::components::JointVelocityCmd
//  - joint->WorldPose / GlobalAxis → gz::sim::Joint wrappers
//  - event callback             → ISystemPreUpdate
//  - boost headers removed; std equivalents used
// ============================================================

#include <limits>
#include <cmath>
#include <string>

#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/JointAxis.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/vector3d.pb.h>
#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

#include <uuv_gazebo_plugins/ThrusterPlugin.hh>
#include <uuv_gazebo_plugins/Def.hh>

namespace gz {
namespace sim {

/////////////////////////////////////////////////
ThrusterPlugin::ThrusterPlugin()
  : thrustForce(0),
    inputCommand(0),
    clampMin(std::numeric_limits<double>::lowest()),
    clampMax(std::numeric_limits<double>::max()),
    thrustMin(std::numeric_limits<double>::lowest()),
    thrustMax(std::numeric_limits<double>::max()),
    gain(1.0),
    isOn(true),
    thrustEfficiency(1.0),
    propellerEfficiency(1.0),
    thrusterID(-1)
{}

ThrusterPlugin::~ThrusterPlugin() {}

/////////////////////////////////////////////////
void ThrusterPlugin::Configure(
    const gz::sim::Entity              &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager    &_ecm,
    gz::sim::EventManager              & /*_eventMgr*/)
{
  this->model = gz::sim::Model(_entity);

  // Link name
  GZ_ASSERT(_sdf->HasElement("linkName"), "Could not find linkName.");
  this->thrusterLinkEntity = this->model.LinkByName(
      _ecm, _sdf->Get<std::string>("linkName"));
  GZ_ASSERT(this->thrusterLinkEntity != kNullEntity, "thruster link is invalid");

  // Thruster ID
  GZ_ASSERT(_sdf->HasElement("thrusterID"), "Thruster ID not provided");
  this->thrusterID = _sdf->Get<int>("thrusterID");

  // Dynamics
  GZ_ASSERT(_sdf->HasElement("dynamics"), "Could not find dynamics.");
  this->thrusterDynamics.reset(
      DynamicsFactory::GetInstance().CreateDynamics(
          const_cast<sdf::ElementPtr>(_sdf->GetElement("dynamics"))));

  // Conversion function
  GZ_ASSERT(_sdf->HasElement("conversion"), "Could not find conversion.");
  this->conversionFunction.reset(
      ConversionFunctionFactory::GetInstance().CreateConversionFunction(
          const_cast<sdf::ElementPtr>(_sdf->GetElement("conversion"))));

  // Optional joint (visualisation)
  if (_sdf->HasElement("jointName"))
  {
    this->jointEntity = this->model.JointByName(
        _ecm, _sdf->Get<std::string>("jointName"));
  }

  // Clamping intervals
  if (_sdf->HasElement("clampMin")) this->clampMin = _sdf->Get<double>("clampMin");
  if (_sdf->HasElement("clampMax")) this->clampMax = _sdf->Get<double>("clampMax");
  if (this->clampMin >= this->clampMax)
  {
    gzmsg << "clampMax must be > clampMin; reverting to defaults\n";
    this->clampMin = std::numeric_limits<double>::lowest();
    this->clampMax = std::numeric_limits<double>::max();
  }

  if (_sdf->HasElement("thrustMin")) this->thrustMin = _sdf->Get<double>("thrustMin");
  if (_sdf->HasElement("thrustMax")) this->thrustMax = _sdf->Get<double>("thrustMax");
  if (this->thrustMin >= this->thrustMax)
  {
    gzmsg << "thrustMax must be > thrustMin; reverting to defaults\n";
    this->thrustMin = std::numeric_limits<double>::lowest();
    this->thrustMax = std::numeric_limits<double>::max();
  }

  if (_sdf->HasElement("gain")) this->gain = _sdf->Get<double>("gain");

  if (_sdf->HasElement("thrust_efficiency"))
  {
    this->thrustEfficiency = _sdf->Get<double>("thrust_efficiency");
    if (this->thrustEfficiency < 0.0 || this->thrustEfficiency > 1.0)
    {
      gzmsg << "Invalid thrust_efficiency; setting to 100%\n";
      this->thrustEfficiency = 1.0;
    }
  }

  if (_sdf->HasElement("propeller_efficiency"))
  {
    this->propellerEfficiency = _sdf->Get<double>("propeller_efficiency");
    if (this->propellerEfficiency < 0.0 || this->propellerEfficiency > 1.0)
    {
      gzmsg << "Invalid propeller_efficiency; setting to 100%\n";
      this->propellerEfficiency = 1.0;
    }
  }

  // Topic prefix
  std::string modelName = this->model.Name(_ecm);
  this->topicPrefix = "/" + modelName + "/thrusters/" +
                      std::to_string(this->thrusterID) + "/";

  // Publish thrust
  this->thrustPub = this->node.Advertise<gz::msgs::Vector3d>(
      this->topicPrefix + "thrust");

  // Subscribe to input
  this->node.Subscribe(this->topicPrefix + "input",
      &ThrusterPlugin::OnInput, this);

  // Compute thruster axis from joint global axis in body frame
  if (this->jointEntity != kNullEntity)
  {
    gz::sim::Joint joint(this->jointEntity);
    // axis in world frame at configure time (orientation may not be set yet;
    // recomputed in first PreUpdate)
    this->thrusterAxis = gz::math::Vector3d::UnitX;  // default; updated below
    auto axisComp = _ecm.Component<gz::sim::components::JointAxis>(
        this->jointEntity);
    if (axisComp)
      this->thrusterAxis = axisComp->Data().Xyz();
  }
  else
  {
    this->thrusterAxis = gz::math::Vector3d::UnitX;
  }
}

/////////////////////////////////////////////////
void ThrusterPlugin::PreUpdate(
    const gz::sim::UpdateInfo          &_info,
    gz::sim::EntityComponentManager    &_ecm)
{
  if (_info.paused) return;

  GZ_ASSERT(!std::isnan(this->inputCommand), "NaN in inputCommand");

  double dynamicsInput = this->isOn ?
      std::min(std::max(this->gain * this->inputCommand,
                        this->clampMin), this->clampMax) : 0.0;

  double simTime = std::chrono::duration<double>(_info.simTime).count();

  double dynamicState = this->propellerEfficiency *
      this->thrusterDynamics->update(dynamicsInput, simTime);

  GZ_ASSERT(!std::isnan(dynamicState), "Invalid dynamic state");

  this->thrustForce = this->thrustEfficiency *
      this->conversionFunction->convert(dynamicState);

  GZ_ASSERT(!std::isnan(this->thrustForce), "Invalid thrust force");

  this->thrustForce = std::max(this->thrustForce, this->thrustMin);
  this->thrustForce = std::min(this->thrustForce, this->thrustMax);

  // Compute world-frame force: rotate body-frame axis to world
  gz::sim::Link link(this->thrusterLinkEntity);
  gz::math::Pose3d pose =
      link.WorldPose(_ecm).value_or(gz::math::Pose3d());
  gz::math::Vector3d forceWorld =
      pose.Rot().RotateVector(this->thrusterAxis * this->thrustForce);

  link.AddWorldForce(_ecm, forceWorld);

  // Spin joint
  if (this->jointEntity != kNullEntity)
  {
    auto *velCmd = _ecm.Component<gz::sim::components::JointVelocityCmd>(
        this->jointEntity);
    if (velCmd)
      velCmd->Data()[0] = dynamicState;
    else
      _ecm.CreateComponent(this->jointEntity,
          gz::sim::components::JointVelocityCmd({dynamicState}));
  }

  // Publish thrust vector
  gz::msgs::Vector3d thrustMsg;
  thrustMsg.set_x(this->thrustForce);
  thrustMsg.set_y(0.0);
  thrustMsg.set_z(0.0);
  this->thrustPub.Publish(thrustMsg);
}

/////////////////////////////////////////////////
void ThrusterPlugin::OnInput(const gz::msgs::Double &_msg)
{
  this->inputCommand = _msg.data();
}

void ThrusterPlugin::Reset()
{
  this->thrusterDynamics->Reset();
}

}  // namespace sim
}  // namespace gz

GZ_ADD_PLUGIN(gz::sim::ThrusterPlugin,
              gz::sim::System,
              gz::sim::ThrusterPlugin::ISystemConfigure,
              gz::sim::ThrusterPlugin::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(gz::sim::ThrusterPlugin, "gz::sim::ThrusterPlugin")
