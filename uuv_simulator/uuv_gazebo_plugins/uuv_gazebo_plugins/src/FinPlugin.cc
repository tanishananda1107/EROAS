// Copyright (c) 2016 The UUV Simulator Authors.
// All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License")

// ============================================================
// ROS2 / Gazebo Harmonic (gz-sim 8) conversion notes:
//
//  ARCHITECTURE CHANGE:
//  Classic Gazebo used ModelPlugin + transport::Node + event callbacks.
//  gz-sim 8 uses the ISystem interface: ISystemConfigure + ISystemPreUpdate.
//
//  Key changes:
//  - GZ_REGISTER_MODEL_PLUGIN  → GZ_ADD_PLUGIN
//  - transport::NodePtr        → gz::transport::Node
//  - node->Subscribe / Advertise → node.Subscribe / node.Advertise
//  - Gazebo msgs (ConstDoublePtr / ConstVector3dPtr) → gz::msgs::Double / Vector3d
//  - event::Events::ConnectWorldUpdateBegin → ISystemPreUpdate callback
//  - physics::ModelPtr / LinkPtr / JointPtr → gz::sim::Entity + ECM wrappers
//  - link->AddRelativeForce    → gz::sim::Link::AddWorldForce (rotated)
//  - joint->SetPosition / SetVelocity → gz::sim::Joint ECM helpers
//  - #if GAZEBO_MAJOR_VERSION guards removed entirely
// ============================================================

#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/vector3d.pb.h>
#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

#include <uuv_gazebo_plugins/FinPlugin.hh>
#include <uuv_gazebo_plugins/Def.hh>

namespace uuv_gz_plugins
{

/////////////////////////////////////////////////
FinPlugin::FinPlugin()
  : inputCommand(0.0), angle(0.0), finID(-1) {}

FinPlugin::~FinPlugin() {}

/////////////////////////////////////////////////
void FinPlugin::Configure(
    const gz::sim::Entity              &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager    &_ecm,
    gz::sim::EventManager              & /*_eventMgr*/)
{
  this->model = gz::sim::Model(_entity);
  auto sdf = std::const_pointer_cast<sdf::Element>(_sdf);

  // Fin ID
  GZ_ASSERT(_sdf->HasElement("fin_id"), "Could not find fin_id parameter.");
  this->finID = _sdf->Get<int>("fin_id");
  GZ_ASSERT(this->finID >= 0, "Fin ID must be >= 0");

  // Topic prefix
  std::string modelName = this->model.Name(_ecm);
  this->topicPrefix = "/" + modelName + "/fins/" +
                      std::to_string(this->finID) + "/";

  std::string inputTopic  = _sdf->HasElement("input_topic")  ?
      _sdf->Get<std::string>("input_topic")  : this->topicPrefix + "input";
  std::string outputTopic = _sdf->HasElement("output_topic") ?
      _sdf->Get<std::string>("output_topic") : this->topicPrefix + "output";
  this->commandTopic = inputTopic;
  this->angleTopic = outputTopic;

  // Link
  GZ_ASSERT(_sdf->HasElement("link_name"), "Could not find link_name.");
  this->linkName = _sdf->Get<std::string>("link_name");
  this->linkEntity = this->model.LinkByName(_ecm, this->linkName);
  GZ_ASSERT(this->linkEntity != gz::sim::kNullEntity, "link is invalid");

  // Joint
  GZ_ASSERT(_sdf->HasElement("joint_name"), "Could not find joint_name.");
  this->jointEntity = this->model.JointByName(_ecm,
      _sdf->Get<std::string>("joint_name"));
  GZ_ASSERT(this->jointEntity != gz::sim::kNullEntity, "joint is invalid");

  // Dynamics model
  GZ_ASSERT(_sdf->HasElement("dynamics"), "Could not find dynamics.");
  this->dynamics.reset(DynamicsFactory::GetInstance().CreateDynamics(
      sdf->GetElement("dynamics")));

  // Lift/drag model
  GZ_ASSERT(_sdf->HasElement("liftdrag"), "Could not find liftdrag.");
  this->liftdrag.reset(LiftDragFactory::GetInstance().CreateLiftDrag(
      sdf->GetElement("liftdrag")));

  // Subscribe to current velocity
  GZ_ASSERT(_sdf->HasElement("current_velocity_topic"),
            "Could not find current_velocity_topic.");
  std::string currentTopic = _sdf->Get<std::string>("current_velocity_topic");
  GZ_ASSERT(!currentTopic.empty(), "current_velocity_topic cannot be empty");

  gzmsg << "FinPlugin: subscribing to " << currentTopic << "\n";
  this->node.Subscribe(currentTopic,
      &FinPlugin::OnCurrentVelocity, this);

  // Subscribe to input command
  this->node.Subscribe(inputTopic, &FinPlugin::OnInput, this);

  // Advertise output (angle)
  this->anglePub = this->node.Advertise<gz::msgs::Double>(outputTopic);
}

/////////////////////////////////////////////////
void FinPlugin::PreUpdate(
    const gz::sim::UpdateInfo          &_info,
    gz::sim::EntityComponentManager    &_ecm)
{
  if (_info.paused) return;

  GZ_ASSERT(!std::isnan(this->inputCommand), "NaN in inputCommand");

  gz::sim::Joint joint(this->jointEntity);

  // Get joint limits
  double upperLimit =  1e6;
  double lowerLimit = -1e6;
  auto axes = joint.Axis(_ecm);
  if (axes && !axes->empty())
  {
    lowerLimit = axes->front().Lower();
    upperLimit = axes->front().Upper();
  }

  this->inputCommand = std::min(upperLimit, this->inputCommand);
  this->inputCommand = std::max(lowerLimit, this->inputCommand);

  double simTime = std::chrono::duration<double>(_info.simTime).count();
  this->angle = this->dynamics->update(this->inputCommand, simTime);

  gz::sim::Link link(this->linkEntity);
  gz::math::Pose3d finPose =
      link.WorldPose(_ecm).value_or(gz::math::Pose3d());
  gz::math::Vector3d linVel =
      link.WorldLinearVelocity(_ecm).value_or(gz::math::Vector3d::Zero);

  gz::math::Vector3d ldNormalI =
      finPose.Rot().RotateVector(gz::math::Vector3d::UnitZ);

  gz::math::Vector3d velI = linVel - this->currentVelocity;
  gz::math::Vector3d velInLDPlaneI =
      ldNormalI.Cross(velI.Cross(ldNormalI));
  gz::math::Vector3d velInLDPlaneL =
      finPose.Rot().RotateVectorReverse(velInLDPlaneI);

  this->finForce = this->liftdrag->compute(velInLDPlaneL);

  // Rotate body-frame force to world frame and apply
  gz::math::Vector3d finForceWorld =
      finPose.Rot().RotateVector(this->finForce);
  link.AddWorldForce(_ecm, finForceWorld);

  // Set joint position
  auto *posComp = _ecm.Component<gz::sim::components::JointPosition>(
      this->jointEntity);
  if (posComp)
    posComp->Data()[0] = this->angle;
  else
    _ecm.CreateComponent(this->jointEntity,
        gz::sim::components::JointPosition({this->angle}));

  // Publish angle
  gz::msgs::Double msg;
  msg.set_data(this->angle);
  this->anglePub.Publish(msg);
}

/////////////////////////////////////////////////
void FinPlugin::OnInput(const gz::msgs::Double &_msg)
{
  this->inputCommand = _msg.data();
}

void FinPlugin::OnCurrentVelocity(const gz::msgs::Vector3d &_msg)
{
  this->currentVelocity.X(_msg.x());
  this->currentVelocity.Y(_msg.y());
  this->currentVelocity.Z(_msg.z());
}

}  // namespace uuv_gz_plugins

// Register the plugin with gz-sim 8
GZ_ADD_PLUGIN(uuv_gz_plugins::FinPlugin,
              gz::sim::System,
              uuv_gz_plugins::FinPlugin::ISystemConfigure,
              uuv_gz_plugins::FinPlugin::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(uuv_gz_plugins::FinPlugin, "uuv_gz_plugins::FinPlugin")
