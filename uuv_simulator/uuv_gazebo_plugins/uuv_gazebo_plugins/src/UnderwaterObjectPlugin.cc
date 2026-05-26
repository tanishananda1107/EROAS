// Copyright (c) 2016 The UUV Simulator Authors.
// All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License")

// ============================================================
// ROS2 / Gazebo Harmonic (gz-sim 8) conversion notes:
//
//  - GZ_REGISTER_MODEL_PLUGIN → GZ_ADD_PLUGIN
//  - transport::NodePtr       → gz::transport::Node
//  - physics::World / Model / LinkPtr → gz::sim::World / Model / Entity
//  - world->Gravity()         → gz::sim::World::Gravity(_ecm)
//  - world->SimTime()         → _info.simTime (passed to PostUpdate)
//  - link->RelativeLinearAccel → gz::sim::Link::WorldLinearAcceleration
//  - msgs::WrenchStamped      → gz::msgs::Wrench (WrenchStamped no longer
//                               needed for internal debug topics; use Wrench)
//  - event::ConnectWorldUpdateBegin → ISystemPreUpdate
//  - All #if GAZEBO_MAJOR_VERSION guards removed
// ============================================================

#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/World.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Gravity.hh>
#include <gz/sim/components/Name.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/wrench.pb.h>
#include <gz/msgs/vector3d.pb.h>
#include <gz/common/Console.hh>
#include <gz/math/Vector3.hh>

#include <uuv_gazebo_plugins/UnderwaterObjectPlugin.hh>
#include <uuv_gazebo_plugins/HydrodynamicModel.hh>
#include <uuv_gazebo_plugins/Def.hh>

namespace gz {
namespace sim {

/////////////////////////////////////////////////
UnderwaterObjectPlugin::UnderwaterObjectPlugin()
  : useGlobalCurrent(true) {}

UnderwaterObjectPlugin::~UnderwaterObjectPlugin() {}

/////////////////////////////////////////////////
void UnderwaterObjectPlugin::Configure(
    const gz::sim::Entity              &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager    &_ecm,
    gz::sim::EventManager              & /*_eventMgr*/)
{
  this->model = gz::sim::Model(_entity);
  this->worldEntity = this->model.WorldEntity(_ecm);

  // Subscribe to flow velocity
  if (_sdf->HasElement("flow_velocity_topic"))
  {
    std::string flowTopic = _sdf->Get<std::string>("flow_velocity_topic");
    GZ_ASSERT(!flowTopic.empty(), "flow_velocity_topic cannot be empty");
    gzmsg << "UnderwaterObjectPlugin: subscribing to " << flowTopic << "\n";
    this->node.Subscribe(flowTopic,
        &UnderwaterObjectPlugin::OnFlowVelocity, this);
  }

  double fluidDensity = 1028.0;
  if (_sdf->HasElement("fluid_density"))
    fluidDensity = _sdf->Get<double>("fluid_density");

  if (_sdf->HasElement("use_global_current"))
    this->useGlobalCurrent = _sdf->Get<bool>("use_global_current");

  bool debugFlag = false;
  if (_sdf->HasElement("debug"))
    debugFlag = static_cast<bool>(_sdf->Get<int>("debug"));

  // Gravity from world
  double gAcc = 9.81;
  auto gravComp = _ecm.Component<gz::sim::components::Gravity>(
      this->worldEntity);
  if (gravComp)
    gAcc = std::abs(gravComp->Data().Z());

  // Iterate over <link> elements in SDF
  if (_sdf->HasElement("link"))
  {
    for (sdf::ElementPtr linkElem = _sdf->GetElement("link"); linkElem;
         linkElem = linkElem->GetNextElement("link"))
    {
      if (!linkElem->HasAttribute("name"))
      {
        gzwarn << "link element missing name attribute\n";
        continue;
      }

      std::string linkName = linkElem->Get<std::string>("name");

      // Detect base link
      if (linkName.find("base_link") != std::string::npos)
      {
        this->baseLinkName = linkName;
        gzmsg << "BASE_LINK: " << linkName << "\n";
      }

      gz::sim::Entity linkEntity = this->model.LinkByName(_ecm, linkName);
      if (linkEntity == kNullEntity)
      {
        gzwarn << "Specified link [" << linkName << "] not found.\n";
        continue;
      }

      HydrodynamicModelPtr hydro;
      hydro.reset(
          HydrodynamicModelFactory::GetInstance().CreateHydrodynamicModel(
              linkElem, linkEntity, _ecm));
      hydro->SetFluidDensity(fluidDensity);
      hydro->SetGravity(gAcc);

      if (debugFlag)
        this->InitDebug(linkEntity, linkName, hydro);

      this->models[linkEntity] = hydro;
      this->models[linkEntity]->Print("all");
    }
  }
}

/////////////////////////////////////////////////
void UnderwaterObjectPlugin::InitDebug(
    gz::sim::Entity                linkEntity,
    const std::string             &linkName,
    HydrodynamicModelPtr           hydro)
{
  std::string rootTopic = "/debug/forces/" + linkName + "/";

  for (const auto &topic :
       {"restoring", "damping", "added_mass", "added_coriolis"})
  {
    std::string fullTopic = rootTopic + topic;
    this->hydroPub[linkName + "/" + topic] =
        this->node.Advertise<gz::msgs::Wrench>(fullTopic);
  }

  hydro->SetDebugFlag(true);
  hydro->SetStoreVector(RESTORING_FORCE);
  hydro->SetStoreVector(UUV_DAMPING_FORCE);
  hydro->SetStoreVector(UUV_DAMPING_TORQUE);
  hydro->SetStoreVector(UUV_ADDED_CORIOLIS_FORCE);
  hydro->SetStoreVector(UUV_ADDED_CORIOLIS_TORQUE);
  hydro->SetStoreVector(UUV_ADDED_MASS_FORCE);
  hydro->SetStoreVector(UUV_ADDED_MASS_TORQUE);
}

/////////////////////////////////////////////////
void UnderwaterObjectPlugin::PreUpdate(
    const gz::sim::UpdateInfo          &_info,
    gz::sim::EntityComponentManager    &_ecm)
{
  if (_info.paused) return;

  double time = std::chrono::duration<double>(_info.simTime).count();

  for (auto &[linkEntity, hydro] : this->models)
  {
    gz::sim::Link link(linkEntity);

    auto linAccel =
        link.WorldLinearAcceleration(_ecm).value_or(gz::math::Vector3d::Zero);
    auto angAccel =
        link.WorldAngularAcceleration(_ecm).value_or(gz::math::Vector3d::Zero);

    GZ_ASSERT(!std::isnan(linAccel.Length()) && !std::isnan(angAccel.Length()),
              "Accelerations are NaN");

    hydro->ApplyHydrodynamicForces(time, this->flowVelocity, _ecm);
    this->PublishRestoringForce(linkEntity, _ecm);
    this->PublishHydrodynamicWrenches(linkEntity, _ecm);
  }
}

/////////////////////////////////////////////////
void UnderwaterObjectPlugin::OnFlowVelocity(const gz::msgs::Vector3d &_msg)
{
  if (this->useGlobalCurrent)
  {
    this->flowVelocity.X(_msg.x());
    this->flowVelocity.Y(_msg.y());
    this->flowVelocity.Z(_msg.z());
  }
}

/////////////////////////////////////////////////
void UnderwaterObjectPlugin::PublishRestoringForce(
    gz::sim::Entity                  _linkEntity,
    gz::sim::EntityComponentManager &_ecm)
{
  if (!this->models.count(_linkEntity)) return;
  if (!this->models[_linkEntity]->GetDebugFlag()) return;

  std::string linkName =
      _ecm.Component<gz::sim::components::Name>(_linkEntity)
          ->Data();

  gz::math::Vector3d restoring =
      this->models[_linkEntity]->GetStoredVector(RESTORING_FORCE);

  gz::msgs::Wrench msg;
  this->FillWrenchMsg(restoring, gz::math::Vector3d::Zero, msg);

  auto it = this->hydroPub.find(linkName + "/restoring");
  if (it != this->hydroPub.end())
    it->second.Publish(msg);
}

/////////////////////////////////////////////////
void UnderwaterObjectPlugin::PublishHydrodynamicWrenches(
    gz::sim::Entity                  _linkEntity,
    gz::sim::EntityComponentManager &_ecm)
{
  if (!this->models.count(_linkEntity)) return;
  if (!this->models[_linkEntity]->GetDebugFlag()) return;

  std::string linkName =
      _ecm.Component<gz::sim::components::Name>(_linkEntity)->Data();

  auto publish = [&](const std::string &suffix,
                     const std::string &forceTag,
                     const std::string &torqueTag)
  {
    gz::msgs::Wrench msg;
    this->FillWrenchMsg(
        this->models[_linkEntity]->GetStoredVector(forceTag),
        this->models[_linkEntity]->GetStoredVector(torqueTag), msg);
    auto it = this->hydroPub.find(linkName + "/" + suffix);
    if (it != this->hydroPub.end()) it->second.Publish(msg);
  };

  publish("added_mass",    UUV_ADDED_MASS_FORCE,     UUV_ADDED_MASS_TORQUE);
  publish("damping",       UUV_DAMPING_FORCE,        UUV_DAMPING_TORQUE);
  publish("added_coriolis",UUV_ADDED_CORIOLIS_FORCE, UUV_ADDED_CORIOLIS_TORQUE);
}

/////////////////////////////////////////////////
void UnderwaterObjectPlugin::FillWrenchMsg(
    const gz::math::Vector3d &_force,
    const gz::math::Vector3d &_torque,
    gz::msgs::Wrench         &_msg)
{
  _msg.mutable_force()->set_x(_force.X());
  _msg.mutable_force()->set_y(_force.Y());
  _msg.mutable_force()->set_z(_force.Z());
  _msg.mutable_torque()->set_x(_torque.X());
  _msg.mutable_torque()->set_y(_torque.Y());
  _msg.mutable_torque()->set_z(_torque.Z());
}

}  // namespace sim
}  // namespace gz

GZ_ADD_PLUGIN(gz::sim::UnderwaterObjectPlugin,
              gz::sim::System,
              gz::sim::UnderwaterObjectPlugin::ISystemConfigure,
              gz::sim::UnderwaterObjectPlugin::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(gz::sim::UnderwaterObjectPlugin,
                    "gz::sim::UnderwaterObjectPlugin")
