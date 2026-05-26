// Copyright (c) 2016 The UUV Simulator Authors.
// All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// GZ-SIM 8 (Gazebo Harmonic) port
// Changes from ROS1/Gazebo Classic:
//   - Inherits from gz::sim::System + ISystemConfigure + ISystemPreUpdate
//     instead of gazebo::ModelPlugin
//   - Load()  → Configure()  (receives Entity, sdf::Element, ECM, EventManager)
//   - OnUpdate() → PreUpdate() with gz::sim::UpdateInfo
//   - event::Events::ConnectWorldUpdateBegin → ISystemPreUpdate interface
//   - transport::Node (gz-transport13) replaces gazebo::transport::Node
//   - ConstVector3dPtr → gz::msgs::Vector3d
//   - boost::bind removed – plain lambda / std::bind used instead
//   - GZ_REGISTER_MODEL_PLUGIN → GZ_ADD_PLUGIN
//   - GAZEBO_MAJOR_VERSION guards removed (Harmonic is always ≥ 8)

#include <uuv_gazebo_plugins/UmbilicalPlugin.hh>
#include <uuv_gazebo_plugins/UmbilicalModel.hh>

// gz-sim
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>

// gz-transport
#include <gz/transport/Node.hh>

// gz-msgs
#include <gz/msgs/vector3d.pb.h>

// gz-utils
#include <gz/utils/AssertionInternalError.hh>

// std
#include <memory>
#include <string>
#include <iostream>
#include <mutex>

namespace gz
{
namespace sim
{

/////////////////////////////////////////////////
class UmbilicalPluginPrivate
{
public:
  /// \brief Model entity this plugin is attached to.
  Entity modelEntity{kNullEntity};

  /// \brief Wrapped gz::sim::Model helper.
  gz::sim::Model model;

  /// \brief The umbilical model implementation (Berg, etc.).
  std::unique_ptr<UmbilicalModel> umbilical;

  /// \brief gz-transport node for subscribing to flow velocity.
  gz::transport::Node node;

  /// \brief Most recently received ocean current velocity.
  gz::math::Vector3d flowVelocity{gz::math::Vector3d::Zero};

  /// \brief Mutex protecting flowVelocity (transport callback runs on a
  ///        separate thread in gz-transport).
  std::mutex flowMutex;

  /// \brief Callback invoked when a new flow velocity message arrives.
  void OnFlowVelocity(const gz::msgs::Vector3d& _msg)
  {
    std::lock_guard<std::mutex> lock(this->flowMutex);
    this->flowVelocity.X(_msg.x());
    this->flowVelocity.Y(_msg.y());
    this->flowVelocity.Z(_msg.z());
  }
};

/////////////////////////////////////////////////
UmbilicalPlugin::UmbilicalPlugin()
  : dataPtr(std::make_unique<UmbilicalPluginPrivate>())
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;
}

/////////////////////////////////////////////////
UmbilicalPlugin::~UmbilicalPlugin() = default;

/////////////////////////////////////////////////
void UmbilicalPlugin::Configure(
    const Entity& _entity,
    const std::shared_ptr<const sdf::Element>& _sdf,
    EntityComponentManager& _ecm,
    EventManager& /*_eventMgr*/)
{
  this->dataPtr->modelEntity = _entity;
  this->dataPtr->model       = gz::sim::Model(_entity);

  // --- Umbilical model ---
  // sdf::Element::GetElement returns a non-const ptr; clone so we can mutate.
  auto sdfClone = std::const_pointer_cast<sdf::Element>(
      _sdf)->GetElement("umbilical_model");

  GZ_UTILS_ASSERT(sdfClone != nullptr,
                  "Could not find <umbilical_model> in plugin SDF.");

  this->dataPtr->umbilical.reset(
      UmbilicalModelFactory::GetInstance().CreateUmbilicalModel(
          sdfClone, _entity));

  GZ_UTILS_ASSERT(this->dataPtr->umbilical != nullptr,
                  "UmbilicalModelFactory returned a null model.");

  this->dataPtr->umbilical->Init();

  // --- Flow velocity subscription ---
  GZ_UTILS_ASSERT(_sdf->HasElement("flow_velocity_topic"),
                  "Umbilical plugin requires <flow_velocity_topic>.");

  std::string flowTopic = std::const_pointer_cast<sdf::Element>(_sdf)
                              ->Get<std::string>("flow_velocity_topic");

  GZ_UTILS_ASSERT(!flowTopic.empty(),
                  "<flow_velocity_topic> tag cannot be empty.");

  // gz-transport callback — runs on transport thread, mutex-protected.
  this->dataPtr->node.Subscribe(
      flowTopic,
      &UmbilicalPluginPrivate::OnFlowVelocity,
      this->dataPtr.get());

  gzmsg << "[UmbilicalPlugin] Subscribed to flow topic: "
        << flowTopic << std::endl;
}

/////////////////////////////////////////////////
// PreUpdate is called every simulation step before physics integration,
// equivalent to the old WorldUpdateBegin event connection.
void UmbilicalPlugin::PreUpdate(
    const UpdateInfo& _info,
    EntityComponentManager& _ecm)
{
  if (_info.paused) return;

  gz::math::Vector3d flow;
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->flowMutex);
    flow = this->dataPtr->flowVelocity;
  }

  this->dataPtr->umbilical->OnUpdate(_info, flow, _ecm);
}

}  // namespace sim
}  // namespace gz

// Register with gz-sim plugin system.
// First arg  = fully-qualified class name
// Second arg = interface(s) this plugin satisfies (comma-separated)
GZ_ADD_PLUGIN(
    gz::sim::UmbilicalPlugin,
    gz::sim::System,
    gz::sim::UmbilicalPlugin::ISystemConfigure,
    gz::sim::UmbilicalPlugin::ISystemPreUpdate)

// Optional alias so the plugin can be referenced without the namespace prefix
// in the SDF <plugin filename="..."> tag.
GZ_ADD_PLUGIN_ALIAS(gz::sim::UmbilicalPlugin, "gz::sim::UmbilicalPlugin")
