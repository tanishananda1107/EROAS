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
//   - ignition::math  → gz::math
//   - gazebo::physics::ModelPtr / LinkPtr → ECS (Entity + ECM)
//   - common::UpdateInfo → gz::sim::UpdateInfo
//   - GZ_ASSERT → GZ_UTILS_ASSERT (gz-utils2)
//   - OnUpdate() now receives EntityComponentManager& for ECS link access
//   - Pose/velocity/force all go through gz::sim::Link helper

#include <uuv_gazebo_plugins/UmbilicalModel.hh>

// gz-sim
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Pose.hh>

// gz-utils
#include <gz/utils/AssertionInternalError.hh>   // GZ_UTILS_ASSERT

// std
#include <iostream>
#include <cmath>

namespace gz
{
namespace sim
{

/////////////////////////////////////////////////
void UmbilicalModel::Init()
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;
}

/////////////////////////////////////////////////
UmbilicalModel* UmbilicalModelFactory::CreateUmbilicalModel(
    const sdf::ElementPtr& _sdf,
    const Entity& _modelEntity)
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;

  if (!_sdf->HasElement("type"))
  {
    std::cerr << "umbilical_model does not have a <type> element" << std::endl;
    return nullptr;
  }

  std::string identifier = _sdf->Get<std::string>("type");

  if (creators_.find(identifier) == creators_.end())
  {
    std::cerr << "Cannot create UmbilicalModel with unknown identifier: "
              << identifier << std::endl;
    return nullptr;
  }

  return creators_[identifier](_sdf, _modelEntity);
}

/////////////////////////////////////////////////
UmbilicalModelFactory& UmbilicalModelFactory::GetInstance()
{
  static UmbilicalModelFactory instance;
  return instance;
}

/////////////////////////////////////////////////
bool UmbilicalModelFactory::RegisterCreator(
    const std::string& _identifier,
    UmbilicalModelCreator _creator)
{
  if (creators_.find(_identifier) != creators_.end())
  {
    std::cerr << "Warning: Registering UmbilicalModel with identifier: "
              << _identifier << " twice" << std::endl;
  }
  creators_[_identifier] = _creator;
  std::cout << "Registered UmbilicalModel type " << _identifier << std::endl;
  return true;
}

/////////////////////////////////////////////////
// ---- UmbilicalModelBerg ----

const std::string UmbilicalModelBerg::IDENTIFIER = "Berg";
REGISTER_UMBILICALMODEL_CREATOR(UmbilicalModelBerg, &UmbilicalModelBerg::create)

UmbilicalModelBerg::UmbilicalModelBerg(
    const sdf::ElementPtr& _sdf,
    const Entity& _modelEntity)
  : modelEntity_(_modelEntity)
{
  GZ_UTILS_ASSERT(_sdf->HasElement("connector_link"),
                  "Could not find <connector_link>.");
  this->connectorLinkName_ = _sdf->Get<std::string>("connector_link");

  GZ_UTILS_ASSERT(_sdf->HasElement("diameter"),
                  "Could not find <diameter>.");
  this->diameter_ = _sdf->Get<double>("diameter");

  GZ_UTILS_ASSERT(_sdf->HasElement("water_density"),
                  "Could not find <water_density>.");
  this->rho_ = _sdf->Get<double>("water_density");
}

/////////////////////////////////////////////////
UmbilicalModel* UmbilicalModelBerg::create(
    const sdf::ElementPtr& _sdf,
    const Entity& _modelEntity)
{
  std::cout << __PRETTY_FUNCTION__ << std::endl;
  return new UmbilicalModelBerg(_sdf, _modelEntity);
}

/////////////////////////////////////////////////
void UmbilicalModelBerg::OnUpdate(
    const UpdateInfo& /*_info*/,
    const gz::math::Vector3d& _flow,
    EntityComponentManager& _ecm)
{
  // Resolve the connector link entity lazily (first call after Configure).
  if (this->connectorLinkEntity_ == kNullEntity)
  {
    gz::sim::Model model(this->modelEntity_);
    this->connectorLinkEntity_ = model.LinkByName(_ecm, this->connectorLinkName_);
    GZ_UTILS_ASSERT(this->connectorLinkEntity_ != kNullEntity,
                    "connector_link entity not found – check the link name.");
  }

  gz::sim::Link link(this->connectorLinkEntity_);

  // Ensure velocity and pose components are enabled.
  link.EnableVelocityChecks(_ecm, true);

  // World pose of the connector link.
  auto poseOpt = link.WorldPose(_ecm);
  if (!poseOpt.has_value()) return;
  const gz::math::Pose3d& pose = poseOpt.value();

  double h = -pose.Pos().Z();

  // Allow some wiggle room when the UUV is near the surface.
  GZ_UTILS_ASSERT(h < 10.0,
                  "Z coordinate should be negative (vehicle is submerged).");

  // Relative flow velocity at the connector link.
  auto worldLinVelOpt = link.WorldLinearVelocity(_ecm);
  if (!worldLinVelOpt.has_value()) return;

  const gz::math::Vector3d uvR = _flow - worldLinVelOpt.value();

  double uR2 = uvR.X() * std::abs(uvR.X());
  double vR2 = uvR.Y() * std::abs(uvR.Y());
  double factor = 0.25 * 1.2 * this->rho_;

  gz::math::Vector3d fWorld(uR2 * factor, vR2 * factor, 0.0);

  // Apply force in the world frame.
  link.AddWorldForce(_ecm, fWorld);
}

}  // namespace sim
}  // namespace gz
