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

// ============================================================
// ROS2 / Gazebo Harmonic (gz-sim 8) conversion notes:
//
//  - gazebo/gazebo.hh        → gz/sim/System.hh + gz/sim/Link.hh etc.
//  - ignition::math::*       → gz::math::*
//  - ignition::math::Pose3d  → gz::math::Pose3d
//  - physics::LinkPtr        → gz::sim::Entity  (accessed via gz::sim::Link)
//  - #if GAZEBO_MAJOR_VERSION guards removed; single modern API used.
//  - GZ_ASSERT               → GZ_ASSERT (still valid in gz-sim 8)
//  - link->GetInertial()->Mass()  → gz::sim::Link::WorldInertial
//  - link->AddForceAtRelativePosition / AddRelativeTorque / AddForce
//      → gz::sim::Link::AddWorldForce / AddWorldWrench (applied at CoM by
//        default; CoB offset handled manually here)
// ============================================================

#include <cmath>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/AxisAlignedBox.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Inertial.hh>

#include <uuv_gazebo_plugins/BuoyantObject.hh>

namespace gz {
namespace sim {

/////////////////////////////////////////////////
BuoyantObject::BuoyantObject(gz::sim::Entity _linkEntity,
                             gz::sim::EntityComponentManager &_ecm)
  : linkEntity(_linkEntity)
{
  GZ_ASSERT(_linkEntity != kNullEntity, "Invalid link entity");

  this->volume              = 0.0;
  this->fluidDensity        = 1028.0;   // sea water at 0 °C
  this->g                   = 9.81;
  this->centerOfBuoyancy    = gz::math::Vector3d::Zero;
  this->debugFlag           = false;
  this->isSubmerged         = true;
  this->metacentricWidth    = 0.0;
  this->metacentricLength   = 0.0;
  this->waterLevelPlaneArea = 0.0;
  this->submergedHeight     = 0.0;
  this->isSurfaceVessel     = false;
  this->scalingVolume       = 1.0;
  this->offsetVolume        = 0.0;
  this->isSurfaceVesselFloating = false;
  this->neutrallyBuoyant    = false;

  // Retrieve bounding box from ECM
  gz::sim::Link link(_linkEntity);
  auto bbox = link.WorldInertialPose(_ecm);   // fallback; real bbox below
  // gz-sim 8: axis-aligned bounding box via Model::BoundingBox or collision
  // For correctness the caller should invoke SetBoundingBox() with the
  // shape dimensions after construction (same workaround as classic Gazebo).
  this->boundingBox = gz::math::AxisAlignedBox(
      gz::math::Vector3d(-0.5, -0.5, -0.5),
      gz::math::Vector3d( 0.5,  0.5,  0.5));
}

/////////////////////////////////////////////////
BuoyantObject::~BuoyantObject() {}

/////////////////////////////////////////////////
void BuoyantObject::SetNeutrallyBuoyant(
    gz::sim::EntityComponentManager &_ecm)
{
  this->neutrallyBuoyant = true;
  gz::sim::Link link(this->linkEntity);
  auto inertial = _ecm.Component<gz::sim::components::Inertial>(
      this->linkEntity);
  double mass = inertial ? inertial->Data().MassMatrix().Mass() : 0.0;
  this->volume = mass / this->fluidDensity;
  gzmsg << gz::sim::Link(this->linkEntity).Name(_ecm).value_or("?")
        << " is neutrally buoyant\n";
}

/////////////////////////////////////////////////
void BuoyantObject::GetBuoyancyForce(
    const gz::math::Pose3d      &_pose,
    gz::math::Vector3d          &buoyancyForce,
    gz::math::Vector3d          &buoyancyTorque,
    gz::sim::EntityComponentManager &_ecm)
{
  double height = this->boundingBox.ZLength();
  double z      = _pose.Pos().Z();
  double volume = 0.0;

  buoyancyForce  = gz::math::Vector3d::Zero;
  buoyancyTorque = gz::math::Vector3d::Zero;

  auto inertial = _ecm.Component<gz::sim::components::Inertial>(
      this->linkEntity);
  double mass = inertial ? inertial->Data().MassMatrix().Mass() : 0.0;

  if (!this->isSurfaceVessel)
  {
    if (z + height / 2 > 0 && z < 0)
    {
      this->isSubmerged = false;
      volume = this->GetVolume() * (std::fabs(z) + height / 2) / height;
    }
    else if (z + height / 2 < 0)
    {
      this->isSubmerged = true;
      volume = this->GetVolume();
    }

    if (!this->neutrallyBuoyant || volume != this->volume)
      buoyancyForce = gz::math::Vector3d(
          0, 0, volume * this->fluidDensity * this->g);
    else if (this->neutrallyBuoyant)
      buoyancyForce = gz::math::Vector3d(0, 0, mass * this->g);
  }
  else
  {
    // Linear (small-angle) theory for box-shaped surface vessels.
    // Ref: Fossen, "Handbook of Marine Craft Hydrodynamics and Motion
    //      Control", 2011, p.65.
    if (this->waterLevelPlaneArea <= 0)
    {
      this->waterLevelPlaneArea =
          this->boundingBox.XLength() * this->boundingBox.YLength();
      gzmsg << gz::sim::Link(this->linkEntity).Name(_ecm).value_or("?")
            << "::waterLevelPlaneArea = "
            << this->waterLevelPlaneArea << "\n";
    }

    this->waterLevelPlaneArea =
        mass / (this->fluidDensity * this->submergedHeight);

    GZ_ASSERT(this->waterLevelPlaneArea > 0.0,
              "Water level plane area must be greater than zero");

    double curSubmergedHeight;
    if (z > height / 2.0)
    {
      buoyancyForce  = gz::math::Vector3d::Zero;
      buoyancyTorque = gz::math::Vector3d::Zero;
      return;
    }
    else if (z < -height / 2.0)
      curSubmergedHeight = this->boundingBox.ZLength();
    else
      curSubmergedHeight = height / 2.0 - z;

    volume = curSubmergedHeight * this->waterLevelPlaneArea;
    buoyancyForce = gz::math::Vector3d(
        0, 0, volume * this->fluidDensity * this->g);
    buoyancyTorque = gz::math::Vector3d(
        -1 * this->metacentricWidth  *
             std::sin(_pose.Rot().Roll())  * buoyancyForce.Z(),
        -1 * this->metacentricLength *
             std::sin(_pose.Rot().Pitch()) * buoyancyForce.Z(),
        0);

    this->StoreVector(RESTORING_FORCE, buoyancyForce);
  }

  this->StoreVector(RESTORING_FORCE, buoyancyForce);
}

/////////////////////////////////////////////////
void BuoyantObject::ApplyBuoyancyForce(
    gz::sim::EntityComponentManager &_ecm)
{
  gz::sim::Link link(this->linkEntity);
  gz::math::Pose3d pose = link.WorldPose(_ecm).value_or(gz::math::Pose3d());

  gz::math::Vector3d buoyancyForce, buoyancyTorque;
  this->GetBuoyancyForce(pose, buoyancyForce, buoyancyTorque, _ecm);

  GZ_ASSERT(!std::isnan(buoyancyForce.Length()),  "Buoyancy force is invalid");
  GZ_ASSERT(!std::isnan(buoyancyTorque.Length()), "Buoyancy torque is invalid");

  if (!this->isSurfaceVessel)
  {
    // Apply force at center of buoyancy (expressed in link frame → world)
    gz::math::Vector3d cobWorld =
        pose.Rot().RotateVector(this->GetCoB()) + pose.Pos();
    link.AddWorldForce(_ecm, buoyancyForce, cobWorld);
  }
  else
  {
    link.AddWorldForce(_ecm, buoyancyForce);
    // AddWorldWrench applies torque in world frame at CoM
    link.AddWorldWrench(_ecm,
        gz::math::Vector3d::Zero, buoyancyTorque);
  }
}

/////////////////////////////////////////////////
void BuoyantObject::SetBoundingBox(
    const gz::math::AxisAlignedBox &_bBox)
{
  this->boundingBox = _bBox;
}

/////////////////////////////////////////////////
void BuoyantObject::SetVolume(double _volume)
{
  GZ_ASSERT(_volume > 0, "Invalid input volume");
  this->volume = _volume;
}

/////////////////////////////////////////////////
double BuoyantObject::GetVolume()
{
  return std::max(0.0,
      this->scalingVolume * (this->volume + this->offsetVolume));
}

/////////////////////////////////////////////////
void BuoyantObject::SetFluidDensity(double _fluidDensity)
{
  GZ_ASSERT(_fluidDensity > 0, "Fluid density must be a positive value");
  this->fluidDensity = _fluidDensity;
}

double BuoyantObject::GetFluidDensity() { return this->fluidDensity; }

void BuoyantObject::SetCoB(const gz::math::Vector3d &_cob)
{
  this->centerOfBuoyancy = _cob;
}

gz::math::Vector3d BuoyantObject::GetCoB() { return this->centerOfBuoyancy; }

void BuoyantObject::SetGravity(double _g)
{
  GZ_ASSERT(_g > 0, "Acceleration of gravity must be positive");
  this->g = _g;
}

double BuoyantObject::GetGravity() { return this->g; }

void BuoyantObject::SetDebugFlag(bool _debugOn)
{ this->debugFlag = _debugOn; }

bool BuoyantObject::GetDebugFlag() { return this->debugFlag; }

void BuoyantObject::SetStoreVector(std::string _tag)
{
  if (!this->debugFlag) return;
  if (!this->hydroWrench.count(_tag))
    this->hydroWrench[_tag] = gz::math::Vector3d::Zero;
}

gz::math::Vector3d BuoyantObject::GetStoredVector(std::string _tag)
{
  if (!this->debugFlag) return gz::math::Vector3d::Zero;
  if (this->hydroWrench.count(_tag))
    return this->hydroWrench[_tag];
  return gz::math::Vector3d::Zero;
}

void BuoyantObject::StoreVector(std::string _tag, gz::math::Vector3d _vec)
{
  if (!this->debugFlag) return;
  if (this->hydroWrench.count(_tag))
    this->hydroWrench[_tag] = _vec;
}

bool BuoyantObject::IsSubmerged()      { return this->isSubmerged; }
bool BuoyantObject::IsNeutrallyBuoyant(){ return this->neutrallyBuoyant; }

}  // namespace sim
}  // namespace gz
