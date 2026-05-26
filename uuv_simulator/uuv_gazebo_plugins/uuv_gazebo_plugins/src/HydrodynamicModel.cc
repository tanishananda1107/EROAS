// Copyright (c) 2016 The UUV Simulator Authors.
// All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License")

// ============================================================
// ROS2 / Gazebo Harmonic (gz-sim 8) conversion notes:
//
//  - All #if GAZEBO_MAJOR_VERSION guards removed; single modern API used.
//  - ignition::math::*      → gz::math::*
//  - ignition::math::Box    → gz::math::AxisAlignedBox
//  - physics::LinkPtr       → gz::sim::Entity  (via gz::sim::Link wrapper)
//  - link->GetInertial()->Mass()  → ECM component lookup (done in BuoyantObject)
//  - link->GetWorldPose()   → gz::sim::Link::WorldPose(_ecm)
//  - link->RelativeLinearVel()    → gz::sim::Link::WorldLinearVelocity(_ecm)
//    (then rotated to body frame)
//  - link->AddRelativeForce / AddRelativeTorque
//      → gz::sim::Link::AddWorldForce / AddWorldWrench
//  - gz-sim 8 system plugins use ISystemPreUpdate / ISystemUpdate callbacks;
//    the ECM is passed each tick so we thread it through ApplyHydrodynamicForces.
// ============================================================

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/AxisAlignedBox.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/common/Console.hh>

#include <uuv_gazebo_plugins/HydrodynamicModel.hh>

namespace gz {
namespace sim {

/////////////////////////////////////////////////
HydrodynamicModel::HydrodynamicModel(
    sdf::ElementPtr                  _sdf,
    gz::sim::Entity                  _linkEntity,
    gz::sim::EntityComponentManager &_ecm)
  : BuoyantObject(_linkEntity, _ecm)
{
  GZ_ASSERT(_linkEntity != kNullEntity, "Invalid link entity");

  this->filteredAcc.setZero();
  this->lastVelRel.setZero();

  if (_sdf->HasElement("volume"))
    this->volume = _sdf->Get<double>("volume");

  // Surface vessel / floating body parameters
  if (_sdf->HasElement("metacentric_width") &&
      _sdf->HasElement("metacentric_length") &&
      _sdf->HasElement("submerged_height"))
  {
    this->metacentricWidth  = _sdf->Get<double>("metacentric_width");
    this->metacentricLength = _sdf->Get<double>("metacentric_length");
    this->submergedHeight   = _sdf->Get<double>("submerged_height");
    this->isSurfaceVessel   = true;

    gzmsg << "Surface vessel parameters\n"
          << "\tMetacentric width  [m]=" << this->metacentricWidth  << "\n"
          << "\tMetacentric length [m]=" << this->metacentricLength << "\n"
          << "\tSubmerged height   [m]=" << this->submergedHeight   << "\n";
  }
  else
  {
    this->metacentricWidth    = 0.0;
    this->metacentricLength   = 0.0;
    this->waterLevelPlaneArea = 0.0;
    this->isSurfaceVessel     = false;
  }

  // Center of buoyancy
  if (_sdf->HasElement("center_of_buoyancy"))
  {
    auto cob = Str2Vector(_sdf->Get<std::string>("center_of_buoyancy"));
    this->SetCoB(gz::math::Vector3d(cob[0], cob[1], cob[2]));
  }

  // Override bounding box from SDF (workaround for Gazebo bbox inaccuracy)
  if (_sdf->HasElement("box"))
  {
    sdf::ElementPtr sdfBox = _sdf->GetElement("box");
    if (sdfBox->HasElement("width") &&
        sdfBox->HasElement("length") &&
        sdfBox->HasElement("height"))
    {
      double w = sdfBox->Get<double>("width");
      double l = sdfBox->Get<double>("length");
      double h = sdfBox->Get<double>("height");
      gz::math::AxisAlignedBox bbox(
          gz::math::Vector3d(-w/2, -l/2, -h/2),
          gz::math::Vector3d( w/2,  l/2,  h/2));
      this->SetBoundingBox(bbox);
    }
  }

  if (_sdf->HasElement("neutrally_buoyant"))
    if (_sdf->Get<bool>("neutrally_buoyant"))
      this->SetNeutrallyBuoyant(_ecm);

  this->Re          = 0;
  this->temperature = 0;
}

/////////////////////////////////////////////////
void HydrodynamicModel::ComputeAcc(Eigen::Vector6d _velRel,
                                   double          _time,
                                   double          _alpha)
{
  double dt = _time - lastTime;
  if (dt <= 0.0 || this->lastVelRel(0) == 0.0) return;

  Eigen::Vector6d acc = (_velRel - this->lastVelRel) / dt;
  this->filteredAcc   = (1.0 - _alpha) * this->filteredAcc + _alpha * acc;

  lastTime         = _time;
  this->lastVelRel = _velRel;
}

/////////////////////////////////////////////////
gz::math::Vector3d HydrodynamicModel::ToNED(gz::math::Vector3d _vec)
{
  _vec.Y() *= -1;
  _vec.Z() *= -1;
  return _vec;
}

gz::math::Vector3d HydrodynamicModel::FromNED(gz::math::Vector3d _vec)
{
  return this->ToNED(_vec);  // same transform
}

/////////////////////////////////////////////////
bool HydrodynamicModel::CheckParams(sdf::ElementPtr _sdf)
{
  for (auto &tag : this->params)
  {
    if (!_sdf->HasElement(tag))
    {
      gzerr << "Hydrodynamic model: Expected element " << tag << "\n";
      return false;
    }
  }
  return true;
}

/////////////////////////////////////////////////
HydrodynamicModel* HydrodynamicModelFactory::CreateHydrodynamicModel(
    sdf::ElementPtr                  _sdf,
    gz::sim::Entity                  _link,
    gz::sim::EntityComponentManager &_ecm)
{
  GZ_ASSERT(_sdf->HasElement("hydrodynamic_model"),
            "Hydrodynamic model element is missing");
  sdf::ElementPtr sdfModel = _sdf->GetElement("hydrodynamic_model");

  if (!sdfModel->HasElement("type"))
  {
    std::cerr << "Hydrodynamic model has no type\n";
    return nullptr;
  }

  std::string identifier = sdfModel->Get<std::string>("type");

  if (creators_.find(identifier) == creators_.end())
  {
    std::cerr << "Cannot create HydrodynamicModel with unknown identifier: "
              << identifier << "\n";
    return nullptr;
  }

  return creators_[identifier](_sdf, _link, _ecm);
}

/////////////////////////////////////////////////
HydrodynamicModelFactory& HydrodynamicModelFactory::GetInstance()
{
  static HydrodynamicModelFactory instance;
  return instance;
}

/////////////////////////////////////////////////
bool HydrodynamicModelFactory::RegisterCreator(
    const std::string          &_identifier,
    HydrodynamicModelCreator    _creator)
{
  if (creators_.find(_identifier) != creators_.end())
  {
    std::cerr << "Warning: Registering HydrodynamicModel with identifier: "
              << _identifier << " twice\n";
  }
  creators_[_identifier] = _creator;
  std::cout << "Registered HydrodynamicModel type " << _identifier << "\n";
  return true;
}

// ===========================================================================
// HMFossen
// ===========================================================================
const std::string HMFossen::IDENTIFIER = "fossen";
REGISTER_HYDRODYNAMICMODEL_CREATOR(HMFossen, &HMFossen::create);

HydrodynamicModel* HMFossen::create(
    sdf::ElementPtr                  _sdf,
    gz::sim::Entity                  _link,
    gz::sim::EntityComponentManager &_ecm)
{
  return new HMFossen(_sdf, _link, _ecm);
}

HMFossen::HMFossen(sdf::ElementPtr                  _sdf,
                   gz::sim::Entity                  _link,
                   gz::sim::EntityComponentManager &_ecm)
  : HydrodynamicModel(_sdf, _link, _ecm)
{
  std::vector<double> addedMass(36, 0.0);
  std::vector<double> linDampCoef(6,  0.0);
  std::vector<double> linDampForward(6, 0.0);
  std::vector<double> quadDampCoef(6, 0.0);

  GZ_ASSERT(_sdf->HasElement("hydrodynamic_model"),
            "Hydrodynamic model element is missing");

  sdf::ElementPtr mp = _sdf->GetElement("hydrodynamic_model");

  if (mp->HasElement("added_mass"))
    addedMass = Str2Vector(mp->Get<std::string>("added_mass"));
  else
    gzmsg << "HMFossen: Using added mass NULL\n";

  this->params.push_back("added_mass");

  if (mp->HasElement("linear_damping"))
    linDampCoef = Str2Vector(mp->Get<std::string>("linear_damping"));
  else
    gzmsg << "HMFossen: Using linear damping NULL\n";

  this->scalingAddedMass  = 1.0;
  this->offsetAddedMass   = 0.0;
  this->params.push_back("scaling_added_mass");
  this->params.push_back("offset_added_mass");
  this->params.push_back("linear_damping");

  if (mp->HasElement("linear_damping_forward_speed"))
    linDampForward = Str2Vector(
        mp->Get<std::string>("linear_damping_forward_speed"));
  else
    gzmsg << "HMFossen: Using linear damping forward speed NULL\n";
  this->params.push_back("linear_damping_forward_speed");

  if (mp->HasElement("quadratic_damping"))
    quadDampCoef = Str2Vector(mp->Get<std::string>("quadratic_damping"));
  else
    gzmsg << "HMFossen: Using quadratic damping NULL\n";

  this->params.push_back("quadratic_damping");
  this->scalingDamping              = 1.0;
  this->offsetLinearDamping         = 0.0;
  this->offsetLinForwardSpeedDamping= 0.0;
  this->offsetNonLinDamping         = 0.0;
  this->params.push_back("scaling_damping");
  this->params.push_back("offset_linear_damping");
  this->params.push_back("offset_lin_forward_speed_damping");
  this->params.push_back("offset_nonlin_damping");
  this->params.push_back("volume");
  this->params.push_back("scaling_volume");

  GZ_ASSERT(addedMass.size() == 36,
            "Added-mass vector must have 36 elements");
  GZ_ASSERT(linDampCoef.size() == 6 || linDampCoef.size() == 36,
            "Linear damping vector must have 6 or 36 elements");
  GZ_ASSERT(linDampForward.size() == 6 || linDampForward.size() == 36,
            "Linear forward-speed damping vector must have 6 or 36 elements");
  GZ_ASSERT(quadDampCoef.size() == 6 || quadDampCoef.size() == 36,
            "Quadratic damping vector must have 6 or 36 elements");

  this->DLin.setZero();
  this->DNonLin.setZero();
  this->DLinForwardSpeed.setZero();

  for (int r = 0; r < 6; r++)
    for (int c = 0; c < 6; c++)
    {
      this->Ma(r, c) = addedMass[6*r+c];
      if (linDampCoef.size()    == 36) this->DLin(r, c)           = linDampCoef[6*r+c];
      if (quadDampCoef.size()   == 36) this->DNonLin(r, c)        = quadDampCoef[6*r+c];
      if (linDampForward.size() == 36) this->DLinForwardSpeed(r,c) = linDampForward[6*r+c];
    }

  for (int i = 0; i < 6; i++)
  {
    if (linDampCoef.size()    == 6) this->DLin(i, i)            = linDampCoef[i];
    if (quadDampCoef.size()   == 6) this->DNonLin(i, i)         = quadDampCoef[i];
    if (linDampForward.size() == 6) this->DLinForwardSpeed(i, i) = linDampForward[i];
  }

  this->linearDampCoef = linDampCoef;
  this->quadDampCoef   = quadDampCoef;
}

/////////////////////////////////////////////////
void HMFossen::ApplyHydrodynamicForces(
    double                           _time,
    const gz::math::Vector3d        &_flowVelWorld,
    gz::sim::EntityComponentManager &_ecm)
{
  gz::sim::Link link(this->linkEntity);

  gz::math::Pose3d pose =
      link.WorldPose(_ecm).value_or(gz::math::Pose3d());

  // World-frame velocities → rotate to body frame
  gz::math::Vector3d linVelWorld =
      link.WorldLinearVelocity(_ecm).value_or(gz::math::Vector3d::Zero);
  gz::math::Vector3d angVelWorld =
      link.WorldAngularVelocity(_ecm).value_or(gz::math::Vector3d::Zero);

  gz::math::Vector3d linVel = pose.Rot().RotateVectorReverse(linVelWorld);
  gz::math::Vector3d angVel = pose.Rot().RotateVectorReverse(angVelWorld);

  // Flow in body frame
  gz::math::Vector3d flowVel =
      pose.Rot().RotateVectorReverse(_flowVelWorld);

  Eigen::Vector6d velRel;
  velRel = EigenStack(this->ToNED(linVel - flowVel),
                      this->ToNED(angVel));

  this->ComputeAddedCoriolisMatrix(velRel, this->Ma, this->Ca);
  this->ComputeDampingMatrix(velRel, this->D);
  this->ComputeAcc(velRel, _time, 0.3);

  Eigen::Vector6d damping = -this->D               * velRel;
  Eigen::Vector6d added   = -this->GetAddedMass()  * this->filteredAcc;
  Eigen::Vector6d cor     = -this->Ca              * velRel;
  Eigen::Vector6d tau     =  damping + added + cor;

  GZ_ASSERT(!std::isnan(tau.norm()), "Hydrodynamic forces vector is NaN");

  if (!std::isnan(tau.norm()))
  {
    // Convert back from NED body frame to Gazebo world frame
    gz::math::Vector3d hydForce  =
        this->FromNED(Vec3dToGz(tau.head<3>()));
    gz::math::Vector3d hydTorque =
        this->FromNED(Vec3dToGz(tau.tail<3>()));

    // Rotate body-frame force/torque to world frame
    hydForce  = pose.Rot().RotateVector(hydForce);
    hydTorque = pose.Rot().RotateVector(hydTorque);

    link.AddWorldForce(_ecm, hydForce);
    link.AddWorldWrench(_ecm, gz::math::Vector3d::Zero, hydTorque);
  }

  this->ApplyBuoyancyForce(_ecm);

  if (this->debugFlag)
  {
    this->StoreVector(UUV_DAMPING_FORCE,   Vec3dToGz(damping.head<3>()));
    this->StoreVector(UUV_DAMPING_TORQUE,  Vec3dToGz(damping.tail<3>()));
    this->StoreVector(UUV_ADDED_MASS_FORCE,  Vec3dToGz(added.head<3>()));
    this->StoreVector(UUV_ADDED_MASS_TORQUE, Vec3dToGz(added.tail<3>()));
    this->StoreVector(UUV_ADDED_CORIOLIS_FORCE,  Vec3dToGz(cor.head<3>()));
    this->StoreVector(UUV_ADDED_CORIOLIS_TORQUE, Vec3dToGz(cor.tail<3>()));
  }
}

/////////////////////////////////////////////////
void HMFossen::ComputeAddedCoriolisMatrix(const Eigen::Vector6d &_vel,
                                          const Eigen::Matrix6d &_Ma,
                                          Eigen::Matrix6d       &_Ca) const
{
  Eigen::Vector6d ab = this->GetAddedMass() * _vel;
  Eigen::Matrix3d Sa = -1 * CrossProductOperator(ab.head<3>());
  _Ca << Eigen::Matrix3d::Zero(), Sa,
         Sa, -1 * CrossProductOperator(ab.tail<3>());
}

/////////////////////////////////////////////////
void HMFossen::ComputeDampingMatrix(const Eigen::Vector6d &_vel,
                                    Eigen::Matrix6d       &_D) const
{
  _D.setZero();
  _D = -1 * (this->DLin +
             this->offsetLinearDamping * Eigen::Matrix6d::Identity()) -
       _vel[0] * (this->DLinForwardSpeed +
                  this->offsetLinForwardSpeedDamping * Eigen::Matrix6d::Identity());

  for (int i = 0; i < 6; i++)
    _D(i, i) += -1 * (this->DNonLin(i, i) + this->offsetNonLinDamping) *
                std::fabs(_vel[i]);

  _D *= this->scalingDamping;
}

Eigen::Matrix6d HMFossen::GetAddedMass() const
{
  return this->scalingAddedMass *
         (this->Ma + this->offsetAddedMass * Eigen::Matrix6d::Identity());
}

/////////////////////////////////////////////////
bool HMFossen::GetParam(std::string _tag, std::vector<double> &_output)
{
  _output.clear();
  auto pushMatrix = [&](const Eigen::Matrix6d &M) {
    for (int i = 0; i < 6; i++)
      for (int j = 0; j < 6; j++)
        _output.push_back(M(i, j));
  };

  if      (_tag == "added_mass")                  pushMatrix(this->Ma);
  else if (_tag == "linear_damping")              pushMatrix(this->DLin);
  else if (_tag == "linear_damping_forward_speed")pushMatrix(this->DLinForwardSpeed);
  else if (_tag == "quadratic_damping")           pushMatrix(this->DNonLin);
  else if (_tag == "center_of_buoyancy")
  {
    _output = {this->centerOfBuoyancy.X(),
               this->centerOfBuoyancy.Y(),
               this->centerOfBuoyancy.Z()};
  }
  else return false;

  gzmsg << "HydrodynamicModel::GetParam <" << _tag << ">\n";
  return true;
}

bool HMFossen::GetParam(std::string _tag, double &_output)
{
  _output = -1.0;
  if      (_tag == "volume")                       _output = this->volume;
  else if (_tag == "scaling_volume")               _output = this->scalingVolume;
  else if (_tag == "scaling_added_mass")           _output = this->scalingAddedMass;
  else if (_tag == "scaling_damping")              _output = this->scalingDamping;
  else if (_tag == "fluid_density")                _output = this->fluidDensity;
  else if (_tag == "bbox_height")                  _output = this->boundingBox.ZLength();
  else if (_tag == "bbox_width")                   _output = this->boundingBox.YLength();
  else if (_tag == "bbox_length")                  _output = this->boundingBox.XLength();
  else if (_tag == "offset_volume")                _output = this->offsetVolume;
  else if (_tag == "offset_added_mass")            _output = this->offsetAddedMass;
  else if (_tag == "offset_linear_damping")        _output = this->offsetLinearDamping;
  else if (_tag == "offset_lin_forward_speed_damping") _output = this->offsetLinForwardSpeedDamping;
  else if (_tag == "offset_nonlin_damping")        _output = this->offsetNonLinDamping;
  else { _output = -1.0; return false; }

  gzmsg << "HydrodynamicModel::GetParam <" << _tag << ">=" << _output << "\n";
  return true;
}

bool HMFossen::SetParam(std::string _tag, double _input)
{
  if (_tag == "scaling_volume")
  { if (_input < 0) return false; this->scalingVolume = _input; }
  else if (_tag == "scaling_added_mass")
  { if (_input < 0) return false; this->scalingAddedMass = _input; }
  else if (_tag == "scaling_damping")
  { if (_input < 0) return false; this->scalingDamping = _input; }
  else if (_tag == "fluid_density")
  { if (_input < 0) return false; this->fluidDensity = _input; }
  else if (_tag == "offset_volume")                this->offsetVolume = _input;
  else if (_tag == "offset_added_mass")            this->offsetAddedMass = _input;
  else if (_tag == "offset_linear_damping")        this->offsetLinearDamping = _input;
  else if (_tag == "offset_lin_forward_speed_damping") this->offsetLinForwardSpeedDamping = _input;
  else if (_tag == "offset_nonlin_damping")        this->offsetNonLinDamping = _input;
  else return false;

  gzmsg << "HydrodynamicModel::SetParam <" << _tag << ">=" << _input << "\n";
  return true;
}

void HMFossen::Print(std::string _paramName, std::string _message)
{
  if (_paramName == "all")
  {
    for (auto &tag : this->params) this->Print(tag);
    return;
  }
  std::cout << (_message.empty() ? _paramName : _message) << "\n";
  auto printMatrix = [](const Eigen::Matrix6d &M) {
    for (int i = 0; i < 6; i++) {
      for (int j = 0; j < 6; j++) std::cout << std::setw(12) << M(i,j);
      std::cout << "\n";
    }
  };
  if      (_paramName == "added_mass")                   printMatrix(this->Ma);
  else if (_paramName == "linear_damping")               printMatrix(this->DLin);
  else if (_paramName == "linear_damping_forward_speed") printMatrix(this->DLinForwardSpeed);
  else if (_paramName == "quadratic_damping")            printMatrix(this->DNonLin);
  else if (_paramName == "volume")
    std::cout << std::setw(12) << this->volume << " m^3\n";
}

// ===========================================================================
// HMSphere
// ===========================================================================
const std::string HMSphere::IDENTIFIER = "sphere";
REGISTER_HYDRODYNAMICMODEL_CREATOR(HMSphere, &HMSphere::create);

HydrodynamicModel* HMSphere::create(
    sdf::ElementPtr _sdf, gz::sim::Entity _link,
    gz::sim::EntityComponentManager &_ecm)
{ return new HMSphere(_sdf, _link, _ecm); }

HMSphere::HMSphere(sdf::ElementPtr _sdf, gz::sim::Entity _link,
                   gz::sim::EntityComponentManager &_ecm)
  : HMFossen(_sdf, _link, _ecm)
{
  sdf::ElementPtr mp = _sdf->GetElement("hydrodynamic_model");

  if (mp->HasElement("radius"))
    this->radius = mp->Get<double>("radius");
  else
  {
    gzmsg << "HMSphere: Using smallest bbox dimension as radius\n";
    this->radius = std::min({this->boundingBox.XLength(),
                             this->boundingBox.YLength(),
                             this->boundingBox.ZLength()});
  }
  gzmsg << "HMSphere::radius=" << this->radius << "\n";

  this->params.push_back("radius");
  this->Re          = 3e5;
  this->Cd          = 0.5;
  this->areaSection = GZ_PI * std::pow(this->radius, 2.0);

  double sphereMa = -2.0/3.0 * this->fluidDensity * GZ_PI *
                    std::pow(this->radius, 3.0);
  double Dq = -0.5 * this->fluidDensity * this->Cd * this->areaSection;

  for (int i = 0; i < 3; i++)
  {
    this->Ma(i, i)      = -sphereMa;
    this->DNonLin(i, i) = Dq;
  }
}

void HMSphere::Print(std::string _paramName, std::string _message)
{
  if (_paramName == "all")
  { for (auto &tag : this->params) this->Print(tag); return; }
  if (!_message.empty()) std::cout << _message << "\n";
  if (_paramName == "radius")
    std::cout << std::setw(12) << this->radius << "\n";
  else
    HMFossen::Print(_paramName, _message);
}

// ===========================================================================
// HMCylinder
// ===========================================================================
const std::string HMCylinder::IDENTIFIER = "cylinder";
REGISTER_HYDRODYNAMICMODEL_CREATOR(HMCylinder, &HMCylinder::create);

HydrodynamicModel* HMCylinder::create(
    sdf::ElementPtr _sdf, gz::sim::Entity _link,
    gz::sim::EntityComponentManager &_ecm)
{ return new HMCylinder(_sdf, _link, _ecm); }

HMCylinder::HMCylinder(sdf::ElementPtr _sdf, gz::sim::Entity _link,
                       gz::sim::EntityComponentManager &_ecm)
  : HMFossen(_sdf, _link, _ecm)
{
  sdf::ElementPtr mp = _sdf->GetElement("hydrodynamic_model");

  if (mp->HasElement("radius"))
    this->radius = mp->Get<double>("radius");
  else
    this->radius = std::min({this->boundingBox.XLength(),
                             this->boundingBox.YLength(),
                             this->boundingBox.ZLength()});

  if (mp->HasElement("length"))
    this->length = mp->Get<double>("length");
  else
    this->length = std::max({this->boundingBox.XLength(),
                             this->boundingBox.YLength(),
                             this->boundingBox.ZLength()});

  this->dimRatio = this->length / (2 * this->radius);

  // Drag coefficients (circular & rectangular profiles)
  if      (this->dimRatio <= 1)                      { this->cdCirc = 0.91; this->cdLength = 0.63; }
  else if (this->dimRatio > 1 && this->dimRatio <= 2) { this->cdCirc = 0.85; this->cdLength = 0.68; }
  else if (this->dimRatio > 2 && this->dimRatio <= 4) { this->cdCirc = 0.87; this->cdLength = 0.74; }
  else if (this->dimRatio > 4 && this->dimRatio <= 5) { this->cdCirc = 0.99; this->cdLength = 0.74; }
  else if (this->dimRatio > 5 && this->dimRatio <= 7) { this->cdCirc = 0.99; this->cdLength = 0.82; }
  else if (this->dimRatio > 7 && this->dimRatio <= 10){ this->cdCirc = 0.99; this->cdLength = 0.82; }
  else                                                 { this->cdCirc = 0.99; this->cdLength = 0.98; }

  if (mp->HasElement("axis"))
  {
    this->axis = mp->Get<std::string>("axis");
    GZ_ASSERT(this->axis == "i" || this->axis == "j" || this->axis == "k",
              "Invalid axis");
  }
  else
  {
    double maxLen = std::max({this->boundingBox.XLength(),
                              this->boundingBox.YLength(),
                              this->boundingBox.ZLength()});
    this->axis = (maxLen == this->boundingBox.XLength()) ? "i" :
                 (maxLen == this->boundingBox.YLength()) ? "j" : "k";
  }

  double MaLen  = -this->fluidDensity * GZ_PI *
                   std::pow(this->radius, 2.0) * this->length;
  double MaCirc = -this->fluidDensity * GZ_PI * std::pow(this->radius, 2.0);
  double MaLenT = (-1.0/12.0) * this->fluidDensity * GZ_PI *
                   std::pow(this->radius, 2.0) * std::pow(this->length, 3.0);
  double DCirc  = -0.5 * this->cdCirc  * GZ_PI * std::pow(this->radius, 2.0) * this->fluidDensity;
  double DLen   = -0.5 * this->cdLength * this->radius * this->length          * this->fluidDensity;

  if (this->axis == "i")
  {
    this->Ma(0,0)=-MaCirc; this->Ma(1,1)=-MaLen; this->Ma(2,2)=-MaLen;
    this->Ma(4,4)=-MaLenT; this->Ma(5,5)=-MaLenT;
    this->DNonLin(0,0)=DCirc; this->DNonLin(1,1)=DLen; this->DNonLin(2,2)=DLen;
  }
  else if (this->axis == "j")
  {
    this->Ma(0,0)=-MaLen; this->Ma(1,1)=-MaCirc; this->Ma(2,2)=-MaLen;
    this->Ma(3,3)=-MaLenT; this->Ma(5,5)=-MaLenT;
    this->DNonLin(0,0)=DLen; this->DNonLin(1,1)=DCirc; this->DNonLin(2,2)=DLen;
  }
  else
  {
    this->Ma(0,0)=-MaLen; this->Ma(1,1)=-MaLen; this->Ma(2,2)=-MaCirc;
    this->Ma(3,3)=-MaLenT; this->Ma(4,4)=-MaLenT;
    this->DNonLin(0,0)=DLen; this->DNonLin(1,1)=DLen; this->DNonLin(2,2)=DCirc;
  }
}

void HMCylinder::Print(std::string _paramName, std::string _message)
{
  if (!_message.empty()) gzmsg << _message << "\n";
  if      (_paramName == "radius") std::cout << std::setw(12) << this->radius << "\n";
  else if (_paramName == "length") std::cout << std::setw(12) << this->length << "\n";
  else HMFossen::Print(_paramName, _message);
}

// ===========================================================================
// HMSpheroid
// ===========================================================================
const std::string HMSpheroid::IDENTIFIER = "spheroid";
REGISTER_HYDRODYNAMICMODEL_CREATOR(HMSpheroid, &HMSpheroid::create);

HydrodynamicModel* HMSpheroid::create(
    sdf::ElementPtr _sdf, gz::sim::Entity _link,
    gz::sim::EntityComponentManager &_ecm)
{ return new HMSpheroid(_sdf, _link, _ecm); }

HMSpheroid::HMSpheroid(sdf::ElementPtr _sdf, gz::sim::Entity _link,
                       gz::sim::EntityComponentManager &_ecm)
  : HMFossen(_sdf, _link, _ecm)
{
  gzerr << "Hydrodynamic model for a spheroid is still in development!\n";

  sdf::ElementPtr mp = _sdf->GetElement("hydrodynamic_model");

  if (mp->HasElement("radius"))
    this->radius = mp->Get<double>("radius");
  else
    this->radius = std::min({this->boundingBox.XLength(),
                             this->boundingBox.YLength(),
                             this->boundingBox.ZLength()});
  GZ_ASSERT(this->radius > 0, "Radius cannot be negative");

  if (mp->HasElement("length"))
    this->length = mp->Get<double>("length");
  else
    this->length = std::max({this->boundingBox.XLength(),
                             this->boundingBox.YLength(),
                             this->boundingBox.ZLength()});
  GZ_ASSERT(this->length > 0, "Length cannot be negative");

  double ecc = std::sqrt(1 - std::pow(this->radius / this->length, 2.0));
  double ln  = std::log((1 + ecc) / (1 - ecc));
  double alpha = 2 * (1 - std::pow(ecc, 2.0)) / std::pow(ecc, 3.0) *
                 (0.5 * ln - ecc);
  double beta  = 1 / std::pow(ecc, 2.0) -
                 (1 - std::pow(ecc, 2.0)) / (2 * std::pow(ecc, 3.0)) * ln;

  auto inertial = _ecm.Component<gz::sim::components::Inertial>(
      this->linkEntity);
  double mass = inertial ? inertial->Data().MassMatrix().Mass() : 0.0;

  this->Ma(0,0) = mass * alpha / (2 - alpha);
  this->Ma(1,1) = mass * beta  / (2 - beta);
  this->Ma(2,2) = this->Ma(1,1);
  this->Ma(3,3) = 0;

  double ba_minus = std::pow(this->radius, 2.0) - std::pow(this->length, 2.0);
  double ba_plus  = std::pow(this->radius, 2.0) + std::pow(this->length, 2.0);
  this->Ma(4,4) = -0.2 * mass * std::pow(ba_minus, 2.0) * (alpha - beta) /
                  (2 * ba_minus - ba_plus * (alpha - beta));
  this->Ma(5,5) = this->Ma(4,4);
}

void HMSpheroid::Print(std::string _paramName, std::string _message)
{
  if (!_message.empty()) gzmsg << _message << "\n";
  if      (_paramName == "radius") std::cout << std::setw(12) << this->radius << "\n";
  else if (_paramName == "length") std::cout << std::setw(12) << this->length << "\n";
  else HMFossen::Print(_paramName, _message);
}

// ===========================================================================
// HMBox
// ===========================================================================
const std::string HMBox::IDENTIFIER = "box";
REGISTER_HYDRODYNAMICMODEL_CREATOR(HMBox, &HMBox::create);

HydrodynamicModel* HMBox::create(
    sdf::ElementPtr _sdf, gz::sim::Entity _link,
    gz::sim::EntityComponentManager &_ecm)
{ return new HMBox(_sdf, _link, _ecm); }

HMBox::HMBox(sdf::ElementPtr _sdf, gz::sim::Entity _link,
             gz::sim::EntityComponentManager &_ecm)
  : HMFossen(_sdf, _link, _ecm)
{
  gzerr << "Hydrodynamic model for box is still in development!\n";

  sdf::ElementPtr mp = _sdf->GetElement("hydrodynamic_model");

  this->Cd = mp->HasElement("cd") ? mp->Get<double>("cd") : 1.0;

  GZ_ASSERT(mp->HasElement("length"), "Length missing");
  GZ_ASSERT(mp->HasElement("width"),  "Width missing");
  GZ_ASSERT(mp->HasElement("height"), "Height missing");

  this->length = mp->Get<double>("length");
  this->width  = mp->Get<double>("width");
  this->height = mp->Get<double>("height");

  this->quadDampCoef[0] = -0.5 * this->Cd * this->width  * this->height * this->fluidDensity;
  this->quadDampCoef[1] = -0.5 * this->Cd * this->length * this->height * this->fluidDensity;
  this->quadDampCoef[2] = -0.5 * this->Cd * this->width  * this->length * this->fluidDensity;
}

void HMBox::Print(std::string _paramName, std::string _message)
{
  if (!_message.empty()) gzmsg << _message << "\n";
  if      (_paramName == "length") std::cout << std::setw(12) << this->length << "\n";
  else if (_paramName == "width")  std::cout << std::setw(12) << this->width  << "\n";
  else if (_paramName == "height") std::cout << std::setw(12) << this->height << "\n";
  else HMFossen::Print(_paramName, _message);
}

}  // namespace sim
}  // namespace gz
