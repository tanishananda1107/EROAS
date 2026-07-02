// Copyright (c) 2016 The UUV Simulator Authors.
// All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License")

// ============================================================
// ROS2 / Gazebo Harmonic (gz-sim 8) conversion notes:
//
//  - #include <gazebo/gazebo.hh>  →  #include <gz/sim/System.hh>
//  - ignition::math::Vector3d     →  gz::math::Vector3d
//  - ignition::math::Vector3d::UnitZ → gz::math::Vector3d::UnitZ
//  - gzmsg kept (available via gz/common/Console.hh)
//  - No gazebo physics types used here → minimal changes needed.
// ============================================================

#include <gz/sim/System.hh>
#include <gz/math/Vector3.hh>
#include <gz/common/Console.hh>

#include <uuv_gazebo_plugins/LiftDragModel.hh>

namespace uuv_gz_plugins
{

/////////////////////////////////////////////////
bool LiftDrag::CheckForElement(sdf::ElementPtr _sdf,
                               const std::string &element)
{
  if (!_sdf->HasElement(element))
  {
    std::cerr << "LiftDrag: Missing required element: " << element << "\n";
    return false;
  }
  return true;
}

/////////////////////////////////////////////////
LiftDrag* LiftDragFactory::CreateLiftDrag(sdf::ElementPtr _sdf)
{
  if (!_sdf->HasElement("type"))
  {
    std::cerr << "liftdrag does not have a type element\n";
    return nullptr;
  }

  std::string identifier = _sdf->Get<std::string>("type");

  if (creators_.find(identifier) == creators_.end())
  {
    std::cerr << "Cannot create LiftDrag with unknown identifier: "
              << identifier << "\n";
    return nullptr;
  }

  return creators_[identifier](_sdf);
}

/////////////////////////////////////////////////
LiftDragFactory& LiftDragFactory::GetInstance()
{
  static LiftDragFactory instance;
  return instance;
}

/////////////////////////////////////////////////
bool LiftDragFactory::RegisterCreator(const std::string &_identifier,
                                      LiftDragCreator    _creator)
{
  if (creators_.find(_identifier) != creators_.end())
  {
    std::cerr << "Warning: Registering LiftDrag with identifier: "
              << _identifier << " twice\n";
  }
  creators_[_identifier] = _creator;
  std::cout << "Registered LiftDrag type " << _identifier << "\n";
  return true;
}

// ---------------------------------------------------------------------------
// LiftDragQuadratic
// ---------------------------------------------------------------------------
const std::string LiftDragQuadratic::IDENTIFIER = "Quadratic";
REGISTER_LIFTDRAG_CREATOR(LiftDragQuadratic, &LiftDragQuadratic::create)

LiftDrag* LiftDragQuadratic::create(sdf::ElementPtr _sdf)
{
  if (!_sdf->HasElement("lift_constant"))
  {
    std::cerr << "LiftDragQuadratic: expected element lift_constant\n";
    return nullptr;
  }
  if (!_sdf->HasElement("drag_constant"))
  {
    std::cerr << "LiftDragQuadratic: expected element drag_constant\n";
    return nullptr;
  }

  gzmsg << "Lift constant= " << _sdf->Get<double>("lift_constant") << "\n";
  gzmsg << "Drag constant= " << _sdf->Get<double>("drag_constant") << "\n";

  return new LiftDragQuadratic(_sdf->Get<double>("lift_constant"),
                               _sdf->Get<double>("drag_constant"));
}

gz::math::Vector3d LiftDragQuadratic::compute(
    const gz::math::Vector3d &_velL)
{
  gz::math::Vector3d velL = _velL;
  double angle = atan2(_velL.Y(), _velL.X());

  if (angle > GZ_PI_2)       { angle -= GZ_PI; velL = -_velL; }
  else if (angle < -GZ_PI_2) { angle += GZ_PI; velL = -_velL; }

  double u   = velL.Length();
  double u2  = u * u;
  double du2 = angle * u2;

  double drag = angle * du2 * this->dragConstant;
  double lift = du2 * this->liftConstant;

  gz::math::Vector3d liftDir =
      -(gz::math::Vector3d::UnitZ.Cross(_velL)).Normalize();
  gz::math::Vector3d dragDir = -_velL;

  return lift * liftDir + drag * dragDir.Normalize();
}

bool LiftDragQuadratic::GetParam(const std::string &_tag, double &_output)
{
  _output = 0.0;
  if (_tag == "drag_constant")       _output = this->dragConstant;
  else if (_tag == "lift_constant")  _output = this->liftConstant;
  else                               return false;

  gzmsg << "LiftDragQuadratic::GetParam <" << _tag << ">=" << _output << "\n";
  return true;
}

std::map<std::string, double> LiftDragQuadratic::GetListParams()
{
  return {{"drag_constant", this->dragConstant},
          {"lift_constant", this->liftConstant}};
}

// ---------------------------------------------------------------------------
// LiftDragTwoLines
// ---------------------------------------------------------------------------
const std::string LiftDragTwoLines::IDENTIFIER = "TwoLines";
REGISTER_LIFTDRAG_CREATOR(LiftDragTwoLines, &LiftDragTwoLines::create)

LiftDrag* LiftDragTwoLines::create(sdf::ElementPtr _sdf)
{
  if (LiftDrag::CheckForElement(_sdf, "area")         &&
      LiftDrag::CheckForElement(_sdf, "fluid_density") &&
      LiftDrag::CheckForElement(_sdf, "a0")            &&
      LiftDrag::CheckForElement(_sdf, "alpha_stall")   &&
      LiftDrag::CheckForElement(_sdf, "cla")           &&
      LiftDrag::CheckForElement(_sdf, "cla_stall")     &&
      LiftDrag::CheckForElement(_sdf, "cda")           &&
      LiftDrag::CheckForElement(_sdf, "cda_stall"))
  {
    return new LiftDragTwoLines(
        _sdf->Get<double>("area"),
        _sdf->Get<double>("fluid_density"),
        _sdf->Get<double>("a0"),
        _sdf->Get<double>("alpha_stall"),
        _sdf->Get<double>("cla"),
        _sdf->Get<double>("cla_stall"),
        _sdf->Get<double>("cda"),
        _sdf->Get<double>("cda_stall"));
  }
  return nullptr;
}

gz::math::Vector3d LiftDragTwoLines::compute(
    const gz::math::Vector3d &_velL)
{
  gz::math::Vector3d velL = _velL;
  double angle = atan2(_velL.Y(), _velL.X());

  if (angle > GZ_PI_2)       { angle -= GZ_PI; velL = -_velL; }
  else if (angle < -GZ_PI_2) { angle += GZ_PI; velL = -_velL; }

  double alpha = angle + this->a0;
  while (fabs(alpha) > 0.5 * GZ_PI)
    alpha = alpha > 0 ? alpha - GZ_PI : alpha + GZ_PI;

  double u = velL.Length();
  double q = 0.5 * this->fluidDensity * u * u;

  double cl, cd;
  if (alpha > this->alphaStall)
  {
    double d = alpha - this->alphaStall;
    cl = this->cla * this->alphaStall + this->claStall * d;
    cd = this->cda * this->alphaStall + this->cdaStall * d;
  }
  else if (alpha < -this->alphaStall)
  {
    double s = alpha + this->alphaStall;
    cl = -this->cla * this->alphaStall + this->cdaStall * s;
    cd = -this->cda * this->alphaStall + this->cdaStall * s;
  }
  else
  {
    cd = this->cda * alpha;
    cl = this->cla * alpha;
  }

  double lift = cl * q * this->area;
  double drag = cd * q * this->area;

  gz::math::Vector3d liftDir =
      -(gz::math::Vector3d::UnitZ.Cross(_velL)).Normalize();
  gz::math::Vector3d dragDir = -_velL;

  return lift * liftDir + drag * dragDir.Normalize();
}

bool LiftDragTwoLines::GetParam(const std::string &_tag, double &_output)
{
  _output = 0.0;
  if      (_tag == "area")          _output = this->area;
  else if (_tag == "fluid_density") _output = this->fluidDensity;
  else if (_tag == "a0")            _output = this->a0;
  else if (_tag == "alpha_stall")   _output = this->alphaStall;
  else if (_tag == "cla")           _output = this->cla;
  else if (_tag == "cla_stall")     _output = this->claStall;
  else if (_tag == "cda")           _output = this->cda;
  else if (_tag == "cda_stall")     _output = this->cdaStall;
  else                              return false;

  gzmsg << "LiftDragTwoLines::GetParam <" << _tag << ">=" << _output << "\n";
  return true;
}

std::map<std::string, double> LiftDragTwoLines::GetListParams()
{
  return {{"area",         this->area},
          {"fluid_density",this->fluidDensity},
          {"a0",           this->a0},
          {"alpha_stall",  this->alphaStall},
          {"cla",          this->cla},
          {"cla_stall",    this->claStall},
          {"cda",          this->cda},
          {"cda_stall",    this->cdaStall}};
}

}  // namespace uuv_gz_plugins
