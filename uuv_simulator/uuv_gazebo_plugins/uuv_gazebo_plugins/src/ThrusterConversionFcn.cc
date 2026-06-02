// Copyright (c) 2016 The UUV Simulator Authors.
// All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License")

// ============================================================
// ROS2 / Gazebo Harmonic (gz-sim 8) conversion notes:
//
//  - No ignition:: types or gazebo physics types used here.
//  - #include <gazebo/...> removed; only sdf + std headers needed.
//  - GZ_ASSERT still valid (gz/common/AssertionComparator.hh via gz-common).
//  - gzmsg kept via gz/common/Console.hh.
// ============================================================

#include <gz/common/Console.hh>
#include <uuv_gazebo_plugins/ThrusterConversionFcn.hh>
#include <uuv_gazebo_plugins/Def.hh>

namespace uuv_gz_plugins
{

std::map<double, double> ConversionFunction::GetTable()
{
  return {};
}

/////////////////////////////////////////////////
ConversionFunction* ConversionFunctionFactory::CreateConversionFunction(
    sdf::ElementPtr _sdf)
{
  if (!_sdf->HasElement("type"))
  {
    std::cerr << "conversion does not have a type element\n";
    return nullptr;
  }

  std::string identifier = _sdf->Get<std::string>("type");

  if (creators_.find(identifier) == creators_.end())
  {
    std::cerr << "Cannot create ConversionFunction with unknown identifier: "
              << identifier << "\n";
    return nullptr;
  }

  return creators_[identifier](_sdf);
}

/////////////////////////////////////////////////
ConversionFunctionFactory& ConversionFunctionFactory::GetInstance()
{
  static ConversionFunctionFactory instance;
  return instance;
}

/////////////////////////////////////////////////
bool ConversionFunctionFactory::RegisterCreator(
    const std::string        &_identifier,
    ConversionFunctionCreator _creator)
{
  if (creators_.find(_identifier) != creators_.end())
  {
    std::cerr << "Warning: Registering ConversionFunction with identifier: "
              << _identifier << " twice\n";
  }
  creators_[_identifier] = _creator;
  std::cout << "Registered ConversionFunction type " << _identifier << "\n";
  return true;
}

// ---------------------------------------------------------------------------
// Basic
// ---------------------------------------------------------------------------
const std::string ConversionFunctionBasic::IDENTIFIER = "Basic";
REGISTER_CONVERSIONFUNCTION_CREATOR(ConversionFunctionBasic,
                                    &ConversionFunctionBasic::create)

ConversionFunction* ConversionFunctionBasic::create(sdf::ElementPtr _sdf)
{
  if (!_sdf->HasElement("rotorConstant"))
  {
    std::cerr << "ConversionFunctionBasic: expected element rotorConstant\n";
    return nullptr;
  }
  return new ConversionFunctionBasic(_sdf->Get<double>("rotorConstant"));
}

double ConversionFunctionBasic::convert(double _cmd)
{
  return this->rotorConstant * std::abs(_cmd) * _cmd;
}

bool ConversionFunctionBasic::GetParam(const std::string &_tag, double &_output)
{
  _output = 0.0;
  if (_tag != "rotor_constant") return false;
  _output = this->rotorConstant;
  gzmsg << "ConversionFunctionBasic::GetParam <" << _tag << ">="
        << _output << "\n";
  return true;
}

ConversionFunctionBasic::ConversionFunctionBasic(double _rotorConstant)
  : rotorConstant(_rotorConstant)
{
  gzmsg << "ConversionFunctionBasic: rotorConstant=" << rotorConstant << "\n";
}

// ---------------------------------------------------------------------------
// Bessa
// ---------------------------------------------------------------------------
const std::string ConversionFunctionBessa::IDENTIFIER = "Bessa";
REGISTER_CONVERSIONFUNCTION_CREATOR(ConversionFunctionBessa,
                                    &ConversionFunctionBessa::create)

ConversionFunction* ConversionFunctionBessa::create(sdf::ElementPtr _sdf)
{
  for (const char *tag :
       {"rotorConstantL", "rotorConstantR", "deltaL", "deltaR"})
  {
    if (!_sdf->HasElement(tag))
    {
      std::cerr << "ConversionFunctionBessa: expected element " << tag << "\n";
      return nullptr;
    }
  }
  return new ConversionFunctionBessa(
      _sdf->Get<double>("rotorConstantL"),
      _sdf->Get<double>("rotorConstantR"),
      _sdf->Get<double>("deltaL"),
      _sdf->Get<double>("deltaR"));
}

double ConversionFunctionBessa::convert(double _cmd)
{
  double basic = _cmd * std::abs(_cmd);
  if      (basic <= this->deltaL) return this->rotorConstantL * (basic - this->deltaL);
  else if (basic >= this->deltaR) return this->rotorConstantR * (basic - this->deltaR);
  return 0.0;
}

ConversionFunctionBessa::ConversionFunctionBessa(
    double _rotorConstantL, double _rotorConstantR,
    double _deltaL,          double _deltaR)
  : rotorConstantL(_rotorConstantL), rotorConstantR(_rotorConstantR),
    deltaL(_deltaL), deltaR(_deltaR)
{
  GZ_ASSERT(rotorConstantL >= 0.0,
            "ConversionFunctionBessa: rotorConstantL should be >= 0");
  GZ_ASSERT(rotorConstantR >= 0.0,
            "ConversionFunctionBessa: rotorConstantR should be >= 0");
  GZ_ASSERT(deltaL <= 0.0,
            "ConversionFunctionBessa: deltaL should be <= 0");
  GZ_ASSERT(deltaR >= 0.0,
            "ConversionFunctionBessa: deltaR should be >= 0");

  gzmsg << "ConversionFunctionBessa:\n"
        << "\t- rotorConstantL: " << rotorConstantL << "\n"
        << "\t- rotorConstantR: " << rotorConstantR << "\n"
        << "\t- deltaL: "         << deltaL          << "\n"
        << "\t- deltaR: "         << deltaR          << "\n";
}

bool ConversionFunctionBessa::GetParam(const std::string &_tag, double &_output)
{
  _output = 0.0;
  if      (_tag == "rotor_constant_l") _output = this->rotorConstantL;
  else if (_tag == "rotor_constant_r") _output = this->rotorConstantR;
  else if (_tag == "delta_l")          _output = this->deltaL;
  else if (_tag == "delta_r")          _output = this->deltaR;
  else                                 return false;

  gzmsg << "ConversionFunctionBessa::GetParam <" << _tag << ">="
        << _output << "\n";
  return true;
}

// ---------------------------------------------------------------------------
// LinearInterp
// ---------------------------------------------------------------------------
const std::string ConversionFunctionLinearInterp::IDENTIFIER = "LinearInterp";
REGISTER_CONVERSIONFUNCTION_CREATOR(ConversionFunctionLinearInterp,
                                    &ConversionFunctionLinearInterp::create)

ConversionFunction* ConversionFunctionLinearInterp::create(
    sdf::ElementPtr _sdf)
{
  if (!_sdf->HasElement("inputValues"))
  {
    std::cerr << "ConversionFunctionLinearInterp: expected inputValues\n";
    return nullptr;
  }
  if (!_sdf->HasElement("outputValues"))
  {
    std::cerr << "ConversionFunctionLinearInterp: expected outputValues\n";
    return nullptr;
  }

  std::vector<double> in  = Str2Vector(_sdf->Get<std::string>("inputValues"));
  std::vector<double> out = Str2Vector(_sdf->Get<std::string>("outputValues"));

  if (in.empty())
  {
    std::cerr << "ConversionFunctionLinearInterp: need at least one pair\n";
    return nullptr;
  }
  if (in.size() != out.size())
  {
    std::cerr << "ConversionFunctionLinearInterp: input/output size mismatch\n";
    return nullptr;
  }

  return new ConversionFunctionLinearInterp(in, out);
}

double ConversionFunctionLinearInterp::convert(double _cmd)
{
  GZ_ASSERT(!lookupTable.empty(), "Lookup table is empty");

  auto iter = lookupTable.lower_bound(_cmd);
  if (iter == lookupTable.end())
    return lookupTable.rbegin()->second;

  double i1 = iter->first;
  double o1 = iter->second;
  if (iter == lookupTable.begin()) return o1;

  --iter;
  double i0 = iter->first;
  double o0 = iter->second;

  double w1 = _cmd - i0;
  double w0 = i1   - _cmd;
  return (o0 * w0 + o1 * w1) / (w0 + w1);
}

ConversionFunctionLinearInterp::ConversionFunctionLinearInterp(
    const std::vector<double> &_input,
    const std::vector<double> &_output)
{
  GZ_ASSERT(_input.size() == _output.size(), "input/output size mismatch");
  for (std::size_t i = 0; i < _input.size(); ++i)
    lookupTable[_input[i]] = _output[i];

  gzmsg << "ConversionFunctionLinearInterp created\n\t- Input:  ";
  for (auto &kv : lookupTable) std::cout << kv.first  << " ";
  std::cout << "\n\t- Output: ";
  for (auto &kv : lookupTable) std::cout << kv.second << " ";
  std::cout << "\n";
}

bool ConversionFunctionLinearInterp::GetParam(const std::string &/*_tag*/,
                                              double      &/*_output*/)
{
  return false;
}

std::map<double, double> ConversionFunctionLinearInterp::GetTable()
{
  return this->lookupTable;
}

}  // namespace uuv_gz_plugins
