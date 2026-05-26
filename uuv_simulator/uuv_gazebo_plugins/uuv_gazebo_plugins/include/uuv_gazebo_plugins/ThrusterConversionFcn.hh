#ifndef UUV_GZ_THRUSTER_CONVERSION_FCN_HH_
#define UUV_GZ_THRUSTER_CONVERSION_FCN_HH_

#include <string>
#include <map>
#include <vector>
#include <memory>

namespace uuv_gz_plugins
{

class ConversionFunction
{
public:
  virtual ~ConversionFunction() = default;

  virtual std::string GetType() = 0;

  virtual bool GetParam(const std::string &_tag,
                        double &_out) = 0;

  virtual std::map<double, double> GetTable()
  {
    return {};
  }

  virtual double Convert(double _cmd) = 0;
};

class ConversionFunctionBasic : public ConversionFunction
{
public:
  ConversionFunctionBasic(double _k)
  : rotorConstant(_k) {}

  std::string GetType() override { return "basic"; }

  bool GetParam(const std::string &_tag, double &_out) override
  {
    if (_tag == "rotor_constant")
    {
      _out = rotorConstant;
      return true;
    }
    return false;
  }

  double Convert(double _cmd) override
  {
    return rotorConstant * _cmd * std::abs(_cmd);
  }

protected:
  double rotorConstant;
};

class ConversionFunctionBessa : public ConversionFunction
{
public:
  ConversionFunctionBessa(double _l, double _r,
                          double _dl, double _dr)
  : rotorConstantL(_l), rotorConstantR(_r),
    deltaL(_dl), deltaR(_dr) {}

  std::string GetType() override { return "bessa"; }

  bool GetParam(const std::string &_tag, double &_out) override
  {
    return false;
  }

  double Convert(double _cmd) override
  {
    if (_cmd < 0)
      return rotorConstantL * (_cmd + deltaL);
    return rotorConstantR * (_cmd - deltaR);
  }

protected:
  double rotorConstantL;
  double rotorConstantR;
  double deltaL;
  double deltaR;
};

} // namespace uuv_gz_plugins

#endif
