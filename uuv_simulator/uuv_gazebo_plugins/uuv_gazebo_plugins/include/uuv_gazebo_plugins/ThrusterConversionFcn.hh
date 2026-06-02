#ifndef UUV_GZ_THRUSTER_CONVERSION_FCN_HH_
#define UUV_GZ_THRUSTER_CONVERSION_FCN_HH_

#include <string>
#include <map>
#include <vector>
#include <memory>

#include <sdf/sdf.hh>

namespace uuv_gz_plugins
{

class ConversionFunction
{
public:
  virtual ~ConversionFunction() = default;

  virtual std::string GetType() = 0;

  virtual bool GetParam(const std::string &_tag,
                        double &_out) = 0;

  virtual std::map<double, double> GetTable();

  virtual double convert(double _cmd) = 0;
};

using ConversionFunctionCreator = ConversionFunction* (*)(sdf::ElementPtr);

class ConversionFunctionFactory
{
public:
  ConversionFunction* CreateConversionFunction(sdf::ElementPtr _sdf);

  static ConversionFunctionFactory& GetInstance();

  bool RegisterCreator(const std::string &_identifier,
                       ConversionFunctionCreator _creator);

private:
  std::map<std::string, ConversionFunctionCreator> creators_;
};

#define REGISTER_CONVERSIONFUNCTION_CREATOR(classname, creator) \
  static bool classname##_registered = \
    ConversionFunctionFactory::GetInstance().RegisterCreator( \
      classname::IDENTIFIER, creator);

class ConversionFunctionBasic : public ConversionFunction
{
public:
  explicit ConversionFunctionBasic(double _rotorConstant);

  static const std::string IDENTIFIER;

  std::string GetType() override { return IDENTIFIER; }
  bool GetParam(const std::string &_tag, double &_out) override;
  double convert(double _cmd) override;

  static ConversionFunction* create(sdf::ElementPtr _sdf);

protected:
  double rotorConstant{0.0};
};

class ConversionFunctionBessa : public ConversionFunction
{
public:
  ConversionFunctionBessa(double _rotorConstantL, double _rotorConstantR,
                          double _deltaL, double _deltaR);

  static const std::string IDENTIFIER;

  std::string GetType() override { return IDENTIFIER; }
  bool GetParam(const std::string &_tag, double &_out) override;
  double convert(double _cmd) override;

  static ConversionFunction* create(sdf::ElementPtr _sdf);

protected:
  double rotorConstantL{0.0};
  double rotorConstantR{0.0};
  double deltaL{0.0};
  double deltaR{0.0};
};

class ConversionFunctionLinearInterp : public ConversionFunction
{
public:
  ConversionFunctionLinearInterp(const std::vector<double> &_input,
                                 const std::vector<double> &_output);

  static const std::string IDENTIFIER;

  std::string GetType() override { return IDENTIFIER; }
  bool GetParam(const std::string &_tag, double &_out) override;
  std::map<double, double> GetTable() override;
  double convert(double _cmd) override;

  static ConversionFunction* create(sdf::ElementPtr _sdf);

protected:
  std::map<double, double> lookupTable;
};

} // namespace uuv_gz_plugins

#endif
