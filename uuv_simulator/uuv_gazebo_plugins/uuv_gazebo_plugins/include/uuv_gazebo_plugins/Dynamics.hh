#ifndef UUV_GZ_DYNAMICS_HH_
#define UUV_GZ_DYNAMICS_HH_

#include <string>
#include <map>

#include <sdf/sdf.hh>

namespace uuv_gz_plugins
{

class Dynamics
{
protected:
  Dynamics();

public:
  virtual ~Dynamics() = default;

  virtual std::string GetType() = 0;
  virtual double update(double _cmd, double _t) = 0;

  virtual void Reset();

protected:
  double prevTime{0};
  double state{0};
};

using DynamicsCreator = Dynamics* (*)(sdf::ElementPtr);

class DynamicsFactory
{
public:
  Dynamics* CreateDynamics(sdf::ElementPtr _sdf);

  static DynamicsFactory& GetInstance();

  bool RegisterCreator(const std::string &_identifier,
                       DynamicsCreator _creator);

private:
  std::map<std::string, DynamicsCreator> creators_;
};

#define REGISTER_DYNAMICS_CREATOR(classname, creator) \
  static bool classname##_registered = \
    DynamicsFactory::GetInstance().RegisterCreator(classname::IDENTIFIER, creator);

class DynamicsZeroOrder : public Dynamics
{
public:
  static const std::string IDENTIFIER;

  std::string GetType() override { return IDENTIFIER; }
  double update(double _cmd, double _t) override;

  static Dynamics* create(sdf::ElementPtr _sdf);
};

class DynamicsFirstOrder : public Dynamics
{
public:
  explicit DynamicsFirstOrder(double _tau);

  static const std::string IDENTIFIER;

  std::string GetType() override { return IDENTIFIER; }
  double update(double _cmd, double _t) override;

  static Dynamics* create(sdf::ElementPtr _sdf);

private:
  double tau{0.0};
};

class ThrusterDynamicsYoerger : public Dynamics
{
public:
  ThrusterDynamicsYoerger(double _alpha, double _beta);

  static const std::string IDENTIFIER;

  std::string GetType() override { return IDENTIFIER; }
  double update(double _cmd, double _t) override;

  static Dynamics* create(sdf::ElementPtr _sdf);

private:
  double alpha{0.0};
  double beta{0.0};
};

class ThrusterDynamicsBessa : public Dynamics
{
public:
  ThrusterDynamicsBessa(double _Jmsp, double _Kv1, double _Kv2,
                        double _Kt, double _Rm);

  static const std::string IDENTIFIER;

  std::string GetType() override { return IDENTIFIER; }
  double update(double _cmd, double _t) override;

  static Dynamics* create(sdf::ElementPtr _sdf);

private:
  double Jmsp{0.0};
  double Kv1{0.0};
  double Kv2{0.0};
  double Kt{0.0};
  double Rm{0.0};
};

} // namespace uuv_gz_plugins

#endif
