#ifndef UUV_GZ_LIFTDRAG_MODEL_HH_
#define UUV_GZ_LIFTDRAG_MODEL_HH_

#include <string>
#include <map>
#include <gz/math/Vector3.hh>
#include <sdf/sdf.hh>

namespace uuv_gz_plugins
{

class LiftDrag
{
protected:
  LiftDrag() = default;

public:
  virtual ~LiftDrag() = default;

  virtual std::string GetType() = 0;

  virtual gz::math::Vector3d compute(const gz::math::Vector3d &_vel) = 0;

  virtual bool GetParam(const std::string &_tag, double &_out) = 0;

  virtual std::map<std::string, double> GetListParams() = 0;

  static bool CheckForElement(sdf::ElementPtr _sdf,
                              const std::string &element);
};

using LiftDragCreator = LiftDrag* (*)(sdf::ElementPtr);

class LiftDragFactory
{
public:
  LiftDrag* CreateLiftDrag(sdf::ElementPtr _sdf);

  static LiftDragFactory& GetInstance();

  bool RegisterCreator(const std::string &_identifier,
                       LiftDragCreator _creator);

private:
  std::map<std::string, LiftDragCreator> creators_;
};

#define REGISTER_LIFTDRAG_CREATOR(classname, creator) \
  static bool classname##_registered = \
    LiftDragFactory::GetInstance().RegisterCreator(classname::IDENTIFIER, creator);

class LiftDragQuadratic : public LiftDrag
{
public:
  LiftDragQuadratic(double _liftConstant, double _dragConstant)
  : liftConstant(_liftConstant), dragConstant(_dragConstant) {}

  static const std::string IDENTIFIER;

  std::string GetType() override { return IDENTIFIER; }
  gz::math::Vector3d compute(const gz::math::Vector3d &_vel) override;
  bool GetParam(const std::string &_tag, double &_out) override;
  std::map<std::string, double> GetListParams() override;

  static LiftDrag* create(sdf::ElementPtr _sdf);

private:
  double liftConstant{0.0};
  double dragConstant{0.0};
};

class LiftDragTwoLines : public LiftDrag
{
public:
  LiftDragTwoLines(double _area, double _fluidDensity, double _a0,
                   double _alphaStall, double _cla, double _claStall,
                   double _cda, double _cdaStall)
  : area(_area), fluidDensity(_fluidDensity), a0(_a0),
    alphaStall(_alphaStall), cla(_cla), claStall(_claStall),
    cda(_cda), cdaStall(_cdaStall) {}

  static const std::string IDENTIFIER;

  std::string GetType() override { return IDENTIFIER; }
  gz::math::Vector3d compute(const gz::math::Vector3d &_vel) override;
  bool GetParam(const std::string &_tag, double &_out) override;
  std::map<std::string, double> GetListParams() override;

  static LiftDrag* create(sdf::ElementPtr _sdf);

private:
  double area{0.0};
  double fluidDensity{0.0};
  double a0{0.0};
  double alphaStall{0.0};
  double cla{0.0};
  double claStall{0.0};
  double cda{0.0};
  double cdaStall{0.0};
};

} // namespace uuv_gz_plugins

#endif
