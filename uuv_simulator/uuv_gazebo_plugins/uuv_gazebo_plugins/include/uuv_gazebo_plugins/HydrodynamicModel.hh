#ifndef UUV_GZ_HYDRODYNAMIC_MODEL_HH_
#define UUV_GZ_HYDRODYNAMIC_MODEL_HH_

#include <string>
#include <vector>
#include <map>
#include <memory>

#include <gz/math/Vector3.hh>
#include <gz/math/Matrix6.hh>

#include <gz/sim/Entity.hh>

#include "BuoyantObject.hh"

namespace uuv_gz_plugins
{

class HydrodynamicModel : public BuoyantObject
{
public:
  HydrodynamicModel(gz::sim::Entity _link)
  : BuoyantObject(_link)
  {}

  virtual ~HydrodynamicModel() = default;

  virtual std::string GetType() = 0;

  virtual void ApplyHydrodynamicForces(
      double _time,
      const gz::math::Vector3d &_flowVel) = 0;

  virtual void Print(const std::string &_name,
                     const std::string &_msg = "") = 0;

  virtual bool GetParam(const std::string &_tag,
                        std::vector<double> &_out) = 0;

  virtual bool GetParam(const std::string &_tag,
                        double &_out) = 0;

  virtual bool SetParam(const std::string &_tag,
                        double _input) = 0;

protected:
  double lastTime{0.0};

  gz::math::Vector6d lastVelRel;

  std::vector<std::string> params;
};

} // namespace uuv_gz_plugins

#endif
