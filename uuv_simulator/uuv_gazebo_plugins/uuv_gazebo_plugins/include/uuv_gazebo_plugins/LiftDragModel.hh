#ifndef UUV_GZ_LIFTDRAG_MODEL_HH_
#define UUV_GZ_LIFTDRAG_MODEL_HH_

#include <string>
#include <map>
#include <gz/math/Vector3.hh>

namespace uuv_gz_plugins
{

class LiftDrag
{
protected:
  LiftDrag() = default;

public:
  virtual ~LiftDrag() = default;

  virtual std::string GetType() = 0;

  virtual gz::math::Vector3d Compute(const gz::math::Vector3d &_vel) = 0;

  virtual bool GetParam(const std::string &_tag, double &_out) = 0;

  virtual std::map<std::string, double> GetListParams() = 0;
};

} // namespace uuv_gz_plugins

#endif
