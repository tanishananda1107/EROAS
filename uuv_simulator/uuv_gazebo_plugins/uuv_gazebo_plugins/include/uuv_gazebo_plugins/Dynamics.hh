#ifndef UUV_GZ_DYNAMICS_HH_
#define UUV_GZ_DYNAMICS_HH_

#include <string>
#include <map>

namespace uuv_gz_plugins
{

class Dynamics
{
protected:
  Dynamics() { this->Reset(); }

public:
  virtual ~Dynamics() = default;

  virtual std::string GetType() = 0;
  virtual double Update(double _cmd, double _t) = 0;

  virtual void Reset()
  {
    this->prevTime = 0;
    this->state = 0;
  }

protected:
  double prevTime{0};
  double state{0};
};

} // namespace uuv_gz_plugins

#endif
