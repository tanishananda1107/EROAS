#ifndef UUV_GZ_DEF_HH_
#define UUV_GZ_DEF_HH_

#include <vector>
#include <string>
#include <map>
#include <sstream>

#include <gz/math/Vector3.hh>
#include <gz/math/Matrix3.hh>

namespace uuv_gz_plugins
{

#define PI 3.14159265359

inline std::vector<double> Str2Vector(const std::string &_input)
{
  std::vector<double> output;
  std::stringstream ss(_input);
  std::string v;

  while (ss >> v)
    output.push_back(std::stod(v));

  return output;
}

inline gz::math::Vector3d Cross(const gz::math::Vector3d &_x)
{
  return gz::math::Vector3d(_x.Y(), -_x.X(), 0.0);
}

inline gz::math::Vector3d ToVec3(const gz::math::Vector3d &_x)
{
  return _x;
}

} // namespace uuv_gz_plugins

#endif
