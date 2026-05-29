// Ported to ROS 2 / Gazebo Harmonic (gz-sim 8)

#ifndef UUV_SENSOR_PLUGINS_COMMON_H_
#define UUV_SENSOR_PLUGINS_COMMON_H_

#include <string>
#include <Eigen/Dense>
#include <gz/sim/System.hh>
#include <sdf/sdf.hh>

namespace gz {
namespace sim {

template<class T>
bool GetSDFParam(sdf::ElementPtr sdf, const std::string& name, T& param,
                 const T& default_value, const bool& verbose = false)
{
  if (sdf->HasElement(name)) {
    param = sdf->GetElement(name)->Get<T>();
    return true;
  } else {
    param = default_value;
    if (verbose)
      gzerr << "[uuv_sensor_plugins] Please specify a value for parameter \""
            << name << "\".\n";
  }
  return false;
}

}  // namespace sim
}  // namespace gz

template <typename T>
class FirstOrderFilter {
public:
  FirstOrderFilter(double timeConstantUp, double timeConstantDown, T initialState)
    : timeConstantUp_(timeConstantUp), timeConstantDown_(timeConstantDown),
      previousState_(initialState) {}

  T updateFilter(T inputState, double samplingTime) {
    T outputState;
    if (inputState > previousState_) {
      double alphaUp = std::exp(-samplingTime / timeConstantUp_);
      outputState = alphaUp * previousState_ + (1.0 - alphaUp) * inputState;
    } else {
      double alphaDown = std::exp(-samplingTime / timeConstantDown_);
      outputState = alphaDown * previousState_ + (1.0 - alphaDown) * inputState;
    }
    previousState_ = outputState;
    return outputState;
  }
  ~FirstOrderFilter() {}

protected:
  double timeConstantUp_, timeConstantDown_;
  T previousState_;
};

template<class Derived>
Eigen::Quaternion<typename Derived::Scalar> QuaternionFromSmallAngle(
    const Eigen::MatrixBase<Derived>& theta)
{
  typedef typename Derived::Scalar Scalar;
  EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
  const Scalar q_squared = theta.squaredNorm() / 4.0;
  if (q_squared < 1)
    return Eigen::Quaternion<Scalar>(std::sqrt(1 - q_squared),
      theta[0]*0.5, theta[1]*0.5, theta[2]*0.5);
  else {
    const Scalar w = 1.0 / std::sqrt(1 + q_squared), f = w * 0.5;
    return Eigen::Quaternion<Scalar>(w, theta[0]*f, theta[1]*f, theta[2]*f);
  }
}

template<class In, class Out>
void copyPosition(const In& in, Out* out) { out->x=in.x; out->y=in.y; out->z=in.z; }

#endif  // UUV_SENSOR_PLUGINS_COMMON_H_
