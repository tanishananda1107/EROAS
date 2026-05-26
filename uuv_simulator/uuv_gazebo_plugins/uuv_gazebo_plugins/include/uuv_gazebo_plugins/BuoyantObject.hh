#ifndef UUV_GZ_BUOYANT_OBJECT_HH_
#define UUV_GZ_BUOYANT_OBJECT_HH_

#include <string>
#include <map>

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>

#include <gz/math/Vector3.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/AxisAlignedBox.hh>

namespace uuv_gz_plugins
{

class BuoyantObject
{
public:
  explicit BuoyantObject(gz::sim::Entity _linkEntity)
  : linkEntity(_linkEntity)
  {
  }

  virtual ~BuoyantObject() = default;

  void SetBoundingBox(const gz::math::AxisAlignedBox &_bBox)
  {
    this->boundingBox = _bBox;
  }

  void GetBuoyancyForce(const gz::math::Pose3d &_pose,
                        gz::math::Vector3d &force,
                        gz::math::Vector3d &torque);

  void ApplyBuoyancyForce();

  void SetVolume(double _volume = -1);
  double GetVolume() const;

  void SetFluidDensity(double _rho);
  double GetFluidDensity() const;

  void SetCoB(const gz::math::Vector3d &_cob);
  gz::math::Vector3d GetCoB() const;

  void SetGravity(double _g);
  double GetGravity() const;

  void SetNeutrallyBuoyant();
  bool IsNeutrallyBuoyant() const;

  bool IsSubmerged() const;

protected:
  gz::math::AxisAlignedBox boundingBox;

  double volume{0};
  double fluidDensity{1000.0};
  double g{9.81};

  gz::math::Vector3d centerOfBuoyancy{0,0,0};

  std::map<std::string, gz::math::Vector3d> hydroWrench;

  bool debugFlag{false};
  bool isSubmerged{false};
  bool neutrallyBuoyant{false};

  gz::sim::Entity linkEntity;
};

} // namespace uuv_gz_plugins

#endif
