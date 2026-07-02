#ifndef UUV_GZ_UMBILICAL_PLUGIN_HH_
#define UUV_GZ_UMBILICAL_PLUGIN_HH_

#include <memory>
#include <string>

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/World.hh>
#include <gz/sim/Entity.hh>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

#include "UmbilicalModel.hh"

namespace uuv_gz_plugins
{

class UmbilicalSegment
{
public:
  UmbilicalSegment() = default;

  UmbilicalSegment(const std::string &_name,
                   const std::string &_fromLink,
                   const gz::math::Pose3d &_fromPose,
                   const gz::math::Pose3d &_toPose,
                   gz::sim::Entity _modelEntity)
  : modelEntity(_modelEntity)
  {}

  void InitSdfSegment();

  gz::sim::Entity link;
  gz::sim::Entity linkA;

  gz::sim::Entity jointA;
  gz::sim::Entity jointB;

  std::shared_ptr<UmbilicalSegment> prev;
  std::shared_ptr<UmbilicalSegment> next;

  static std::shared_ptr<void> sdfSegment;
};

using UmbilicalSegmentPtr = std::shared_ptr<UmbilicalSegment>;

class UmbilicalPlugin :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  UmbilicalPlugin() = default;
  virtual ~UmbilicalPlugin() = default;

  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &) override;

  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override;

protected:
  void UpdateFlowVelocity(const gz::math::Vector3d &_msg);

protected:
  gz::sim::Entity modelEntity;
  gz::sim::Entity worldEntity;

  gz::math::Vector3d flowVelocity;

  std::shared_ptr<UmbilicalModel> umbilical;
};

} // namespace uuv_gz_plugins

#endif
