#ifndef UUV_GZ_UNDERWATER_OBJECT_PLUGIN_HH_
#define UUV_GZ_UNDERWATER_OBJECT_PLUGIN_HH_

#include <map>
#include <string>
#include <memory>

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/World.hh>
#include <gz/sim/EntityComponentManager.hh>

#include <gz/math/Vector3.hh>

#include <gz/msgs/wrench.pb.h>

#include "HydrodynamicModel.hh"
#include "Def.hh"

namespace uuv_gz_plugins
{

class UnderwaterObjectPlugin :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemPostUpdate
{
public:
  UnderwaterObjectPlugin() = default;
  virtual ~UnderwaterObjectPlugin() = default;

  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &) override;

  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override;

  void PostUpdate(const gz::sim::UpdateInfo &_info,
                  const gz::sim::EntityComponentManager &_ecm) override;

protected:
  void Connect();

  void UpdateFlowVelocity(const gz::math::Vector3d &_msg);

  void PublishCurrentVelocityMarker();
  void PublishIsSubmerged();

  void PublishRestoringForce(gz::sim::Entity _link);
  void PublishHydrodynamicWrenches(gz::sim::Entity _link);

  void GenWrenchMsg(const gz::math::Vector3d &_force,
                    const gz::math::Vector3d &_torque,
                    gz::msgs::WrenchStamped &_out);

  void InitDebug(gz::sim::Entity _link,
                 std::shared_ptr<HydrodynamicModel> _hydro);

protected:
  std::map<gz::sim::Entity,
           std::shared_ptr<HydrodynamicModel>> models;

  gz::math::Vector3d flowVelocity;

  gz::sim::Entity modelEntity;
  gz::sim::Entity worldEntity;

  std::string baseLinkName;

  bool useGlobalCurrent{true};
};

} // namespace uuv_gz_plugins

#endif
