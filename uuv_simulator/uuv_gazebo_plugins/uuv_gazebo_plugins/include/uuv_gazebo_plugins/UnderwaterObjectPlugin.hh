#ifndef UUV_GZ_UNDERWATER_OBJECT_PLUGIN_HH_
#define UUV_GZ_UNDERWATER_OBJECT_PLUGIN_HH_

#include <memory>
#include <string>
#include <vector>

#include <gz/math/Vector3.hh>
#include <gz/msgs/vector3d.pb.h>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

namespace uuv_gz_plugins
{

class UnderwaterObjectPlugin :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  struct LinkModel
  {
    gz::sim::Entity entity{gz::sim::kNullEntity};
    std::string name;
    double volume{0.0};
    double fluidDensity{1028.0};
    double linearDamping{0.0};
    double angularDamping{0.0};
    gz::math::Vector3d centerOfBuoyancy{0, 0, 0};
    bool neutrallyBuoyant{false};
    bool submerged{true};
  };

  UnderwaterObjectPlugin() = default;
  ~UnderwaterObjectPlugin() override = default;

  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &) override;

  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override;

  void SetUseGlobalCurrent(bool _useGlobal);
  bool UseGlobalCurrent() const;

  void SetFlowVelocity(const gz::math::Vector3d &_flowVelocity);
  gz::math::Vector3d FlowVelocity() const;

  const std::vector<LinkModel> &Models() const;

protected:
  void OnFlowVelocity(const gz::msgs::Vector3d &_msg);
  double LinkMass(gz::sim::Entity _link,
                  const gz::sim::EntityComponentManager &_ecm) const;

protected:
  gz::sim::Model model;
  gz::sim::Entity modelEntity{gz::sim::kNullEntity};
  gz::sim::Entity worldEntity{gz::sim::kNullEntity};
  std::vector<LinkModel> models;

  gz::math::Vector3d flowVelocity{0, 0, 0};
  bool useGlobalCurrent{true};
  double gravity{9.81};

  gz::transport::Node node;
};

} // namespace uuv_gz_plugins

#endif
