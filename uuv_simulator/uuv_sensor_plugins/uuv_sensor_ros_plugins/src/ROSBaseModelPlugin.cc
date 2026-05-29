// Ported to ROS 2 / Gazebo Harmonic (gz-sim 8)
#include <uuv_sensor_ros_plugins/ROSBaseModelPlugin.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/WorldPose.hh>
#include <gz/sim/components/Link.hh>

namespace gz { namespace sim {

ROSBaseModelPlugin::ROSBaseModelPlugin()
{
  this->localNEDFrame = math::Pose3d::Zero;
  this->localNEDFrame.Rot() = math::Quaterniond(math::Vector3d(M_PI, 0, 0));

  this->tfLocalNEDFrame.transform.translation.x = 0;
  this->tfLocalNEDFrame.transform.translation.y = 0;
  this->tfLocalNEDFrame.transform.translation.z = 0;
  // Roll PI rotation (ENU -> NED)
  tf2::Quaternion q;
  q.setRPY(M_PI, 0.0, 0.0);
  this->tfLocalNEDFrame.transform.rotation.x = q.x();
  this->tfLocalNEDFrame.transform.rotation.y = q.y();
  this->tfLocalNEDFrame.transform.rotation.z = q.z();
  this->tfLocalNEDFrame.transform.rotation.w = q.w();
}

ROSBaseModelPlugin::~ROSBaseModelPlugin() {}

void ROSBaseModelPlugin::Configure(
  const Entity& _entity,
  const std::shared_ptr<const sdf::Element>& _sdf,
  EntityComponentManager& _ecm,
  EventManager& _eventMgr)
{
  this->modelEntity = _entity;
  this->model = Model(_entity);
  this->eventMgr_ = &_eventMgr;

  auto sdfPtr = std::const_pointer_cast<sdf::Element>(_sdf);

  std::string linkName;
  GZ_ASSERT(sdfPtr->HasElement("link_name"), "No link_name provided in SDF");
  GetSDFParam<std::string>(sdfPtr, "link_name", linkName, "");
  GZ_ASSERT(!linkName.empty(), "link_name is empty");

  GetSDFParam<bool>(sdfPtr, "enable_local_ned_frame",
    this->enableLocalNEDFrame, true);

  // Resolve link entity
  this->linkEntity = this->model.LinkByName(_ecm, linkName);
  GZ_ASSERT(this->linkEntity != kNullEntity, "Link not found");
  this->link = Link(this->linkEntity);

  // Optional reference link
  if (sdfPtr->HasElement("reference_link_name"))
  {
    std::string refLinkName;
    GetSDFParam<std::string>(sdfPtr, "reference_link_name", refLinkName, "");
    if (!refLinkName.empty())
    {
      this->referenceLink = this->model.LinkByName(_ecm, refLinkName);
      GZ_ASSERT(this->referenceLink != kNullEntity, "Reference link not found");
      this->referenceFrameID = refLinkName;
    }
  }

  // Set NED frame header IDs
  this->tfLocalNEDFrame.header.frame_id = linkName;
  this->tfLocalNEDFrame.child_frame_id  = linkName + "_ned";

  this->InitBasePlugin(sdfPtr);

  // TF broadcaster
  this->tfBroadcaster =
    std::make_shared<tf2_ros::TransformBroadcaster>(this->rosNode);
}

void ROSBaseModelPlugin::Update(
  const UpdateInfo& _info,
  EntityComponentManager& _ecm)
{
  this->OnUpdate(_info, _ecm);
}

bool ROSBaseModelPlugin::OnUpdate(
  const UpdateInfo& /*_info*/,
  EntityComponentManager& /*_ecm*/)
{
  return true;
}

void ROSBaseModelPlugin::SendLocalNEDTransform()
{
  this->tfLocalNEDFrame.header.stamp = this->rosNode->now();
  this->tfBroadcaster->sendTransform(this->tfLocalNEDFrame);
}

}} // namespace gz::sim
