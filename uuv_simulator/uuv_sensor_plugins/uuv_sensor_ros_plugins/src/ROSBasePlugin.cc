// Ported to ROS 2 / Gazebo Harmonic (gz-sim 8)
#include <uuv_sensor_ros_plugins/ROSBasePlugin.hh>
#include <gz/sim/Util.hh>

namespace gz { namespace sim {

ROSBasePlugin::ROSBasePlugin()
{
  this->gazeboMsgEnabled = true;
  this->referenceFrame = math::Pose3d::Zero;
  this->referenceFrameID = "world";
  this->isReferenceInit = false;
  this->isOn.data = true;
  this->referenceLink = kNullEntity;

  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  this->rndGen = std::default_random_engine(seed);
}

ROSBasePlugin::~ROSBasePlugin()
{
  if (this->rosNode)
    rclcpp::shutdown();
}

bool ROSBasePlugin::InitBasePlugin(sdf::ElementPtr _sdf)
{
  using namespace gz::sim;

  GetSDFParam<std::string>(_sdf, "robot_namespace", this->robotNamespace, "");
  GZ_ASSERT(!this->robotNamespace.empty(), "Robot namespace was not provided");

  std::string sensorTopic;
  GetSDFParam<std::string>(_sdf, "sensor_topic", sensorTopic, "");
  if (!sensorTopic.empty())
    this->sensorOutputTopic = sensorTopic;
  GZ_ASSERT(!this->sensorOutputTopic.empty(), "Sensor output topic not provided");

  GetSDFParam<double>(_sdf, "update_rate", this->updateRate, 30.0);
  GetSDFParam<bool>(_sdf, "enable_gazebo_messages", this->gazeboMsgEnabled, false);

  // Create ROS 2 node
  if (!rclcpp::ok())
  {
    gzerr << "ROS 2 not initialized — cannot load sensor plugin.\n";
    return false;
  }
  this->rosNode = std::make_shared<rclcpp::Node>(this->robotNamespace);

  // Reference frame
  if (_sdf->HasElement("static_reference_frame"))
  {
    GetSDFParam<std::string>(_sdf, "static_reference_frame",
      this->referenceFrameID, "world");
    this->referenceLink = kNullEntity;

    if (this->referenceFrameID != "world")
    {
      this->tfStaticSub =
        this->rosNode->create_subscription<tf2_msgs::msg::TFMessage>(
          "/tf_static", 1,
          [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
            this->GetTFMessage(msg);
          });
    }
    else
      this->isReferenceInit = true;
  }
  else
  {
    this->referenceFrameID = "world";
    this->referenceLink = kNullEntity;
    this->isReferenceInit = true;
  }

  // Sensor on/off
  bool isSensorOn;
  GetSDFParam<bool>(_sdf, "is_on", isSensorOn, true);
  this->isOn.data = isSensorOn;

  // ROS 2 service for toggling sensor state
  this->changeSensorSrv =
    this->rosNode->create_service<uuv_sensor_ros_plugins_msgs::srv::ChangeSensorState>(
      this->sensorOutputTopic + "/change_state",
      [this](
        const std::shared_ptr<uuv_sensor_ros_plugins_msgs::srv::ChangeSensorState::Request> req,
        std::shared_ptr<uuv_sensor_ros_plugins_msgs::srv::ChangeSensorState::Response> res)
      { this->ChangeSensorState(req, res); });

  this->pluginStatePub =
    this->rosNode->create_publisher<std_msgs::msg::Bool>(
      this->sensorOutputTopic + "/state", 1);

  GetSDFParam<double>(_sdf, "noise_sigma", this->noiseSigma, 0.0);
  GZ_ASSERT(this->noiseSigma >= 0.0, "Noise sigma must be >= 0");

  GetSDFParam<double>(_sdf, "noise_amplitude", this->noiseAmp, 0.0);
  GZ_ASSERT(this->noiseAmp >= 0.0, "Noise amplitude must be >= 0");

  this->AddNoiseModel("default", this->noiseSigma);
  return true;
}

void ROSBasePlugin::GetTFMessage(const tf2_msgs::msg::TFMessage::SharedPtr _msg)
{
  if (this->isReferenceInit)
    return;

  for (const auto& t : _msg->transforms)
  {
    if (t.header.frame_id == "world" &&
        t.child_frame_id == this->referenceFrameID)
    {
      this->referenceFrame.Pos() = math::Vector3d(
        t.transform.translation.x,
        t.transform.translation.y,
        t.transform.translation.z);
      this->referenceFrame.Rot() = math::Quaterniond(
        t.transform.rotation.w,
        t.transform.rotation.x,
        t.transform.rotation.y,
        t.transform.rotation.z);
      this->isReferenceInit = true;
    }
  }
}

void ROSBasePlugin::ChangeSensorState(
  const std::shared_ptr<uuv_sensor_ros_plugins_msgs::srv::ChangeSensorState::Request> _req,
  std::shared_ptr<uuv_sensor_ros_plugins_msgs::srv::ChangeSensorState::Response> _res)
{
  this->isOn.data = _req->on;
  _res->success = true;
  std::string message = this->sensorOutputTopic + (_req->on ? ":: ON" : ":: OFF");
  _res->message = message;
  gzmsg << message << std::endl;
}

void ROSBasePlugin::PublishState()
{
  this->pluginStatePub->publish(this->isOn);
}

double ROSBasePlugin::GetGaussianNoise(double _amp)
{
  return _amp * this->noiseModels["default"](this->rndGen);
}

double ROSBasePlugin::GetGaussianNoise(const std::string& _name, double _amp)
{
  GZ_ASSERT(this->noiseModels.count(_name), "Gaussian noise model does not exist");
  return _amp * this->noiseModels[_name](this->rndGen);
}

bool ROSBasePlugin::AddNoiseModel(const std::string& _name, double _sigma)
{
  if (this->noiseModels.count(_name))
    return false;
  this->noiseModels[_name] = std::normal_distribution<double>(0.0, _sigma);
  return true;
}

bool ROSBasePlugin::IsOn()
{
  return this->isOn.data;
}

bool ROSBasePlugin::EnableMeasurement(const UpdateInfo& _info) const
{
  using namespace std::chrono;
  double dt = duration<double>(_info.simTime - this->lastMeasurementTime).count();
  return dt >= 1.0 / this->updateRate && this->isReferenceInit && this->isOn.data;
}

void ROSBasePlugin::UpdateReferenceFramePose(EntityComponentManager& _ecm)
{
  if (this->referenceLink == kNullEntity)
    return;

  const auto* pose =
    _ecm.Component<components::WorldPose>(this->referenceLink);
  if (pose)
    this->referenceFrame = pose->Data();
}

}} // namespace gz::sim
