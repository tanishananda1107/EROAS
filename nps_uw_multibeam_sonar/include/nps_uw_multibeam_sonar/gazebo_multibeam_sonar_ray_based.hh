/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
<<<<<<< HEAD
 * http://www.apache.org/licenses/LICENSE-2.0
=======
 *     http://www.apache.org/licenses/LICENSE-2.0
>>>>>>> bde8874 (Remove unused directories from navigator_auv)
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
<<<<<<< HEAD
 *
*/

#ifndef GAZEBO_PLUGINS_FIDUCIAL_DATA_HH
#define GAZEBO_PLUGINS_FIDUCIAL_DATA_HH

#include <string>
#include <gz/math/Vector2.hh>

namespace gazebo_plugins
{
  /// \brief Data class mapping fiducial frame targets onto modern Gazebo Harmonic Math conventions
  class FiducialData
  {
    /// \brief Unique marker identity identifier
    public: std::string id;

    /// \brief Target coordinate intersections within the 2D plane coordinate system
    public: gz::math::Vector2i pt;
  };
}

#endif // GAZEBO_PLUGINS_FIDUCIAL_DATA_HH
=======
*/

#ifndef GZ_SIM_MULTIBEAM_SONAR_RAY_HH
#define GZ_SIM_MULTIBEAM_SONAR_RAY_HH

// ── ROS 2 ─────────────────────────────────────────────────────────────────
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/fill_image.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <image_transport/image_transport.hpp>
#include <marine_acoustic_msgs/msg/projected_sonar_image.hpp>
#include <marine_acoustic_msgs/msg/sonar_image_data.hpp>
#include <marine_acoustic_msgs/msg/ping_info.hpp>

// ── Gazebo Harmonic (gz-sim 8) ────────────────────────────────────────────
#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/components/GpuLidar.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/World.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/pointcloud_packed.pb.h>
#include <gz/common/Time.hh>
#include <gz/math/Angle.hh>
#include <gz/rendering/GpuRays.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/sensors/GpuLidarSensor.hh>

// SDF
#include <sdf/Element.hh>
#include <sdf/Param.hh>

// Selection buffer
#include "selection_buffer/SelectionBuffer.hh"

// Third-party / standard
#include <opencv2/core.hpp>
#include <boost/thread/mutex.hpp>
#include <string>
#include <complex>
#include <valarray>
#include <vector>
#include <sstream>
#include <chrono>
#include <thread>
#include <fstream>
#include <memory>

namespace gz
{
namespace sim
{

typedef std::complex<float>    Complex;
typedef std::valarray<Complex> CArray;
typedef std::valarray<CArray>  CArray2D;
typedef std::valarray<float>   Array;
typedef std::valarray<Array>   Array2D;

class FiducialData
{
  public: std::string id;
  public: gz::math::Vector2i pt;
};

class NpsGazeboRos2MultibeamSonarRay :
    public System,
    public ISystemConfigure,
    public ISystemPreUpdate,
    public ISystemPostUpdate
{
  // ── Lifecycle ─────────────────────────────────────────────────────────
  public: NpsGazeboRos2MultibeamSonarRay();
  public: ~NpsGazeboRos2MultibeamSonarRay() override;

  public: void Configure(
              const Entity & _entity,
              const std::shared_ptr<const sdf::Element> & _sdf,
              EntityComponentManager & _ecm,
              EventManager & _eventMgr) override;

  public: void PreUpdate(
              const UpdateInfo & _info,
              EntityComponentManager & _ecm) override;

  public: void PostUpdate(
              const UpdateInfo & _info,
              const EntityComponentManager & _ecm) override;

  // ── Sensor dimensions ─────────────────────────────────────────────────
  protected: unsigned int width_{0}, height_{0}, depth_{0};
  protected: std::string  format_;

  // ── gz rendering / sensor ─────────────────────────────────────────────
  protected: gz::rendering::GpuRaysPtr        gpuRays_;
  protected: gz::sensors::GpuLidarSensor *    parentSensor_{nullptr};

  // ── gz transport ──────────────────────────────────────────────────────
  private: gz::transport::Node gzNode_;
  private: std::string         gzLidarTopic_;

  // ── gz entity / time ──────────────────────────────────────────────────
  private: Entity          sensorEntity_{kNullEntity};
  private: gz::common::Time sensorUpdateTime_;

  // ── ROS 2 ─────────────────────────────────────────────────────────────
  public: void Advertise();

  private: rclcpp::Node::SharedPtr rosNode_;

  private: rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
               pointCloudPub_;
  private: rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
               normalImagePub_;
  private: rclcpp::Publisher
               marine_acoustic_msgs::msg::ProjectedSonarImage>::SharedPtr
               sonarImageRawPub_;
  private: rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
               sonarImagePub_;

  /// \brief Subscriber replacing ros::Subscriber VelodyneGpuLaserPointCloud
  private: rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
               pointCloudSub_;

  /// \brief Callback group + executor replacing ros::CallbackQueue
  private: rclcpp::CallbackGroup::SharedPtr   pointCloudCbGroup_;
  private: rclcpp::executors::SingleThreadedExecutor::SharedPtr
               pointCloudExecutor_;
  private: std::thread pointCloudSubThread_;
  private: void PointCloudSubThreadFunc();

  private: int pointCloudConnectCount_{0};
  private: int sonarImageConnectCount_{0};

  private: sensor_msgs::msg::PointCloud2        pointCloudMsg_;
  private: sensor_msgs::msg::Image              normalImageMsg_;
  private: marine_acoustic_msgs::msg::ProjectedSonarImage sonarImageRawMsg_;
  private: sensor_msgs::msg::Image              sonarImageMsg_;
  private: sensor_msgs::msg::Image              sonarImageMonoMsg_;
  private: cv::Mat                              pointCloudImage_;
  private: cv::Mat                              pointCloudNormalImage_;

  private: std::string pointCloudTopicName_;
  private: std::string sonarImageRawTopicName_;
  private: std::string sonarImageTopicName_;
  private: double      pointCloudCutoff_{0.0};

  // ── Sonar processing ──────────────────────────────────────────────────
  /// Replaces UpdatePointCloud(const sensor_msgs::PointCloud2ConstPtr&)
  private: void UpdatePointCloud(
               const sensor_msgs::msg::PointCloud2::SharedPtr _msg);
  private: void    ComputeSonarImage();
  private: cv::Mat ComputeNormalImage(cv::Mat & depth);
  private: void    ComputeCorrector();
  private: cv::Mat randImage_;

  // ── Sonar parameters ──────────────────────────────────────────────────
  private: double sonarFreq_{0.0};
  private: double bandwidth_{0.0};
  private: double soundSpeed_{0.0};
  private: double maxDistance_{0.0};
  private: double sourceLevel_{0.0};
  private: bool   constMu_{true};
  private: double absorption_{0.0};
  private: double attenuation_{0.0};
  private: double verticalFOV_{0.0};
  private: double mu_{0.0};
  private: bool   calculateReflectivity_{false};
  private: cv::Mat reflectivityImage_;
  private: std::vector<float> azimuthAngles_;
  private: float * elevationAngles_{nullptr};
  private: float * rangeVector_{nullptr};
  private: float * window_{nullptr};
  private: float **beamCorrector_{nullptr};
  private: float   beamCorrectorSum_{0.f};
  private: int     nFreq_{0};
  private: double  df_{0.0};
  private: int     nBeams_{0};
  private: int     nRays_{0};
  private: int     beamSkips_{0};
  private: int     raySkips_{0};
  private: int     rayNAzimuthRays_{0};
  private: int     rayNElevationRays_{0};
  private: float   plotScaler_{0.f};
  private: float   sensorGain_{0.f};
  protected: bool  debugFlag_{false};

  // ── CSV logging ───────────────────────────────────────────────────────
  protected: std::ofstream writeLog_;
  protected: uint64_t      writeCounter_{0};
  protected: uint64_t      writeNumber_{0};
  protected: uint64_t      writeInterval_{0};
  protected: bool          writeLogFlag_{false};
};

inline double unnormalized_sinc(double t)
{
  const double r = std::sin(t) / t;
  return std::isnan(r) ? 1.0 : r;
}

} // namespace sim
} // namespace gz

GZ_ADD_PLUGIN(
    gz::sim::NpsGazeboRos2MultibeamSonarRay,
    gz::sim::System,
    gz::sim::NpsGazeboRos2MultibeamSonarRay::ISystemConfigure,
    gz::sim::NpsGazeboRos2MultibeamSonarRay::ISystemPreUpdate,
    gz::sim::NpsGazeboRos2MultibeamSonarRay::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(
    gz::sim::NpsGazeboRos2MultibeamSonarRay,
    "gz::sim::NpsGazeboRos2MultibeamSonarRay")

#endif // GZ_SIM_MULTIBEAM_SONAR_RAY_HH
>>>>>>> bde8874 (Remove unused directories from navigator_auv)
