/*
<<<<<<< HEAD
 * Copyright (C) 2012 Open Source Robotics Foundation
=======
 * Copyright 2020 Naval Postgraduate School
>>>>>>> bde8874 (Remove unused directories from navigator_auv)
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

#ifndef GAZEBO_ROS_MULTIBEAM_SONAR_RAY_HH
#define GAZEBO_ROS_MULTIBEAM_SONAR_RAY_HH

// ROS 2 Core Headers
#include <rclcpp/rclcpp.hpp>

// ROS 2 Message Types
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/vector3.hpp>

// Marine Acoustic Message Definitions (ROS 2 Versions)
=======
*/

#ifndef GZ_SIM_MULTIBEAM_SONAR_HH
#define GZ_SIM_MULTIBEAM_SONAR_HH

// ── ROS 2 ─────────────────────────────────────────────────────────────────
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/fill_image.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <image_transport/image_transport.hpp>
>>>>>>> bde8874 (Remove unused directories from navigator_auv)
#include <marine_acoustic_msgs/msg/projected_sonar_image.hpp>
#include <marine_acoustic_msgs/msg/sonar_image_data.hpp>
#include <marine_acoustic_msgs/msg/ping_info.hpp>

<<<<<<< HEAD
// Gazebo Harmonic (Gz Sim 8) Headers
#include <gz/sim/System.hh>
#include <gz/sim/Sensor.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/laserscan.pb.h>

// Gazebo Math & Rendering Headers
#include <gz/math/Vector2.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>

// OpenCV Core
#include <opencv2/core.hpp>

// Standard C++ Implementations
#include <string>
=======
// ── Gazebo Harmonic (gz-sim 8) ────────────────────────────────────────────
#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/components/DepthCamera.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/camera_info.pb.h>
#include <rclcpp/time.hpp>
#include <gz/math/Angle.hh>
#include <gz/rendering/Camera.hh>
#include <gz/rendering/DepthCamera.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderingIface.hh>

// SDF
#include <sdf/Element.hh>

// Selection buffer
#include "selection_buffer/SelectionBuffer.hh"

// Third-party / standard
#include <opencv2/core.hpp>
#include <boost/thread/mutex.hpp>
>>>>>>> bde8874 (Remove unused directories from navigator_auv)
#include <complex>
#include <valarray>
#include <sstream>
#include <chrono>
<<<<<<< HEAD
#include <memory>
#include <mutex>
#include <thread>
#include <fstream>
#include <vector>

namespace gazebo_plugins
{
  typedef std::complex<float> Complex;
  typedef std::valarray<Complex> CArray;
  typedef std::valarray<CArray> CArray2D;

  typedef std::valarray<float> Array;
  typedef std::valarray<Array> Array2D;

  /// \brief Gazebo Harmonic System Plugin mapping a Multibeam Sonar Ray sensor over ROS 2
  class NpsGazeboRosMultibeamSonarRay : 
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPostUpdate
  {
    /// \brief Constructor
    public: NpsGazeboRosMultibeamSonarRay();

    /// \brief Destructor
    public: virtual ~NpsGazeboRosMultibeamSonarRay();

    /// \brief Configures plugin data components on entity loading (Replaces legacy Load)
    public: void Configure(const gz::sim::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &_eventMgr) override;

    /// \brief Post-update hook triggered every calculation cycle step
    public: void PostUpdate(const gz::sim::UpdateInfo &_info,
                            const gz::sim::EntityComponentManager &_ecm) override;

    /// \brief Native subscriber processing new incoming Gazebo GPU sensor frame messages
    private: void OnNewLaserFrame(const gz::msgs::LaserScan &_msg);

    // Image & Array structural layout sizes
    protected: unsigned int width, height, depth;
    protected: std::string format;

    // Gazebo Transport Node and Entity bindings
    private: gz::sim::Entity sensorEntity;
    private: gz::transport::Node gzNode;
    
    // ROS 2 Multithreading Thread-Execution infrastructure 
    private: rclcpp::Node::SharedPtr ros_node_;
    private: rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
    private: std::thread thread_executor_;

    // ROS 2 Publishers
    private: rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
    private: rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr normal_image_pub_;
    private: rclcpp::Publisher<marine_acoustic_msgs::msg::ProjectedSonarImage>::SharedPtr sonar_image_raw_pub_;
    private: rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr sonar_image_pub_;

    // ROS 2 Subscribers
    private: rclcpp::Subscriber<sensor_msgs::msg::PointCloud2>::SharedPtr velodyne_gpu_laser_point_cloud_sub_;

    // Connectivity logic structures
    private: std::mutex mutex_;
    private: int point_cloud_connect_count_;
    private: int sonar_image_connect_count_;
    private: void PointCloudConnect();
    private: void PointCloudDisconnect();
    private: void SonarImageConnect();
    private: void SonarImageDisconnect();

    /// \brief Calculations and image updates
    private: void UpdatePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr _msg);
    private: void ComputeSonarImage();
    private: cv::Mat ComputeNormalImage(cv::Mat& depth_mat);
    private: double point_cloud_cutoff_;

    private: void ComputeCorrector();
    private: cv::Mat rand_image;

    /// \brief Parameters for sonar acoustic modeling
    private: double sonarFreq;
    private: double bandwidth;
    private: double soundSpeed;
    private: double maxDistance;
    private: double sourceLevel;
    private: bool constMu;
    private: double absorption;
    private: double attenuation;
    private: double verticalFOV;
    private: double mu;
    private: bool calculateReflectivity;
    private: cv::Mat reflectivityImage;
    private: std::vector<float> azimuth_angles;
    private: float* elevation_angles;
    private: float* rangeVector;
    private: float* window;
    private: float** beamCorrector;
    private: float beamCorrectorSum;
    private: int nFreq;
    private: double df;
    private: int nBeams;
    private: int nRays;
    private: int beamSkips;
    private: int raySkips;
    private: int ray_nAzimuthRays;
    private: int ray_nElevationRays;
    private: float plotScaler;
    private: float sensorGain;
    protected: bool debugFlag;

    // Local cached variables used to populate topics
    private: sensor_msgs::msg::PointCloud2 point_cloud_msg_;
    private: sensor_msgs::msg::Image normal_image_msg_;
    private: marine_acoustic_msgs::msg::ProjectedSonarImage sonar_image_raw_msg_;
    private: sensor_msgs::msg::Image sonar_image_msg_;
    private: sensor_msgs::msg::Image sonar_image_mono_msg_;
    private: cv::Mat point_cloud_image_;
    private: cv::Mat point_cloud_normal_image_;

    private: std::string point_cloud_topic_name_;
    private: std::string sonar_image_raw_topic_name_;
    private: std::string sonar_image_topic_name_;

    /// \brief Logger parameters
    protected: std::ofstream writeLog;
    protected: uint64_t writeCounter;
    protected: uint64_t writeNumber;
    protected: uint64_t writeInterval;
    protected: bool writeLogFlag;
  };

  ///////////////////////////////////////////
  inline double unnormalized_sinc(double t)
  {
    if (std::abs(t) < 1e-9)
    {
      return 1.0;
    }
    return std::sin(t) / t;
  }
}

#endif // GAZEBO_ROS_MULTIBEAM_SONAR_RAY_HH
=======
#include <string>
#include <set>
#include <vector>
#include <memory>
#include <fstream>
#include <random>

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

class NpsGazeboRos2MultibeamSonar :
    public System,
    public ISystemConfigure,
    public ISystemPreUpdate,
    public ISystemPostUpdate
{
  // ── Lifecycle ─────────────────────────────────────────────────────────
  public: NpsGazeboRos2MultibeamSonar();
  public: ~NpsGazeboRos2MultibeamSonar() override;

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

  // ── Fiducial / selection-buffer ────────────────────────────────────────
  private: void PopulateFiducials(const EntityComponentManager & _ecm);
  public:  std::unique_ptr<gz::rendering::SelectionBuffer> selectionBuffer;
  public:  gz::rendering::ScenePtr scene;
  public:  bool detectAll{false};
  public:  std::set<std::string> fiducials;

  // ── ROS 2 ─────────────────────────────────────────────────────────────
  public: void Advertise();

  private: rclcpp::Node::SharedPtr rosNode_;

  private: rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
               depthImagePub_;
  private: rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
               normalImagePub_;
  private: rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
               pointCloudPub_;
  private: rclcpp::Publisher
               marine_acoustic_msgs::msg::ProjectedSonarImage>::SharedPtr
               sonarImageRawPub_;
  private: rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
               sonarImagePub_;
  private: rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr
               depthImageCameraInfoPub_;

  private: sensor_msgs::msg::Image              depthImageMsg_;
  private: sensor_msgs::msg::Image              normalImageMsg_;
  private: sensor_msgs::msg::PointCloud2        pointCloudMsg_;
  private: marine_acoustic_msgs::msg::ProjectedSonarImage sonarImageRawMsg_;
  private: sensor_msgs::msg::Image              sonarImageMsg_;
  private: sensor_msgs::msg::Image              sonarImageMonoMsg_;
  private: cv::Mat                              pointCloudImage_;

  private: std::string depthImageTopicName_;
  private: std::string depthImageCameraInfoTopicName_;
  private: std::string pointCloudTopicName_;
  private: std::string sonarImageRawTopicName_;
  private: std::string sonarImageTopicName_;
  private: double      pointCloudCutoff_{0.0};

  protected: virtual void PublishCameraInfo();
  private: rclcpp::Time lastDepthImageCameraInfoUpdateTime_;

  // ── Sonar processing ──────────────────────────────────────────────────
  private: void   ComputeSonarImage(const float * _src);
  private: void   ComputePointCloud(const float * _src);
  private: double ComputeIncidence(double azimuth, double elevation,
                                   cv::Vec3f normal);
  private: cv::Mat ComputeNormalImage(cv::Mat & depth);
  private: void   ComputeCorrector();
  private: cv::Mat randImage_;

  // ── Sonar parameters ──────────────────────────────────────────────────
  private: double sonarFreq_{0.0};
  private: double bandwidth_{0.0};
  private: double soundSpeed_{0.0};
  private: double maxDistance_{0.0};
  private: double sourceLevel_{0.0};
  private: bool   constMu_{true};
  private: bool   customTag_{false};
  private: double absorption_{0.0};
  private: double attenuation_{0.0};
  private: double verticalFOV_{0.0};
  private: double mu_{0.0};
  private: std::string reflectivityDatabaseFileName_;
  private: std::string reflectivityDatabaseFilePath_;
  private: std::string customTagDatabaseFileName_;
  private: std::string customTagDatabaseFilePath_;
  private: std::vector<std::string> objectNames_;
  private: std::vector<float>       reflectivities_;
  private: double biofoulingRatingCoeff_{0.0};
  private: double roughnessCoeff_{0.0};
  private: double maxDepth_{0.0}, maxDepthBefore_{0.0},
                  maxDepthBeforeBefore_{0.0};
  private: double maxDepthPrev_{0.0};
  private: bool   calculateReflectivity_{false};
  private: bool   artificialVehicleVibration_{false};
  private: cv::Mat reflectivityImage_;
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
  private: float * elevationAngles_{nullptr};
  private: float   plotScaler_{0.f};
  private: float   sensorGain_{0.f};
  protected: bool  debugFlag_{false};

  // ── CSV logging ───────────────────────────────────────────────────────
  protected: std::ofstream writeLog_;
  protected: uint64_t      writeCounter_{0};
  protected: uint64_t      writeNumber_{0};
  protected: uint64_t      writeInterval_{0};
  protected: bool          writeLogFlag_{false};

  // ── gz-sim entity / sensor ────────────────────────────────────────────
  private: Entity sensorEntity_{kNullEntity};

  protected: unsigned int width_{0}, height_{0}, depth_{0};
  protected: std::string  format_;

  protected: gz::rendering::DepthCameraPtr depthCamera_;
  private:   gz::transport::Node gzNode_;
  private:   bool depthFrameConnected_{false};

  private: std::default_random_engine generator_;
  private: int depthImageConnectCount_{0};
  private: int pointCloudConnectCount_{0};
  private: int sonarImageConnectCount_{0};

  // ── Angle helpers ─────────────────────────────────────────────────────
  private: inline double Azimuth(int col) const
  {
    if (width_ <= 1) return 0.0;
    const double hfov = depthCamera_->HFOV().Radian();
    const double fl = static_cast<double>(width_) / (2.0 * std::tan(hfov / 2.0));
    return std::atan2(static_cast<double>(col) -
                      0.5 * static_cast<double>(width_ - 1), fl);
  }

  private: inline double Elevation(int row) const
  {
    if (height_ <= 1) return 0.0;
    const double hfov = depthCamera_->HFOV().Radian();
    const double fl = static_cast<double>(width_) / (2.0 * std::tan(hfov / 2.0));
    return std::atan2(static_cast<double>(row) -
                      0.5 * static_cast<double>(height_ - 1), fl);
  }
};

inline double unnormalized_sinc(double t)
{
  const double r = std::sin(t) / t;
  return std::isnan(r) ? 1.0 : r;
}

} // namespace sim
} // namespace gz

GZ_ADD_PLUGIN(
    gz::sim::NpsGazeboRos2MultibeamSonar,
    gz::sim::System,
    gz::sim::NpsGazeboRos2MultibeamSonar::ISystemConfigure,
    gz::sim::NpsGazeboRos2MultibeamSonar::ISystemPreUpdate,
    gz::sim::NpsGazeboRos2MultibeamSonar::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(
    gz::sim::NpsGazeboRos2MultibeamSonar,
    "gz::sim::NpsGazeboRos2MultibeamSonar")

#endif // GZ_SIM_MULTIBEAM_SONAR_HH
>>>>>>> bde8874 (Remove unused directories from navigator_auv)
