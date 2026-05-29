// Ported to ROS 2 / Gazebo Harmonic (gz-sim 8)
#ifndef __UUV_IMU_ROS_PLUGIN_HH__
#define __UUV_IMU_ROS_PLUGIN_HH__

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <uuv_sensor_ros_plugins/ROSBaseModelPlugin.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Quaternion.hh>

#define K_DEFAULT_ADIS_GYROSCOPE_NOISE_DENSITY              2.0 * 35.0 / 3600.0 / 180.0 * M_PI
#define K_DEFAULT_ADIS_GYROSCOPE_RANDOM_WALK                2.0 * 4.0 / 3600.0 / 180.0 * M_PI
#define K_DEFAULT_ADIS_GYROSCOPE_BIAS_CORRELATION_TIME      1.0e+3
#define K_DEFAULT_ADIS_GYROSCOPE_TURN_ON_BIAS_SIGMA         0.5 / 180.0 * M_PI
#define K_DEFAULT_ADIS_ACCELEROMETER_NOISE_DENSITY          2.0 * 2.0e-3
#define K_DEFAULT_ADIS_ACCELEROMETER_RANDOM_WALK            2.0 * 3.0e-3
#define K_DEFAULT_ADIS_ACCELEROMETER_BIAS_CORRELATION_TIME  300.0
#define K_DEFAULT_ADIS_ACCELEROMETER_TURN_ON_BIAS_SIGMA     20.0e-3 * 9.8
#define K_DEFAULT_ORIENTATION_NOISE                         0.5

namespace gz { namespace sim {

struct IMUParameters {
  double gyroscopeNoiseDensity, gyroscopeRandomWalk, gyroscopeBiasCorrelationTime,
         gyroscopeTurnOnBiasSigma, accelerometerNoiseDensity, accelerometerRandomWalk,
         accelerometerBiasCorrelationTime, accelerometerTurnOnBiasSigma, orientationNoise;
  IMUParameters()
    : gyroscopeNoiseDensity(K_DEFAULT_ADIS_GYROSCOPE_NOISE_DENSITY),
      gyroscopeRandomWalk(K_DEFAULT_ADIS_GYROSCOPE_RANDOM_WALK),
      gyroscopeBiasCorrelationTime(K_DEFAULT_ADIS_GYROSCOPE_BIAS_CORRELATION_TIME),
      gyroscopeTurnOnBiasSigma(K_DEFAULT_ADIS_GYROSCOPE_TURN_ON_BIAS_SIGMA),
      accelerometerNoiseDensity(K_DEFAULT_ADIS_ACCELEROMETER_NOISE_DENSITY),
      accelerometerRandomWalk(K_DEFAULT_ADIS_ACCELEROMETER_RANDOM_WALK),
      accelerometerBiasCorrelationTime(K_DEFAULT_ADIS_ACCELEROMETER_BIAS_CORRELATION_TIME),
      accelerometerTurnOnBiasSigma(K_DEFAULT_ADIS_ACCELEROMETER_TURN_ON_BIAS_SIGMA),
      orientationNoise(K_DEFAULT_ORIENTATION_NOISE) {}
};

class IMUROSPlugin : public ROSBaseModelPlugin {
public:
  IMUROSPlugin();
  virtual ~IMUROSPlugin();
  void Configure(const Entity& _entity,
                 const std::shared_ptr<const sdf::Element>& _sdf,
                 EntityComponentManager& _ecm, EventManager& _eventMgr) override;

protected:
  bool OnUpdate(const UpdateInfo& _info, EntityComponentManager& _ecm) override;
  void AddNoise(gz::math::Vector3d& _linAcc, gz::math::Vector3d& _angVel,
                gz::math::Quaterniond& _orientation, double _dt);

  gz::math::Vector3d measLinearAcc, measAngularVel, gravityWorld;
  gz::math::Quaterniond measOrientation;
  gz::math::Vector3d gyroscopeBias, accelerometerBias,
                     gyroscopeTurnOnBias, accelerometerTurnOnBias;
  IMUParameters imuParameters;
  sensor_msgs::msg::Imu imuROSMessage;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPub;
};

}}  // namespace gz::sim
#endif
