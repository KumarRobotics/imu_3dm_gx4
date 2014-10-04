#include <ros/ros.h>
#include <ros/node_handle.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/FluidPressure.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>

#include <imu_3dm_gx4/FilterOutput.h>
#include "imu.hpp"

using namespace imu_3dm_gx4;

#define kEarthGravity (9.80665)

ros::Publisher pubIMU;
ros::Publisher pubMag;
ros::Publisher pubPressure;
ros::Publisher pubFilter;
std::string frameId;

Imu::Info info;
Imu::DiagnosticFields fields;

//  diagnostic_updater resources
std::shared_ptr<diagnostic_updater::Updater> updater;
std::shared_ptr<diagnostic_updater::TopicDiagnostic> imuDiag;
std::shared_ptr<diagnostic_updater::TopicDiagnostic> filterDiag;

void publishData(const Imu::IMUData &data) {
  sensor_msgs::Imu imu;
  sensor_msgs::MagneticField field;
  sensor_msgs::FluidPressure pressure;

  //  assume we have all of these since they were requested
  /// @todo: Replace this with a mode graceful failure...
  assert(data.fields & Imu::IMUData::Accelerometer);
  assert(data.fields & Imu::IMUData::Magnetometer);
  assert(data.fields & Imu::IMUData::Barometer);
  assert(data.fields & Imu::IMUData::Gyroscope);
  
  //  timestamp identically
  imu.header.stamp = ros::Time::now();
  imu.header.frame_id = frameId;
  field.header.stamp = imu.header.stamp;
  field.header.frame_id = frameId;
  pressure.header.stamp = imu.header.stamp;
  pressure.header.frame_id = frameId;

  imu.orientation_covariance[0] =
      -1; //  orientation data is on a separate topic

  imu.linear_acceleration.x = data.accel[0] * kEarthGravity;
  imu.linear_acceleration.y = data.accel[1] * kEarthGravity;
  imu.linear_acceleration.z = data.accel[2] * kEarthGravity;
  imu.angular_velocity.x = data.gyro[0];
  imu.angular_velocity.y = data.gyro[1];
  imu.angular_velocity.z = data.gyro[2];

  field.magnetic_field.x = data.mag[0];
  field.magnetic_field.y = data.mag[1];
  field.magnetic_field.z = data.mag[2];

  pressure.fluid_pressure = data.pressure;

  //  publish
  pubIMU.publish(imu);
  pubMag.publish(field);
  pubPressure.publish(pressure);
  if (imuDiag) {
    imuDiag->tick(imu.header.stamp);
  }
}

void publishFilter(const Imu::FilterData &data) {
  assert(data.fields & Imu::FilterData::Quaternion);
  assert(data.fields & Imu::FilterData::Bias);
  assert(data.fields & Imu::FilterData::AngleUnertainty);
  assert(data.fields & Imu::FilterData::BiasUncertainty);
  
  imu_3dm_gx4::FilterOutput output;
  output.header.stamp = ros::Time::now();
  output.header.frame_id = frameId;
  output.orientation.w = data.quaternion[0];
  output.orientation.x = data.quaternion[1];
  output.orientation.y = data.quaternion[2];
  output.orientation.z = data.quaternion[3];
  output.bias.x = data.bias[0];
  output.bias.y = data.bias[1];
  output.bias.z = data.bias[2];

  output.bias_covariance[0] = data.biasUncertainty[0]*data.biasUncertainty[0];
  output.bias_covariance[4] = data.biasUncertainty[1]*data.biasUncertainty[1];
  output.bias_covariance[8] = data.biasUncertainty[2]*data.biasUncertainty[2];
  
  output.orientation_covariance[0] = data.angleUncertainty[0]*
      data.angleUncertainty[0];
  output.orientation_covariance[4] = data.angleUncertainty[1]*
      data.angleUncertainty[1];
  output.orientation_covariance[8] = data.angleUncertainty[2]*
      data.angleUncertainty[2];
  
  output.quat_status = data.quaternionStatus;
  output.bias_status = data.biasStatus;
  output.orientation_covariance_status = data.angleUncertaintyStatus;
  output.bias_covariance_status = data.biasUncertaintyStatus;
  
  pubFilter.publish(output);
  if (filterDiag) {
    filterDiag->tick(output.header.stamp);
  }
}

std::shared_ptr<diagnostic_updater::TopicDiagnostic> configTopicDiagnostic(
    const std::string& name, double * target) {
  std::shared_ptr<diagnostic_updater::TopicDiagnostic> diag;
  const double period = 1.0 / *target;  //  for 1000Hz, period is 1e-3
  
  diagnostic_updater::FrequencyStatusParam freqParam(target, target, 0.01, 10);
  diagnostic_updater::TimeStampStatusParam timeParam(0, period * 0.5);
  diag.reset(new diagnostic_updater::TopicDiagnostic(name, 
                                                     *updater, 
                                                     freqParam,
                                                     timeParam));
  return diag;
}

void updateDiagnosticInfo(diagnostic_updater::DiagnosticStatusWrapper& stat,
                          imu_3dm_gx4::Imu* imu) {
  //  add base device info
  std::map<std::string,std::string> map = info.toMap();
  for (const std::pair<std::string,std::string>& p : map) {
    stat.add(p.first, p.second);
  }
  
  try {
    //  try to read diagnostic info
    imu->getDiagnosticInfo(fields);
    
    auto map = fields.toMap();
    for (const std::pair<std::string, unsigned int>& p : map) {
      stat.add(p.first, p.second);
    }
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Read diagnostic info.");
  }
  catch (std::exception& e) {
    const std::string message = std::string("Failed: ") + e.what();
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, message);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "imu_3dm_gx4");
  ros::NodeHandle nh("~");

  std::string device;
  int baudrate;
  bool enableFilter;
  bool enableMagUpdate, enableAccelUpdate;
  int requestedImuRate, requestedFilterRate;
  
  //  load parameters from launch file
  nh.param<std::string>("device", device, "/dev/ttyACM0");
  nh.param<int>("baudrate", baudrate, 115200);
  nh.param<std::string>("frameId", frameId, std::string("imu"));
  nh.param<int>("imu_rate", requestedImuRate, 100);
  nh.param<int>("filter_rate", requestedFilterRate, 100);
  nh.param<bool>("enable_filter", enableFilter, false);
  nh.param<bool>("enable_mag_update", enableMagUpdate, false);
  nh.param<bool>("enable_accel_update", enableAccelUpdate, true);
  
  if (requestedFilterRate < 0 || requestedImuRate < 0) {
    ROS_ERROR("imu_rate and filter_rate must be > 0");
    return -1;
  }
  
  pubIMU = nh.advertise<sensor_msgs::Imu>("imu", 1);
  pubMag = nh.advertise<sensor_msgs::MagneticField>("magnetic_field", 1);
  pubPressure = nh.advertise<sensor_msgs::FluidPressure>("pressure", 1);

  if (enableFilter) {
    pubFilter = nh.advertise<imu_3dm_gx4::FilterOutput>("filter", 1);
  }

  //  new instance of the IMU
  Imu imu(device);
  try {
    imu.connect();

    ROS_INFO("Selecting baud rate %u", baudrate);
    imu.selectBaudRate(baudrate);

    ROS_INFO("Fetching device info.");
    imu.getDeviceInfo(info);
    std::map<std::string,std::string> map = info.toMap();
    for (const std::pair<std::string,std::string>& p : map) {
      ROS_INFO("\t%s: %s", p.first.c_str(), p.second.c_str());
    }

    ROS_INFO("Idling the device");
    imu.idle();

    //  read back data rates
    uint16_t imuBaseRate, filterBaseRate;
    imu.getIMUDataBaseRate(imuBaseRate);
    ROS_INFO("IMU data base rate: %u Hz", imuBaseRate);
    imu.getFilterDataBaseRate(filterBaseRate);
    ROS_INFO("Filter data base rate: %u Hz", filterBaseRate);

    //  calculate decimation rates
    if (static_cast<uint16_t>(requestedImuRate) > imuBaseRate) {
      throw std::runtime_error("imu_rate cannot exceed " + 
                               std::to_string(imuBaseRate));
    }
    if (static_cast<uint16_t>(requestedFilterRate) > filterBaseRate) {
      throw std::runtime_error("filter_rate cannot exceed " + 
                               std::to_string(filterBaseRate));
    }
    
    const uint16_t imuDecimation = imuBaseRate / requestedImuRate;
    const uint16_t filterDecimation = filterBaseRate / requestedFilterRate;
    
    ROS_INFO("Selecting IMU decimation: %u", imuDecimation);
    imu.setIMUDataRate(
        imuDecimation, Imu::IMUData::Accelerometer | 
          Imu::IMUData::Gyroscope |
          Imu::IMUData::Magnetometer |
          Imu::IMUData::Barometer);

    ROS_INFO("Selecting filter decimation: %u", filterDecimation);
    imu.setFilterDataRate(filterDecimation, Imu::FilterData::Quaternion |
                          Imu::FilterData::Bias |
                          Imu::FilterData::AngleUnertainty |
                          Imu::FilterData::BiasUncertainty);

    ROS_INFO("Enabling IMU data stream");
    imu.enableIMUStream(true);

    if (enableFilter) {
      ROS_INFO("Enabling filter data stream");
      imu.enableFilterStream(true);

      ROS_INFO("Enabling filter measurements");
      imu.enableMeasurements(enableAccelUpdate, enableMagUpdate);

      ROS_INFO("Enabling gyro bias estimation");
      imu.enableBiasEstimation(true);
    } else {
      ROS_INFO("Disabling filter data stream");
      imu.enableFilterStream(false);
    }
    imu.setIMUDataCallback(publishData);
    imu.setFilterDataCallback(publishFilter);

    //  configure diagnostic updater
    if (!nh.hasParam("diagnostic_period")) {
      nh.setParam("diagnostic_period", 0.2);  //  5hz period
    }
    
    updater.reset(new diagnostic_updater::Updater());
    const std::string hwId = info.modelName + "-" + info.modelNumber;
    updater->setHardwareID(hwId);
    
    //  calculate the actual rates we will get
    double imuRate = imuBaseRate / (1.0 * imuDecimation);
    double filterRate = filterBaseRate / (1.0 * filterDecimation);
    imuDiag = configTopicDiagnostic("imu",&imuRate);
    if (enableFilter) {
      filterDiag = configTopicDiagnostic("filter",&filterRate);
    }
    
    updater->add("diagnostic_info", 
                 boost::bind(&updateDiagnosticInfo, _1, &imu));
    
    ROS_INFO("Resuming the device");
    imu.resume();

    while (ros::ok()) {
      imu.runOnce();
      updater->update();
    }
    imu.disconnect();
  }
  catch (Imu::io_error &e) {
    ROS_ERROR("IO error: %s\n", e.what());
  }
  catch (Imu::timeout_error &e) {
    ROS_ERROR("Timeout: %s\n", e.what());
  }
  catch (std::exception &e) {
    ROS_ERROR("Exception: %s\n", e.what());
  }

  return 0;
}
