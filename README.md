# imu_3dm_gx4

![Picture of IMU](http://www.microstrain.com/sites/default/files/styles/product_image_main/public/products/GX4-25_ProductImage1.00.jpg?itok=gkOa-CBI)

The `imu_3dm_gx4` package provides support for the [Lord Corporation](http://www.microstrain.com) Microstrain [3DM-GX4-25](http://www.microstrain.com/inertial/3dm-gx4-25) series IMU. The package employs the MIP packet format, so it could conceivably be adapted to support other versions of Microstrain products with relatively little effort. At present, the 15 and 45 series AHRS systems are not supported.

This package works on Ubuntu 14.04 (ROS _indigo_) or later.

## Version History

* **0.0.2**:
  - Units of acceleration are now m/s^2, in agreement with ROS convention.
  - Cleaned up code base, replaced error codes with exceptions.
  - 
  - Removed TF broadcast option.
  - Reformated code base to clang-llvm convention.
* **0.0.1**:
  - First release.

## Options

The `imu_3dm_gx4` node supports the following base options:
* `device`: Path to the device in `/dev`. Defaults to `/dev/ttyACM0`.
* `baudrate`: Baudrate to employ with serial communication. Defaults to `115200`.
* `imu_decimation`: IMU decimation rate to use. Determines the IMU data rate according to: `hz = 1000 / decimation`. Default is 10.

The following additional options are present for leveraging the 3DM's onboard estimation filter:
* `enable_filter`: If true, the IMU estimation filter is enabled. Default is false.
* `filter_decimation`: Estimation filter rate to use. Determines the filter data rate according to: `hz = 500 / decimation`. Default is 5.
* `enable_mag_update`: If true, the IMU will use the magnetometer to correct the heading angle estimate. Default is false.

**In order to launch the node** (streaming IMU data at 100Hz), execute:

`$ roslaunch imu_3dm_gx4 imu.launch`

## Output

On launch, the node will configure the IMU according to the parameters and then enable streaming node. At least three topics are published:

* `/imu_3dm_gx4/imu`: An instance of `sensor_msgs/Imu`. Orientation quaternion not provided in this message.
* `/imu_3dm_gx4/magnetic_field`: An instance of `sensor_msgs/MagneticField`.
* `/imu_3dm_gx4/pressure`: An instance of `sensor_msgs/FluidPressure`.

All of the above topics are published with synchronized timestamps.

Additional topics will be published if `enable_filter` is true:

* `/imu_3dm_gx4/orientation`: An instance of `geometry_msgs/StampedQuaternion`. Orientation of the device.
* `/imu_3dm_gx4/bias`: An instance of `geometry_msgs/StampedVector3` Current estimate of gyroscope biases.
* `/imu_3dm_gx4/filterStatus`: A custom message providing the status of the IMU filter. See `FilterStatus.msg` for details.

## FAQs

1. What data rates can I use?
The driver supports up to 1000Hz for IMU data, and up to 500Hz for filter data. For high data rates, it is recommended that you use a baudrate of 921600.

2. The driver can't open my device, even though the path is specified correctly - what gives??
Make sure you have ownership of the device in `/dev`, or are part of the dialout group.
