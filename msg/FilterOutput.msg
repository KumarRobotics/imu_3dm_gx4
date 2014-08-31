# Output from the 3DM-GX4 attitude estimation filter.
std_msgs/Header header

# Node on status flags:
# Status flags are implemented as bit-fields.
#  0 = invalid
#  1 = valid
#  2 = valid and geo-magnetically referenced (quat_status only)
#
# Note that covariance on orientation becomes invalid as pitch angle exceeds 70 # degrees. This will be indicated by the status flag.

# Quaternion status:
uint16 quat_status

# Bias status
uint16 bias_status

# Orientation estimate and diagonal covariance
geometry_msgs/Quaternion orientation
float64[9] orientation_covariance

# Gyro bias and diagonal covariance
geometry_msgs/Vector3 bias
float64[9] bias_covariance

# Covariance statuses
uint16 bias_covariance_status
uint16 orientation_covariance_status

# Constants
uint16 STATUS_INVALID = 0
uint16 STATUS_VALID = 1
uint16 STATUS_VALID_REFERENCED = 2
