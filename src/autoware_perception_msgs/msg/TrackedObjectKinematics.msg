# A track's state. Unlike a detection, a track always carries twist and
# acceleration -- estimating them is what a tracker is for -- so there are no
# has_* flags here, only the covariances to say how much to trust them.

# orientation_availability
uint8 UNAVAILABLE=0
uint8 SIGN_UNKNOWN=1
uint8 AVAILABLE=2

geometry_msgs/PoseWithCovariance pose_with_covariance
geometry_msgs/TwistWithCovariance twist_with_covariance
geometry_msgs/AccelWithCovariance acceleration_with_covariance
uint8 orientation_availability
bool is_stationary
