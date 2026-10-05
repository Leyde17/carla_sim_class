# `object_id` is the identity that persists across frames. It is the field that
# makes this a track and not a detection, so it must stay stable through
# occlusions for as long as the tracker believes it is the same object.
unique_identifier_msgs/UUID object_id
float32 existence_probability
ObjectClassification[] classification
TrackedObjectKinematics kinematics
Shape shape
