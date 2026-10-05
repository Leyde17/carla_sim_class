# Object extent. BOUNDING_BOX uses `dimensions` (x=length, y=width, z=height)
# and ignores `footprint`; POLYGON uses `footprint` plus `dimensions.z` as the
# height; CYLINDER uses dimensions.x as the diameter.

uint8 BOUNDING_BOX=0
uint8 CYLINDER=1
uint8 POLYGON=2

uint8 type
geometry_msgs/Polygon footprint
geometry_msgs/Vector3 dimensions
