#
# Individual marker within message
#
string type # what type of marker, e.g. "apriltag"
string id # id of marker, e.g. apriltag id
geometry_msgs/Point[] object_points  # 3d point in target frame, not marker frame!
geometry_msgs/Point[] image_points   # 2d point in pixels, z is always 0
