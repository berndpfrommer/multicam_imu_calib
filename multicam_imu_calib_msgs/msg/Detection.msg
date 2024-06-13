#
# Detection message produced by multicam_imu_calib frontend
#
uint32 id # unique id of target (which calib board)
geometry_msgs/Point[] object_points  # 3d point in object frame
geometry_msgs/Point[] image_points   # 2d point in pixels, z is always 0
