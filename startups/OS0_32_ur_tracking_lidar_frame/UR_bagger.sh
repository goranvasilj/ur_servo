rosbag record -b 0 /hawk1/uav_pose \
  /hawk1/uav_pcl_detection_ground/pose_array \
  /ouster/points_filtered \
  /tf \
  /tool_pose \
  /ur_pixhawk/mavros/imu/data \
  -O UR_tracking_imu.bag
