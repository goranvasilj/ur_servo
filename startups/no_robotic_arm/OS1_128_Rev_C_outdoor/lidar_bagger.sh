rosbag record -b 0 /hawk1/uav_pose \
  /hawk1/es_ekf/odom \
  /hawk1/carrot/pose \
  /hawk1/lidar_state \
  /hawk1/uav_pcl_detection_ground/pose_array \
  /ouster/points_filtered \
  -O small_object_lidar

