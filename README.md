# gnss_navigation
ros2 topic echo /vectornav/gnss sensor_msgs/msg/NavSatfix --csv --qos-history keep_all --qos-reliability reliable > output.csv  
ros2 bag play rosbag -r 100
