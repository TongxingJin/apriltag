# apriltag
roslaunch apriltag demo.launch

./src/opencv/apriltag_video.cpp is for apriltag detection and publish its pose in pcl::PointXYZINormal with:
(p.x, p.y, p.z) stands for center position in IMU/Lidar frame,
(p.normal_x, p.normal_y, p.normal_z) for rotation in angle axis,
p.intensity for ID. (Pls refer to ./src/opencv/apriltag_demo.cpp for usage.)

./src/opencv/apriltag_demo.cpp subscribe to the poses of april tags and visualize it. 
