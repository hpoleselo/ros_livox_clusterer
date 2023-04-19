# ros_livox_clusterer

Clusterizes Livox Point Cloud using HDBSCAN.

## Installation

The validation of this Clusterer package is done through [Hilti SLAM Challenge Dataset](https://www.hilti-challenge.com/dataset-2021.html),which makes use of Livox Mid 70. Therefore one should provide minimal installation from [Livox ROS Driver](https://github.com/Livox-SDK/livox_ros_driver) in order to succesfully work with Livox in ROS.

In order to check the robot's footage:

`$ rosbag play decompress bagfile.bag`

Listen to the topic from the camera to view one of the cameras:

` $ rosrun image_view image_view image:=/alphasense/cam1/image_raw`

1. Run the interface for getting Livox messages and visualizing the Point Cloud in RViz:

(This may expect a sensor_msgs/PointCloud2 msg. type)

` $ roslaunch livox_ros_driver livox_lidar_rviz.launch `

2. Probably a remapping is needed?


