# Using libpointmatcher in ROS

If you want to use libpointmatcher in [ROS](http://www.ros.org/) or [ROS 2](https://docs.ros.org/en/iron/index.html), you can use the norlab mapping stack:

+ [libpointmatcher_ros](https://github.com/norlab-ulaval/libpointmatcher_ros) allows the conversion of pointclouds from ROS message formats to a libpointmatcher-compatible format and provides a mapping node that is already functional and that can be customized using YAML configuration files to suite your needs.
+ [norlab_icp_mapper](https://github.com/norlab-ulaval/norlab_icp_mapper) is a configurable mapping library relying on libpointmatcher for point cloud registration.
+ [norlab_icp_mapper_ros](https://github.com/norlab-ulaval/norlab_icp_mapper_ros) is a bridge between the mapper and ROS, providing message conversions to libpointmatcher compatible format.

[Norlab](https://github.com/norlab-ulaval/) also develops and maintain a number of other libraries for robotics and deep learning. Here's a non-exhaustive list:

+ [imu_odom](https://github.com/norlab-ulaval/imu_odom) estimates IMU poses based on ICP poses and accelerometer measurements. Useful when your robot doesn't have access to wheel odometry.
+ [norlab_controllers](https://github.com/norlab-ulaval/norlab_controllers) is a library with different control algorithms for using robots in the field.
+ [norlab_controllers_ros](https://github.com/norlab-ulaval/norlab_controllers_ros) is a ROS wrapper for the controller library.