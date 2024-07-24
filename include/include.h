#ifndef INCLUDE
#define INCLUDE

#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <sstream>
#include <istream>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

// Visualization Pointcloud related
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

// Setting default precision for std::cout globally
struct CoutSettings {
    CoutSettings() {
        std::cout << std::fixed << std::setprecision(5);
    }
};
static CoutSettings coutSettings;

#endif