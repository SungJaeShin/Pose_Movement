#ifndef UTILITY
#define UTILITY

#include "include.h"

// TF btw Rviz and Map coordinate
Eigen::Affine3f tf_coordinate()
{
    // tf conversion (0, 0, 0)
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << 0.0, 0.0, 0.0; 
    return transform;
}
   
Eigen::Matrix4f ros_tf_coordinate(tf::TransformListener& tf_listener, std::string from_coord, std::string to_coord)
{
    // tf conversion (0, 0, 0)
    Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
    tf::StampedTransform transform;
    try 
    {
        tf_listener.lookupTransform(from_coord, to_coord, ros::Time(0), transform);
        pcl_ros::transformAsMatrix(transform, transform_matrix);
    } 
    catch (tf::TransformException &ex) 
    {
        ROS_WARN("%s", ex.what());
        return transform_matrix; // Return Identity Matrix
    }
    return transform_matrix;
}

#endif