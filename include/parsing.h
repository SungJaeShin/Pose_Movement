#include "include.h"

Eigen::Affine3f getTransformFromFirstPose(const std::vector<double>& path, const std::string& flag) {
    Eigen::Vector3f first_pose_translation;
    Eigen::Quaternionf first_pose_orientation;


    if (flag == "WITHOUT_TIME" && path.size() >= 7) 
    {
        first_pose_translation = Eigen::Vector3f(path[0], path[1], path[2]);
        first_pose_orientation = Eigen::Quaternionf(path[3], path[4], path[5], path[6]);
    } 
    else if (flag == "WITH_TIME" && path.size() >= 8) 
    {
        first_pose_translation = Eigen::Vector3f(path[1], path[2], path[3]);
        first_pose_orientation = Eigen::Quaternionf(path[7], path[4], path[5], path[6]);
    } 
    else 
        throw std::runtime_error("Invalid path data or flag");

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() = first_pose_translation - Eigen::Vector3f(1.5, 1.0, 0.0);
    transform.rotate(first_pose_orientation); 
    
    return transform;
}

geometry_msgs::Pose set_pose(double x, double y, double z, double q_w, double q_x, double q_y, double q_z)
{
    geometry_msgs::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;

    pose.orientation.x = q_x;
    pose.orientation.y = q_y;
    pose.orientation.z = q_z;
    pose.orientation.w = q_w;

    return pose;
}

// Without timestamp 
geometry_msgs::PoseStamped get_pose(double x, double y, double z, double q_w, double q_x, double q_y, double q_z)
{
    geometry_msgs::PoseStamped posestamp;
    posestamp.pose = set_pose(x, y, z, q_w, q_x, q_y, q_z);
    return posestamp;
}

// With timestamp (add time)
geometry_msgs::PoseStamped get_pose(double time, double x, double y, double z, double q_x, double q_y, double q_z, double q_w)
{
    geometry_msgs::PoseStamped posestamp;
    posestamp.header.stamp = ros::Time().fromSec(time);
    posestamp.header.frame_id = "map";
    posestamp.pose = set_pose(x, y, z, q_w, q_x, q_y, q_z);
    return posestamp;
}