#include "include.h"

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