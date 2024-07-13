#include "include.h"

// Without timestamp 
geometry_msgs::PoseStamped get_pose(double x, double y, double z, double q_w, double q_x, double q_y, double q_z)
{
    geometry_msgs::PoseStamped posestamp;

    posestamp.pose.position.x = x;
    posestamp.pose.position.y = y;
    posestamp.pose.position.z = z;

    posestamp.pose.orientation.x = q_x;
    posestamp.pose.orientation.y = q_y;
    posestamp.pose.orientation.z = q_z;
    posestamp.pose.orientation.w = q_w;

    return posestamp;
}

// With timestamp (add time)
geometry_msgs::PoseStamped get_pose(double time, double x, double y, double z, double q_x, double q_y, double q_z, double q_w)
{
    geometry_msgs::PoseStamped posestamp;

    ros::Time ros_time;
    ros_time.fromSec(time);

    posestamp.header.stamp = ros_time;
    posestamp.header.frame_id = "map";

    posestamp.pose.position.x = x;
    posestamp.pose.position.y = y;
    posestamp.pose.position.z = z;

    posestamp.pose.orientation.x = q_x;
    posestamp.pose.orientation.y = q_y;
    posestamp.pose.orientation.z = q_z;
    posestamp.pose.orientation.w = q_w;

    return posestamp;
}