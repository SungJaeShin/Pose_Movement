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

std::vector<double> parseCSV(std::istream &file){
	std::vector<double> parse;
	std::string line;

	double val;

	while(std::getline(file, line, ',')){
		std::stringstream s_line(line);

		while(s_line >> val){
			parse.push_back(val);

			if(s_line.peek() == ',') s_line.ignore();
		}
	}

	return parse;
}

geometry_msgs::PoseStamped get_pose(double x, double y, double z, double q_w, double q_x, double q_y, double q_z){
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

int main(int argc, char **argv){
  // To Make extracting values from CSV file!
  std::ifstream file;
  file.open("/home/<user_name>/catkin_ws/src/pose_movement/src/pose_seq.csv");

  std::vector<double> result = parseCSV(file);

  int index = 0;
  double array[7];

  nav_msgs::Path pose1;

  ros::init(argc, argv, "pose_movement");
  ros::NodeHandle nh;
  ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 100);
  ros::Publisher pose_track_pub = nh.advertise<nav_msgs::Path>("tracking", 100);
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 100);
  ros::Rate loop_rate(10);

  auto result_address = result.begin();

  while(ros::ok()){


    if(index == 7){
      // Initialize PoseStamped parameter!
      index = 0;
      std::cout << "Index Initialize" << std::endl;

      geometry_msgs::PoseStamped pose_track = get_pose(array[0], array[1], array[2], array[3], array[4], array[5], array[6]);
      pose_track.header.frame_id = "base_link";
      pose_track.header.stamp = ros::Time::now();

      pose1.header.stamp = ros::Time::now();
      pose1.header.frame_id = "base_link";
      pose1.poses.push_back(pose_track);

      nav_msgs::Odometry odom;
      odom.header.frame_id = "base_link";
      odom.header.stamp = ros::Time::now();
      odom.pose.pose = pose_track.pose;

      pose_pub.publish(pose_track);
      pose_track_pub.publish(pose1);
      odom_pub.publish(odom);
    }

      if(result_address == result.end()){
        std::cout<<"breaking?"<<std::endl;
        break;
      }

	  array[index] = *result_address;

		result_address ++;
		index ++;

    std::cout << " new_index : " << index << std::endl;

    loop_rate.sleep();
  }

	file.close();

	return 0;
}
