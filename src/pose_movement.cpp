#include "read_files.h"
#include "parsing.h"

int main(int argc, char **argv)
{
  // Get path info
  if(argc != 2)
  {
    printf("please intput: rosrun pose_movement pose_movement [pose path] \n");
    return 0;
  }

  // To Make extracting values from CSV or txt file!
  std::string pose_path = argv[1];
  size_t dotPosition = pose_path.find_last_of(".");
  if (dotPosition == std::string::npos) 
    return 0;
  // Get extension of file
  std::string extension = pose_path.substr(dotPosition + 1);
  std::cout << "\033[1;32m === extension: " << extension << " ===\033[0m" << std::endl;

  std::ifstream file;
  file.open(pose_path);

  // Set Flag following file extension !!
  std::string flag;
  if(extension == "csv")
    flag = "WITHOUT_TIME";
  else 
    flag = "WITH_TIME";

  // Get Pose from file !!
  std::vector<double> result;  
  if(flag == "WITHOUT_TIME")
  {
    std::cout << "\033[1;33m Pose file without Time ! \033[0m" << std::endl;
    result = parseCSVfile(file);
  }
  else
  {
    std::cout << "\033[1;33m Pose file with Time ! \033[0m" << std::endl;
    result = parseTXTfile(file);
  }
  
  int index = 0;
  std::vector<double> array;
  
  int seq = 0;
  geometry_msgs::PoseStamped pose_track;
  nav_msgs::Path path;
  nav_msgs::Odometry odom;


  ros::init(argc, argv, "pose_movement");
  ros::NodeHandle nh;
  ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 100);
  ros::Publisher pose_track_pub = nh.advertise<nav_msgs::Path>("tracking", 100);
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 100);
  ros::Rate loop_rate(10);

  auto result_address = result.begin();
  
  while(ros::ok())
  {
    if(flag == "WITHOUT_TIME" && index == 7)
    {
      // Initialize PoseStamped parameter!
      index = 0;
      std::cout << "Index Initialize" << std::endl;
      std::cout << array[0] << ", " << array[1] << ", " << array[2] << ", " << array[3] << ", " << array[4] << ", " << array[5] << ", " << array[6] << std::endl;

      pose_track = get_pose(array[0], array[1], array[2], array[3], array[4], array[5], array[6]);
      pose_track.header.frame_id = "map";
      pose_track.header.stamp = ros::Time::now();
      pose_track.header.seq = seq++;

      path.header.stamp = ros::Time::now();
      path.header.frame_id = "map";
      path.poses.push_back(pose_track);

      odom.header.frame_id = "map";
      odom.header.stamp = ros::Time::now();
      odom.pose.pose = pose_track.pose;

      pose_pub.publish(pose_track);
      pose_track_pub.publish(path);
      odom_pub.publish(odom);

      array.clear();
    }
   
    if(flag == "WITH_TIME" && index == 8)
    {
      // Initialize PoseStamped parameter!
      index = 0;
      std::cout << "Index Initialize" << std::endl;
      std::cout << array[0] << ", " << array[1] << ", " << array[2] << ", " << array[3] << ", " << array[4] << ", " << array[5] << ", " << array[6]  << ", " << array[7] << std::endl;

      pose_track = get_pose(array[0], array[1], array[2], array[3], array[4], array[5], array[6], array[7]);
      pose_track.header.seq = seq++;

      path.header = pose_track.header;
      path.poses.push_back(pose_track);
      
      odom.header = pose_track.header;
      odom.pose.pose = pose_track.pose;

      pose_pub.publish(pose_track);
      pose_track_pub.publish(path);
      odom_pub.publish(odom);

      array.clear();
    }

    if(result_address == result.end())
    {
      std::cout<<"breaking?"<<std::endl;
      break;
    }

    array.push_back(*result_address);
    result_address ++;
    index ++;

    std::cout << " new_index : " << index << std::endl;

    loop_rate.sleep();
  }

  file.close();

  return 0;
}
