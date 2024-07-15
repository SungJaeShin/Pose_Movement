#include "read_files.h"
#include "parsing.h"

void PubPathatOnce(ros::NodeHandle nh, std::vector<double> path, std::string flag)
{
    std::vector<double> tmp_path(path);
    std::cout << "full path size: " << tmp_path.size() / 8 << std::endl;

    int seq = 0;
    int cur_idx = 0;
    nav_msgs::Path full_path;
    full_path.header.stamp = ros::Time::now();
    full_path.header.frame_id = "map";
    full_path.header.seq = 0;

    ros::Publisher pub_full_path = nh.advertise<nav_msgs::Path>("full_trajectory", 10000, true);

    std::cout << "\033[1;34m ===> Get Full Path at Once !! \033[0m" << std::endl;
    while(cur_idx < tmp_path.size())
    {
        // Initialize PoseStamped parameter!
        geometry_msgs::PoseStamped cur_pose;
        if(flag == "WITHOUT_TIME" && cur_idx % 7 == 0 && cur_idx + 7 <= tmp_path.size())
        {
            cur_pose = get_pose(tmp_path[cur_idx], tmp_path[cur_idx + 1], tmp_path[cur_idx + 2], tmp_path[cur_idx + 3], 
                                                   tmp_path[cur_idx + 4], tmp_path[cur_idx + 5], tmp_path[cur_idx + 6]);
            cur_pose.header.stamp = ros::Time::now();
            cur_pose.header.frame_id = "map";
            cur_pose.header.seq = seq++;

            full_path.poses.push_back(cur_pose);
            cur_idx += 7;
        }
        else if(flag == "WITH_TIME" && cur_idx % 8 == 0 && cur_idx + 8 <= tmp_path.size())
        {
            cur_pose = get_pose(tmp_path[cur_idx],     tmp_path[cur_idx + 1], tmp_path[cur_idx + 2], tmp_path[cur_idx + 3], 
                                tmp_path[cur_idx + 4], tmp_path[cur_idx + 5], tmp_path[cur_idx + 6], tmp_path[cur_idx + 7]);
            cur_pose.header.seq = seq++;
            
            full_path.poses.push_back(cur_pose);
            cur_idx += 8;
        }
        else
            break;

        // if(cur_idx % 7 == 0 || cur_idx % 8 == 0)
        // {
            
        // }


        // === For Debugging ===
        // if(cur_idx % 7 == 0 || cur_idx % 8 == 0)
        //     std::cout << "=== cur pose === \n" << cur_pose.pose << std::endl;
    }        
    
    pub_full_path.publish(full_path);

}

int main(int argc, char **argv)
{
    // Get path info
    if(argc != 3)
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
        std::cout << "\033[1;33m ===> Pose file without Time ! \033[0m" << std::endl;
        result = parseCSVfile(file);
    }
    else
    {
        std::cout << "\033[1;33m ===> Pose file with Time ! \033[0m" << std::endl;
        result = parseTXTfile(file);
    }
    
    std::cout << "result size: " << result.size() / 8 << std::endl;

    int index = 0;
    std::vector<double> array;
    
    int seq = 0;
    geometry_msgs::PoseStamped pose_track;
    nav_msgs::Path path;
    nav_msgs::Odometry odom;

    ros::init(argc, argv, "pose_movement");
    ros::NodeHandle nh;
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 1000);
    ros::Publisher pose_track_pub = nh.advertise<nav_msgs::Path>("tracking", 1000);
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 1000);
    ros::Rate loop_rate(100);

    // If you want to publish full trajectory at once, then using this function !
    bool useFullPath = false;
    std::string full_path_once = argv[2];
    if(full_path_once == "true")
    {
        useFullPath = true;
        PubPathatOnce(nh, result, flag);
    }

    auto result_address = result.begin();
    
    while(ros::ok())
    {
        if(flag == "WITHOUT_TIME" && index == 7 && !useFullPath)
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
      
        if(flag == "WITH_TIME" && index == 8 && !useFullPath)
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

        // std::cout << " new_index : " << index << std::endl;

        loop_rate.sleep();
    }

    file.close();

    return 0;
}
