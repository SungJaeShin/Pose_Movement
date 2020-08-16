# Pose Tracking 
## pose의 정보가 담겨있는 .csv file을 이용하여 rviz상으로 움직임 파악하기.

### csv file 
[example] 
- 498.00488,-436.80432,-4.7894874,0.55949587,-0.46982133,0.4388751,-0.52308786 <br>
- 차례대로 다음과 같은 값들을 의미한다.
  * position x = 498.00488
  * position y = -436.80432
  * position z = -4.7894874
  * orientation w = 0.55949587
  * orientation x = -0.46982133
  * orientation y = 0.4388751
  * orientation z = -0.52308786


<br>
### publish topic 
<br>
- "pose" topic 
  * message type : geometry_msgs::PoseStamped <br>
- "tracking" topic
  * message type : nav_msgs::Path <br>
- "odom" topic
  * message type : nav_msgs::Odometry <br>


<br>
### Parameter explanation
- std::ifstream file &#8658; 나의 Computer에 있는 CSV file을 받을 변수 <br>
- std::vector<double> result &#8658;


<br>
### overall code explanation 
- function 1 &#8658; std::vector<double> parseCSV(std::istream &file)
  * CSV file을 받아서 각 방들이 double type을 가지는 std::vector로 넣어준다.
  * 이 경우, comma(,)의 값을 빼고 한 줄에 7개의 Value들이 들어가도록 설정되어 있다.

<br>

- function 2 &#8658; geometry_msgs::PoseStamped get_pose(double x, double y, double z, double q_w, double q_x, double q_y, double q_z)
  * double type의 방을 가진 std::vector의 모든 값을 geometry_msgs::PoseStamped로 바꿔준다.
  * 이 경우, 한 줄씩 받아서 while문 안의 __pose_track__에 넣어준다. 

<br>



