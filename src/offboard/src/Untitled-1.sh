//常用语句
sudo gedit ~/.bashrc
source ~/.bashrc
gedit ~/.bashrc

cd uav_test_ws
catkin_make
rosrun offboard offboard_node

rostopic echo /mavros/local_position/pose
 

listener actuator_controls_0

rostopic echo /mavros/ActuatorControl
    
rostopic echo /mavros/imu/data

rostopic echo /mavros/local_position/pose
roslaunch px4 mavros_posix_sitl.launch


cd uav_test_ws/src/offboard/src
gedit offboard_node.cpp
修改保存cpp
cd ..
gedit CMakeLists.txt

rostopic info
rosmsg show

cd uav_test_ps
catkin_make

source devel/setup.bash
rosrun offboard offboard_node
rosrun rqt_console rqt_console


rosrun rqt_plot rqt_plot /mavros/local_position/pose/pose/position/x:y:z


  void recordRosbag(const std::vector<std::string>& topics, const std::string& bag_filename, double duration = 10) {
  // 创建ROS节点
     ros::NodeHandle nh;
 
    // 创建ROSbag对象
    rosbag::Bag bag;
    bag.open(bag_filename, rosbag::bagmode::Write);
 
  // 创建ROS话题订阅器
    std::vector<ros::Subscriber> subs;
    for (const auto& topic : topics) {
      if (topic == "/error_msg") {
        subs.push_back(nh.subscribe<sensor_msgs::Image>(topic, 1, [&](const sensor_msgs::Image::ConstPtr& msg) {
          bag.write(topic, ros::Time::now(), *msg);
        }));
      }
      else if (topic == "/scan") {
        subs.push_back(nh.subscribe<sensor_msgs::LaserScan>(topic, 1, [&](const sensor_msgs::LaserScan::ConstPtr& msg) {
          bag.write(topic, ros::Time::now(), *msg);
        }));
      }
      else if (topic == "/string") {
        subs.push_back(nh.subscribe<std_msgs::String>(topic, 1, [&](const std_msgs::String::ConstPtr& msg) {
          bag.write(topic, ros::Time::now(), *msg);
        }));
      }
      else {
        ROS_WARN_STREAM("Unknown topic: " << topic);
      }
    }
 
  // 持续录制数据
  if (duration <= 0) {
    ROS_INFO_STREAM("Start recording indefinitely...");
    ros::spin();
  }
  // 持续录制一段时间后停止
  else {
    ROS_INFO_STREAM("Start recording for " << duration << " seconds...");
    ros::Time start_time = ros::Time::now();
    while ((ros::Time::now() - start_time).toSec() < duration) {
      ros::spinOnce();
    }
  }
 
  // 停止订阅器并关闭ROSbag文件
  for (auto& sub : subs) {
    sub.shutdown();
  }
  bag.close();
 
  ROS_INFO_STREAM("Recording finished!");
}