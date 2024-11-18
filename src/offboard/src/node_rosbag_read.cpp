// /*  
//     读取 bag 文件：

// */
// #include "ros/ros.h"
// #include "rosbag/bag.h"
// #include "rosbag/view.h"
// #include "std_msgs/String.h"
// #include "std_msgs/Int32.h"
// #include "my_topic/error_msg.h"

// int main(int argc, char *argv[])
// {

//     setlocale(LC_ALL,"");

//     ros::init(argc,argv,"bag_read");
//     ros::NodeHandle nh;

//     //创建 bag 对象
//     rosbag::Bag bag;
//     //打开 bag 文件
//     bag.open("/home/z017/rosbag/error.bag",rosbag::BagMode::Read);
//     //读数据
//     for (rosbag::MessageInstance const m : rosbag::View(bag))
//     {
//         my_topic::error_msg::ConstPtr error = m.instantiate<my_topic::error_msg>();
//         if(p != nullptr){

//          ROS_INFO_STREAM("error message: " << error->stamp);
//         }
//     }

//     //关闭文件流
//     bag.close();
//     return 0;
// }


