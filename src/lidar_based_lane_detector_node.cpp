// ROS Includes
#include <ros/ros.h>

// User defined includes
#include <lidar_based_lane_detector/lidar_based_lane_detector.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lidar_based_lane_detector");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;
  LaneDetector detector(nh, private_nh);
  ros::Rate loop_rate(20);


  while(ros::ok())
  {
    detector.run();
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}
