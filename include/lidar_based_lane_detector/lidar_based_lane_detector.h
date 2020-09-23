#ifndef LIDAR_BASED_LANE_DETECTOR_H
#define LIDAR_BASED_LANE_DETECTOR_H

// ROS includes
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>

#include <opencv/cv.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

// C++ includes 
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

class LaneDetector
{
public:
  LaneDetector(ros::NodeHandle nh, ros::NodeHandle private_nh);
  ~LaneDetector();
  void run();
  void CallbackLaserCloud(const sensor_msgs::PointCloud2ConstPtr &msg);
  void GroudAreaFiltering();

private:
  void init();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Subscriber SubLaserCloud;
  ros::Publisher  PubGroundAreaFilterd;

  std::shared_ptr<sensor_msgs::PointCloud2> m_originPointMsg_ptr;
  pcl::PointCloud<pcl::PointXYZI>::Ptr m_originCloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr m_groudFilteredCloud;
  // pcl::PointCloud<pcl::PointXYZI>::Ptr outlierCloud;

  bool bNewOriginCloud;

  double m_ego_shape_front;
  double m_ego_shape_rear;
  double m_ego_shape_side;
  double m_groud_height_threshold;


};
#endif  // LIDAR_BASED_LANE_DETECTOR_H
