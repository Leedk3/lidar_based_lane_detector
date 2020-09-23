#include <lidar_based_lane_detector/lidar_based_lane_detector.h>

// Constructor
LaneDetector::LaneDetector(ros::NodeHandle nh, ros::NodeHandle private_nh) : bNewOriginCloud(false)
{
  ROS_INFO("Lidar based lane detector node");
  private_nh_.param("ego_shape_front", m_ego_shape_front, double(2));
  private_nh_.param("ego_shape_rear" , m_ego_shape_rear,  double(5));
  private_nh_.param("ego_shape_side",  m_ego_shape_side,  double(1.5));
  private_nh_.param("groud_height_threshold",  m_groud_height_threshold,  double(1.0));


  PubGroundAreaFilterd = nh_.advertise<sensor_msgs::PointCloud2>("/PointCloud2/LaneDetector/GroundFiltered", 1, true);  
  SubLaserCloud = nh_.subscribe("/merged/velodyne_points", 1, &LaneDetector::CallbackLaserCloud, this);

  init();
}

// Destructor
LaneDetector::~LaneDetector(){}

void LaneDetector::init()
{
  m_originCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
  m_groudFilteredCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
}

void LaneDetector::run()
{
  if(bNewOriginCloud)
  {
    GroudAreaFiltering();
    calculateCloudRange();
    calculateSmoothness();
  }
}

void LaneDetector::CallbackLaserCloud(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  // timeNewOutlierCloud = msg->header.stamp.toSec();
  m_originPointMsg_ptr = std::make_shared<sensor_msgs::PointCloud2>(*msg);
  m_originCloud->clear();
  pcl::fromROSMsg(*msg, *m_originCloud);
  bNewOriginCloud = true;
}

void LaneDetector::GroudAreaFiltering()
{
  pcl::PointIndices::Ptr far_indices (new pcl::PointIndices);
  m_groudFilteredCloud -> clear();
  int cloudSize = m_originCloud->points.size();
  for (int i = 0; i < cloudSize; ++i)
  {
    pcl::PointXYZI point;
    point.x = m_originCloud->points[i].x;
    point.y = m_originCloud->points[i].y;
    point.z = m_originCloud->points[i].z;
    
    if(point.z > m_groud_height_threshold)
    {
      far_indices->indices.push_back(i);
      continue;
    }
    
    if (point.y < m_ego_shape_side && point.y > -1.0 * m_ego_shape_side)
    {
      if (point.x < (m_ego_shape_front) && point.x > -1.0 * m_ego_shape_rear)
      {
        far_indices->indices.push_back(i);
      }
    }
  }
  m_groudFilteredCloud->points.clear();
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud (m_originCloud);
  extract.setIndices(far_indices);
  extract.setNegative(true);//true removes the indices, false leaves only the indices
  extract.filter(*m_groudFilteredCloud);
  
  sensor_msgs::PointCloud2 CloudMsg;
  pcl::toROSMsg(*m_groudFilteredCloud, CloudMsg);
  CloudMsg.header=m_originPointMsg_ptr->header;
  PubGroundAreaFilterd.publish(CloudMsg);
}

void LaneDetector::calculateCloudRange()
{
  int cloudSize = m_groudFilteredCloud->points.size();
  m_CloudRange = new float[cloudSize];
  m_CloudIntensity = new float[cloudSize];
  for (int i = 0; i < cloudSize; ++i)
  {
    pcl::PointXYZI point;
    point.x = m_groudFilteredCloud->points[i].x;
    point.y = m_groudFilteredCloud->points[i].y;
    point.z = m_groudFilteredCloud->points[i].z;
    double range = sqrt(pow(point.x, 2)+ pow(point.y, 2)+ pow(point.z, 2));
    m_CloudRange[i] = range;
    m_CloudIntensity[i] = m_groudFilteredCloud->points[i].intensity;
  }  
}

void LaneDetector::calculateSmoothness()
{
  if(!m_CloudRange)
  {
    ROS_WARN("Cloud range set is empty.");
    return;
  }
  int cloudSize = m_groudFilteredCloud->points.size();
  for (int i = 5; i < cloudSize - 5; i++) {

      float diffRange = m_CloudRange[i-5] + m_CloudRange[i-4]
                      + m_CloudRange[i-3] + m_CloudRange[i-2]
                      + m_CloudRange[i-1] - m_CloudRange[i] * 10
                      + m_CloudRange[i+1] + m_CloudRange[i+2]
                      + m_CloudRange[i+3] + m_CloudRange[i+4]
                      + m_CloudRange[i+5];
                                  
      std::cout << diffRange << std::endl;

      // cloudCurvature[i] = diffRange*diffRange;

      // cloudNeighborPicked[i] = 0;
      // cloudLabel[i] = 0;

      // cloudSmoothness[i].value = cloudCurvature[i];
      // cloudSmoothness[i].ind = i;
  }
}



