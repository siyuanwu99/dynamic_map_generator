#include <math.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/ByteMultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <vector>

#define MAP_LENGTH_VOXEL_NUM 66
#define MAP_WIDTH_VOXEL_NUM  66
#define MAP_HEIGHT_VOXEL_NUM 20
#define PREDICTION_TIMES     6
#define VOXEL_RESOLUTION     0.15

constexpr int VOXEL_NUM = MAP_LENGTH_VOXEL_NUM * MAP_WIDTH_VOXEL_NUM * MAP_HEIGHT_VOXEL_NUM;

constexpr double LOCAL_RANGE_X = MAP_LENGTH_VOXEL_NUM / 2.f * VOXEL_RESOLUTION;
constexpr double LOCAL_RANGE_Y = MAP_WIDTH_VOXEL_NUM / 2.f * VOXEL_RESOLUTION;
constexpr double LOCAL_RANGE_Z = MAP_HEIGHT_VOXEL_NUM / 2.f * VOXEL_RESOLUTION;

ros::Publisher pub_cloud_, future_risk_pub_, time_pub_;

ros::Subscriber odom_sub_;
ros::Subscriber global_map_sub_, local_map_sub_;
ros::Timer      local_sensing_timer_;

nav_msgs::Odometry       odom_;
sensor_msgs::PointCloud2 local_map_pcl_;

bool has_odom_;
bool is_updating_map_{false};

double sensing_horizon_, sensing_rate_, estimation_rate_;
double _x_size, _y_size, _z_size;
double _resolution, _inv_resolution;

float local_map_[VOXEL_NUM][PREDICTION_TIMES];

inline bool isInRange(const Eigen::Vector3f& p) {
  return p.x() > -LOCAL_RANGE_X && p.x() < LOCAL_RANGE_X && p.y() > -LOCAL_RANGE_Y &&
         p.y() < LOCAL_RANGE_Y && p.z() > -LOCAL_RANGE_Z && p.z() < LOCAL_RANGE_Z;
}

inline int getIndex(const Eigen::Vector3f& pf) {
  int x = (pf.x() + LOCAL_RANGE_X) / VOXEL_RESOLUTION;
  int y = (pf.y() + LOCAL_RANGE_Y) / VOXEL_RESOLUTION;
  int z = (pf.z() + LOCAL_RANGE_Z) / VOXEL_RESOLUTION;
  return x + y * MAP_LENGTH_VOXEL_NUM + z * MAP_LENGTH_VOXEL_NUM * MAP_WIDTH_VOXEL_NUM;
}

inline Eigen::Vector3f getPointsMapFrame(const int& index) {
  int x = index % MAP_LENGTH_VOXEL_NUM;
  int y = (index / MAP_LENGTH_VOXEL_NUM) % MAP_WIDTH_VOXEL_NUM;
  int z = index / (MAP_LENGTH_VOXEL_NUM * MAP_WIDTH_VOXEL_NUM);
  return Eigen::Vector3f(x * VOXEL_RESOLUTION - LOCAL_RANGE_X, y * VOXEL_RESOLUTION - LOCAL_RANGE_Y,
                         z * VOXEL_RESOLUTION - LOCAL_RANGE_Z);
}

int getIndexGlobalMap(const Eigen::Vector3f& pf) {
  static int x_l = -_x_size / 2;
  static int y_l = -_y_size / 2;
  static int z_l = 0;
  static int x_h = _x_size / 2;
  static int y_h = _y_size / 2;
  static int z_h = _z_size;
  if (pf.x() < x_l || pf.x() > x_h || pf.y() < y_l || pf.y() > y_h || pf.z() < z_l ||
      pf.z() > z_h) {
    return -1;
  } else {
    int x = (pf.x() - x_l) / _resolution;
    int y = (pf.y() - y_l) / _resolution;
    int z = (pf.z() - z_l) / _resolution;
    return x + static_cast<int>(_x_size / _resolution) * y +
           static_cast<int>(_x_size / _resolution) * static_cast<int>(_y_size / _resolution) * z;
  }
}

void rcvOdometryCallbck(const nav_msgs::Odometry& odom) {
  has_odom_ = true;
  odom_     = odom;
}
void rcvGlobalMapCallBack(const std_msgs::ByteMultiArray& global_map) {
  if (!has_odom_) {
    ROS_WARN("[PCL Render] No odom received!");
    return;
  }
  ros::Time t1         = ros::Time::now();
  is_updating_map_     = true;
  int s                = global_map.layout.dim[0].stride;
  int global_voxel_num = global_map.layout.dim[1].size;
  std::cout << "[dbg] s: " << s << " global_voxel_num: " << global_voxel_num << std::endl;

  if (global_voxel_num == 0) {
    // ROS_WARN("Empty global map received!");
    return;
  }

  t1 = ros::Time::now();

  static int   ix  = _x_size / _resolution;
  static int   iy  = _y_size / _resolution;
  static int   iz  = _z_size / _resolution;
  static float x_l = _x_size / 2;
  static float y_l = _y_size / 2;
  static float z_l = _z_size / 2;

  Eigen::Vector3f pose;
  pose.x() = odom_.pose.pose.position.x;
  pose.y() = odom_.pose.pose.position.y;
  pose.z() = odom_.pose.pose.position.z;

  // std::cout << "[dbg] pose: " << pose.x() << " " << pose.y() << " " << pose.z() << std::endl;

  /* get local index lists */
  for (int i = 0; i < VOXEL_NUM; i++) {
    for (int j = 0; j < PREDICTION_TIMES; j++) {
      Eigen::Vector3f p_world = getPointsMapFrame(i) + pose;
      int             index   = getIndexGlobalMap(p_world);
      if (index < 0) {
        local_map_[i][j] = 0.0f;
      } else {
        local_map_[i][j] = static_cast<float>(global_map.data[index * s + j]);
      }
    }
  }

  // ROS_INFO("Clamping map costs %f ms", (ros::Time::now() - t1).toSec() * 1000.0f);

  /* publish point cloud */
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  for (int i = 0; i < VOXEL_NUM; i++) {
    if (local_map_[i][0] > 0.0f) {
      Eigen::Vector3f p = getPointsMapFrame(i);
      pcl::PointXYZ   pt;
      pt.x = p.x() + pose.x();
      pt.y = p.y() + pose.y();
      pt.z = p.z() + pose.z();
      cloud->push_back(pt);
    }
  }
  cloud->width    = cloud->points.size();
  cloud->height   = 1;
  cloud->is_dense = true;
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud, cloud_msg);
  cloud_msg.header.frame_id = "world";
  cloud_msg.header.stamp    = ros::Time::now();
  pub_cloud_.publish(cloud_msg);

  /** publish future risks */
  std_msgs::Float32MultiArray   future_risk_array_msg;
  std_msgs::MultiArrayDimension future_risk_array_dimension;
  future_risk_array_dimension.size   = VOXEL_NUM;
  future_risk_array_dimension.stride = PREDICTION_TIMES;
  future_risk_array_msg.layout.dim.push_back(future_risk_array_dimension);
  future_risk_array_msg.data.reserve(VOXEL_NUM * PREDICTION_TIMES + 4);
  for (int i = 0; i < VOXEL_NUM; ++i) {
    for (int j = 0; j < PREDICTION_TIMES; ++j) {
      future_risk_array_msg.data.push_back(local_map_[i][j]);
    }
  }
  future_risk_array_msg.data.push_back(pose.x());
  future_risk_array_msg.data.push_back(pose.y());
  future_risk_array_msg.data.push_back(pose.z());
  // add current time
  future_risk_array_msg.data.push_back(ros::Time::now().toSec());
  future_risk_pub_.publish(future_risk_array_msg);

  std_msgs::Header time_msg;
  time_msg.stamp = ros::Time::now();
  time_pub_.publish(time_msg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pcl_render");
  ros::NodeHandle nh("~");

  nh.getParam("sensing_horizon", sensing_horizon_);
  nh.getParam("sensing_rate", sensing_rate_);

  nh.getParam("global_map/x_size", _x_size);
  nh.getParam("global_map/y_size", _y_size);
  nh.getParam("global_map/z_size", _z_size);
  nh.getParam("global_map/resolution", _resolution);

  // subscribe point cloud
  global_map_sub_ = nh.subscribe("future_risk", 1, rcvGlobalMapCallBack);
  odom_sub_       = nh.subscribe("odometry", 50, rcvOdometryCallbck);

  // publisher depth image and color image
  pub_cloud_       = nh.advertise<sensor_msgs::PointCloud2>("map/inflated_occupancy", 10);
  future_risk_pub_ = nh.advertise<std_msgs::Float32MultiArray>("map/future_risk", 10);
  time_pub_        = nh.advertise<std_msgs::Header>("map/time", 1, true);

  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }
}
