#include <map_generator/random_dynamic_map.h>

void RandomDynamicMap::init() {
  nh_.param("map/seed", seed_, 0);
  nh_.param("map/future_num", num_future_map_, 6);
  nh_.param("map/time_step", future_step_size_, 0.2F);
  nh_.param("map/x_size", x_size_, 10.0F);
  nh_.param("map/y_size", y_size_, 10.0F);
  nh_.param("map/z_size", z_size_, 5.0F);
  nh_.param("map/test", is_test_mode_, false);
  nh_.param("map/obs_num", num_obstacles_, 30);
  nh_.param("map/resolution", resolution_, 0.1F);
  nh_.param("map/frame_id", frame_id_, std::string("map"));
  nh_.param("sensing/rate", sense_rate_, 10.0F);
  nh_.param("mode", mode_, 0);

  nh_.param("obstacle/upper_vel", v_h_, 0.1F);
  nh_.param("aabb/num", aabb_config_.num, 0);
  nh_.param("aabb/lower_x", aabb_config_.size_x_min, 0.1F);
  nh_.param("aabb/upper_x", aabb_config_.size_x_max, 0.5F);
  nh_.param("aabb/lower_y", aabb_config_.size_y_min, 0.1F);
  nh_.param("aabb/upper_y", aabb_config_.size_y_max, 0.5F);
  nh_.param("aabb/lower_z", aabb_config_.size_z_min, 0.1F);
  nh_.param("aabb/upper_z", aabb_config_.size_z_max, 0.5F);
  nh_.param("cylinder/num", cylinder_config_.num, 0);
  nh_.param("cylinder/lower_rad", cylinder_config_.radius_min, 0.3F);
  nh_.param("cylinder/upper_rad", cylinder_config_.radius_max, 0.8F);
  nh_.param("cylinder/lower_hei", cylinder_config_.height_min, 3.0F);
  nh_.param("cylinder/upper_hei", cylinder_config_.height_max, 4.0F);
  nh_.param("circlegate/num", circlegate_config_.num, 0);
  nh_.param("circlegate/radius_l", circlegate_config_.radius_min, 7.0F);
  nh_.param("circlegate/radius_h", circlegate_config_.radius_max, 7.0F);
  nh_.param("circlegate/thickness", circlegate_config_.thickness_max, 7.0F);
  nh_.param("circlegate/dr", circlegate_config_.alpha_max, 0.2F);
  nh_.param("circlegate/theta", circlegate_config_.theta_max, 2.0F);
  nh_.param("pcd/num", pcdobs_config_.num, 0);  // TODO: pcd -> yaml -> point clouds
  nh_.param("pcd/path", pcdobs_config_.file_name, std::string(""));

  /* initialize random seed */
  if (seed_ == 0) {
    std::random_device rd;
    eng_ = std::default_random_engine(rd());
  } else {
    eng_ = std::default_random_engine(seed_);
  }

  x_l_ = -x_size_ / 2;
  x_h_ = x_size_ / 2;
  y_l_ = -y_size_ / 2;
  y_h_ = y_size_ / 2;
  z_l_ = 0;
  z_h_ = z_size_;

  ros::Duration(2.0).sleep(); /* sleep for 1s to wait for rviz */

  all_map_cloud_pub_  = nh_.advertise<sensor_msgs::PointCloud2>("global_cloud", 1);
  obstacle_vis_pub_   = nh_.advertise<visualization_msgs::MarkerArray>("global_cylinders_vis", 1);
  obstacle_state_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("global_cylinders_state", 1);
  future_map_pub_     = nh_.advertise<std_msgs::ByteMultiArray>("future_risk", 1);

  /* ---- Initialize Obstacles ---- */
  obstacles_.reserve(num_obstacles_);

  /* PCD obstacles */
  for (int i = 0; i < pcdobs_config_.num; i++) {
    obstacles_.emplace_back(std::make_unique<PCDObstacle>(
        sampleRandomPosition(), sampleRandomVelocity2D(), pcdobs_config_.file_name));
  }

  /* AABB */
  for (int i = 0; i < aabb_config_.num; i++) {
    float           size_x = uniformSample(aabb_config_.size_x_max, aabb_config_.size_x_min);
    float           size_y = uniformSample(aabb_config_.size_y_max, aabb_config_.size_y_min);
    float           size_z = uniformSample(aabb_config_.size_z_max, aabb_config_.size_z_min);
    Eigen::Vector3f size(size_x, size_y, size_z);
    obstacles_.emplace_back(
        std::make_unique<AABB>(sampleRandomPosition(), sampleRandomVelocity2D(), size));
    std::cout << "AABB size: " << size.transpose() << std::endl;
  }

  /* Cylinder */
  for (int i = 0; i < cylinder_config_.num; i++) {
    float radius = uniformSample(cylinder_config_.radius_max, cylinder_config_.radius_min);
    float height = cylinder_config_.height_max;

    Eigen::Vector3f p = sampleRandomPosition();
    p.z()             = z_h_ / 2;
    obstacles_.emplace_back(
        std::make_unique<Cylinder>(p, sampleRandomVelocity2D(), radius, height));
    std::cout << "Cylinder radius: " << radius << " height: " << height << std::endl;
  }

  /* Circles */
  for (int i = 0; i < circlegate_config_.num; i++) {
    float radius    = uniformSample(circlegate_config_.radius_max, circlegate_config_.radius_min);
    float thickness = circlegate_config_.thickness_max;
    float alpha     = circlegate_config_.alpha_max;
    float theta     = uniformSample(circlegate_config_.theta_max, -circlegate_config_.theta_max);
    obstacles_.emplace_back(std::make_unique<CircleGate>(
        sampleRandomPosition(), sampleRandomVelocity2D(), radius, thickness, alpha, theta));
    std::cout << "CircleGate radius: " << radius << " thickness: " << thickness
              << " alpha: " << alpha << " theta: " << theta << std::endl;
  }

  // for (int i = pcdobs_config_.num; i < num_obstacles_; i++) {
  //   int obstacle_type = std::rand() % 3;
  //   switch (obstacle_type) {
  //     case 0: /* AABB */
  //     {
  //       float           size_x = uniformSample(aabb_config_.size_x_max,
  //       aabb_config_.size_x_min); float           size_y =
  //       uniformSample(aabb_config_.size_y_max, aabb_config_.size_y_min); float           size_z
  //       = uniformSample(aabb_config_.size_z_max, aabb_config_.size_z_min); Eigen::Vector3f
  //       size(size_x, size_y, size_z); obstacles_.emplace_back(
  //           std::make_unique<AABB>(sampleRandomPosition(), sampleRandomVelocity2D(), size));
  //       std::cout << "AABB size: " << size.transpose() << std::endl;
  //       break;
  //     }
  //     case 1: /* Cylinder */
  //     {
  //       float radius = uniformSample(cylinder_config_.radius_max, cylinder_config_.radius_min);
  //       float height = cylinder_config_.height_max;
  //
  //       Eigen::Vector3f p = sampleRandomPosition();
  //       p.z()             = z_h_ / 2;
  //       obstacles_.emplace_back(
  //           std::make_unique<Cylinder>(p, sampleRandomVelocity2D(), radius, height));
  //       std::cout << "Cylinder radius: " << radius << " height: " << height << std::endl;
  //       break;
  //     }
  //     case 2: /* Circle Gate */
  //     {
  //       float radius = uniformSample(circlegate_config_.radius_max,
  //       circlegate_config_.radius_min); float thickness = circlegate_config_.thickness_max;
  //       float alpha     = circlegate_config_.alpha_max; float theta =
  //       uniformSample(circlegate_config_.theta_max, -circlegate_config_.theta_max);
  //       obstacles_.emplace_back(std::make_unique<CircleGate>(
  //           sampleRandomPosition(), sampleRandomVelocity2D(), radius, thickness, alpha,
  //           theta));
  //       std::cout << "CircleGate radius: " << radius << " thickness: " << thickness
  //                 << " alpha: " << alpha << " theta: " << theta << std::endl;
  //       break;
  //     }
  //   }
  // }
}

/* render obstacles */
void RandomDynamicMap::renderMap() {
  /* render obstacles */
  ROS_INFO("Rendering map...");
  ros::Time t0 = ros::Time::now();
  for (auto& obs : obstacles_) {
    ros::Time t1 = ros::Time::now();
    obs->render(resolution_);
    std::cout << "Obstacle render cost: " << ros::Time::now() - t1 << std::endl;
  }
  ROS_INFO("Map rendered, cost: %f", (ros::Time::now() - t0).toSec());
  is_rendered_ = true;
}

void RandomDynamicMap::publishMap() {
  if (!is_rendered_) {
    ROS_WARN("Map is not rendered yet!");
    return;
  }
  float                               dt = 1 / sense_rate_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  for (auto& obs : obstacles_) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_obs(new pcl::PointCloud<pcl::PointXYZ>);
    obs->update(dt, x_l_, x_h_, y_l_, y_h_, z_l_, z_h_);
    obs->getCloud(cloud_obs);
    *cloud += *cloud_obs;
  }
  cloud->width    = cloud->points.size();
  cloud->height   = 1;
  cloud->is_dense = true;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  for (auto& p : cloud->points) {
    pcl::PointXYZRGB p_rgb;
    p_rgb.x               = p.x;
    p_rgb.y               = p.y;
    p_rgb.z               = p.z;
    Eigen::Vector3i color = getMagmaColor(p.z, -3, 5);
    p_rgb.r               = color.x();
    p_rgb.g               = color.y();
    p_rgb.b               = color.z();
    colored_cloud->push_back(p_rgb);
  }

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*colored_cloud, cloud_msg);
  cloud_msg.header.frame_id = frame_id_;
  cloud_msg.header.stamp    = ros::Time::now();
  all_map_cloud_pub_.publish(cloud_msg);
}

/**
 * @brief publish the visualization of the obstacles
 *
 * @NOTE: please make sure the obstacles are updated before calling this function
 */
void RandomDynamicMap::publishObstacleState() {
  if (!is_rendered_) {
    ROS_WARN("Map is not rendered yet!");
    return;
  }
  visualization_msgs::MarkerArray obstacle_state_list;
  obstacle_state_list.markers.reserve(obstacles_.size());

  visualization_msgs::Marker obstacle_state;
  obstacle_state.header.stamp    = ros::Time::now();
  obstacle_state.header.frame_id = frame_id_;
  obstacle_state.type            = visualization_msgs::Marker::CUBE;
  obstacle_state.points.clear();
  obstacle_state.id = 0;

  for (auto& obs : obstacles_) {
    Eigen::Vector3f p, v, s;
    p                    = obs->getPosition();
    v                    = obs->getVelocity();
    s                    = obs->getBBox();
    Eigen::Quaternionf q = obs->getQuaternion();

    obstacle_state.pose.position.x    = p.x();
    obstacle_state.pose.position.y    = p.y();
    obstacle_state.pose.position.z    = p.z();
    obstacle_state.pose.orientation.x = q.x();
    obstacle_state.pose.orientation.y = q.y();
    obstacle_state.pose.orientation.z = q.z();
    obstacle_state.pose.orientation.w = q.w();
    obstacle_state.color.a            = 0.6;
    obstacle_state.color.r            = 0.5;
    obstacle_state.color.g            = 0.5;
    obstacle_state.color.b            = 0.5;

    obstacle_state.points.clear();
    geometry_msgs::Point pts;
    pts.x = p.x();
    pts.y = p.y();
    pts.z = p.z();
    obstacle_state.points.push_back(pts);
    pts.x = p.x() + v.x();
    pts.y = p.y() + v.y();
    pts.z = p.z() + v.z();
    obstacle_state.points.push_back(pts);
    obstacle_state.scale.x = s.x();
    obstacle_state.scale.y = s.y();
    obstacle_state.scale.z = s.z();
    obstacle_state_list.markers.push_back(obstacle_state);
    obstacle_state.id++;
  }
  obstacle_state_pub_.publish(obstacle_state_list);
}

void RandomDynamicMap::publishFuturePrediction() {
  std_msgs::ByteMultiArray      future_risk_array_msg;
  std_msgs::MultiArrayDimension future_risk_array_dimension;
  static int                    VOXEL_NUM =
      (x_size_ / resolution_) * (y_size_ / resolution_) * (z_size_ / resolution_);
  future_risk_array_dimension.size   = VOXEL_NUM;
  future_risk_array_dimension.stride = num_future_map_;
  future_risk_array_msg.layout.dim.push_back(future_risk_array_dimension);
  future_risk_array_msg.data.reserve(VOXEL_NUM * num_future_map_);

  std::vector<std::vector<int>> occupany;
  occupany.resize(num_future_map_);
  // std::cout << "[dbg] allocation safe" << std::endl;
  for (int i = 0; i < num_future_map_; i++) {
    float t = i * future_step_size_;
    occupany[i].resize(VOXEL_NUM, 0.0f);

    /* predict occupany status at time t */
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (auto& obs : obstacles_) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_obs(new pcl::PointCloud<pcl::PointXYZ>);
      obs->getPredictedCloud(t, cloud_obs);
      *cloud += *cloud_obs;
    }
    // std::cout << "[dbg] predict safe" << std::endl;

    /* push to new container */
    for (auto p : cloud->points) {
      float x = p.x, y = p.y, z = p.z;
      if (x < x_l_ || x > x_h_ || y < y_l_ || y > y_h_ || z < z_l_ || z > z_h_) continue;
      int ix = (x - x_l_) / resolution_, iy = (y - y_l_) / resolution_,
          iz = (z - z_l_) / resolution_;
      int idx =
          ix + iy * static_cast<int>(x_size_ / resolution_) +
          iz * static_cast<int>(x_size_ / resolution_) * static_cast<int>(y_size_ / resolution_);
      occupany[i][idx] = 1;
    }
    // std::cout << "[dbg] push safe" << std::endl;
  }

  for (int j = 0; j < VOXEL_NUM; j++) {
    for (int i = 0; i < num_future_map_; i++) {
      future_risk_array_msg.data.push_back(occupany[i][j]);
    }
  }

  future_map_pub_.publish(future_risk_array_msg);
}

Eigen::Vector3f RandomDynamicMap::sampleRandomPosition() {
  std::uniform_real_distribution<float> rand_x(x_l_, x_h_);
  std::uniform_real_distribution<float> rand_y(y_l_, y_h_);
  std::uniform_real_distribution<float> rand_z(z_l_, z_h_);
  return Eigen::Vector3f(rand_x(eng_), rand_y(eng_), rand_z(eng_));
}

Eigen::Vector3f RandomDynamicMap::sampleRandomVelocity2D() {
  std::uniform_real_distribution<float> rand_v(-v_h_, v_h_);
  std::uniform_real_distribution<float> rand_omega(-M_PI, M_PI);
  return Eigen::Vector3f(rand_v(eng_) * std::cos(rand_omega(eng_)),
                         rand_v(eng_) * std::sin(rand_omega(eng_)), 0.0F);
}
