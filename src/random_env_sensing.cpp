/**
 * @file random_env_sensing.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief generate a dynamic 3D map
 * To generate the current global map and local map in next time steps,
 * we need to publish current global point cloud and velocity

 *
 * @version 1.0
 * @date 2022-11-20
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <map_generator/random_dynamic_map.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "random_dynamic_map");
  ros::NodeHandle nh("~");

  RandomDynamicMap map(nh);
  map.init();
  map.renderMap();

  ros::Rate rate(map.getSenseRate());
  while (ros::ok()) {
    map.publishMap();
    map.publishObstacleState();
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
