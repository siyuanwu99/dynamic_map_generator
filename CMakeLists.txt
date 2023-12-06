cmake_minimum_required(VERSION 2.8.3)
project(map_generator)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
ADD_COMPILE_OPTIONS(-std=c++11)
ADD_COMPILE_OPTIONS(-std=c++14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  pcl_conversions
)
find_package(PCL 1.7 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(VTK REQUIRED)  # for pcd generation

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_library(${PROJECT_NAME}
  src/random_dynamic_map.cpp
)

add_executable(random_env src/random_env_sensing.cpp)
target_link_libraries(random_env
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})

add_executable(
    pcl_render_node
    src/pointcloud_render_node.cpp
)

target_link_libraries( pcl_render_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(generate_letters src/generate_letters_point_cloud.cpp)
target_link_libraries(generate_letters
  ${PCL_LIBRARIES}
  ${VTK_LIBRARIES}
)


add_executable(random_forest src/random_forest_sensing.cpp)
target_link_libraries(random_forest
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})

add_executable(dynamic_forest src/dynamic_forest_sensing.cpp)
target_link_libraries(dynamic_forest
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})

add_executable(dynamic_forest_seq src/dynamic_forest_seq_sensing.cpp)
target_link_libraries(dynamic_forest_seq
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES})


add_executable(click_map src/click_map.cpp)
target_link_libraries(click_map
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})

install(TARGETS random_forest click_map
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
