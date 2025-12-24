cmake_minimum_required(VERSION 3.0.2)
project(my_pcl_pkg)

# 查找catkin和所需组件
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
  pcl_msgs
  livox_ros_driver  # 添加livox驱动
)

# PCL配置
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 包含catkin的头文件目录
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# 创建节点：收集5秒点云数据
add_executable(collect_livox_cloud src/collect_livox_cloud.cpp)
target_link_libraries(collect_livox_cloud 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# 安装目标
install(TARGETS collect_livox_cloud
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)