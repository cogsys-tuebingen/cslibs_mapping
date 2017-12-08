cmake_minimum_required(VERSION 2.8.3)
project(cslibs_mapping)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cslibs_gridmaps
  cslibs_time
  cslibs_utility
  cslibs_math_ros
  cslibs_ndt_3d
  cslibs_ndt_2d
  nav_msgs
  roscpp
  pcl_ros
)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation io)

catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES muse_mcl_2d_mapping
  #CATKIN_DEPENDS cslibs_ndt_3d muse_mcl_2d_gridmaps muse_mcl_2d_ndt
  #DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${pcl_ros_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_library(${PROJECT_NAME}_2d
    src/mapper/occupancy_grid_mapper_2d.cpp
    src/mapper/ndt_grid_mapper_2d.cpp
)
target_link_libraries(${PROJECT_NAME}_2d
    ${catkin_LIBRARIES}
)

add_library(${PROJECT_NAME}_3d
    src/mapper/ndt_grid_mapper_3d.cpp
)
target_link_libraries(${PROJECT_NAME}_3d
    ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node_2d
    src/nodes/mapper_node_2d.cpp
)
target_link_libraries(${PROJECT_NAME}_node_2d
    ${PROJECT_NAME}_2d
    ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node_3d
    src/nodes/mapper_node_3d.cpp
)
target_link_libraries(${PROJECT_NAME}_node_3d
    ${PROJECT_NAME}_2d
    ${PROJECT_NAME}_3d
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${PCL_COMMON_LIBRARIES}
    ${PCL_FILTERS_LIBRARIES}
    ${PCL_SEGMENTATION_LIBRARIES}
    ${PCL_IO_LIBRARIES}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
