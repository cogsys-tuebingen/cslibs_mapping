cmake_minimum_required(VERSION 2.8.3)
project(cslibs_mapping)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

include(cmake/cslibs_mapping_enable_c++11.cmake)
include(cmake/cslibs_mapping_extras.cmake)
include(cmake/cslibs_mapping_show_headers.cmake)
include(cmake/cslibs_mapping_add_unit_test_gtest.cmake)

find_package(catkin REQUIRED COMPONENTS
    cslibs_gridmaps
    cslibs_time
    cslibs_utility
    cslibs_math_ros
    cslibs_ndt_3d
    cslibs_ndt_2d
    nav_msgs
    roscpp
    sensor_msgs
    geometry_msgs
    std_msgs
    pcl_ros
    rosbag
    message_generation
    octomap_msgs
    rviz
    ndt_map
)

find_package(PCL REQUIRED COMPONENTS common filters segmentation io)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(octomap REQUIRED)
#find_package(ndt_map QUIET)

add_message_files(
    FILES
    Distribution3d.msg
    Distribution3dArray.msg
)
add_service_files(
    FILES
    SaveMap.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    #INCLUDE_DIRS include
    #LIBRARIES muse_mcl_2d_mapping
    #CATKIN_DEPENDS cslibs_ndt_3d muse_mcl_2d_gridmaps muse_mcl_2d_ndt
    #DEPENDS system_lib
    )

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${pcl_ros_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
    ${ndt_map_INCLUDE_DIRS}
)

cslibs_mapping_show_headers()

link_directories(${PCL_LIBRARY_DIRS})

add_library(${PROJECT_NAME}_2d
    src/mapper/occupancy_grid_mapper_2d.cpp
    src/mapper/ndt_grid_mapper_2d.cpp
    src/mapper/occupancy_ndt_grid_mapper_2d.cpp
    )
target_link_libraries(${PROJECT_NAME}_2d
    ${catkin_LIBRARIES}
    yaml-cpp
    )

add_library(${PROJECT_NAME}_3d
    src/mapper/ndt_grid_mapper_3d.cpp
    src/mapper/occupancy_ndt_grid_mapper_3d.cpp
    src/mapper/octomap_mapper_3d.cpp
    )
target_link_libraries(${PROJECT_NAME}_3d
    ${catkin_LIBRARIES}
    yaml-cpp
    ${OCTOMAP_LIBRARIES}
    )
add_dependencies(${PROJECT_NAME}_3d ${${PROJECT_NAME}_EXPORTED_TARGETS})


add_executable(${PROJECT_NAME}_node_2d
    src/nodes/mapper_node_2d.cpp
    )
target_link_libraries(${PROJECT_NAME}_node_2d
    ${PROJECT_NAME}_2d
    ${catkin_LIBRARIES}
     yaml-cpp
    )
add_dependencies(${PROJECT_NAME}_node_2d ${${PROJECT_NAME}_EXPORTED_TARGETS})

#if(${ndt_map_found})
    add_definitions("-DWITH_ORU_NDT")
#endif()

add_executable(${PROJECT_NAME}_node_3d
    src/nodes/mapper_node_3d.cpp
    )
target_link_libraries(${PROJECT_NAME}_node_3d
    ${PROJECT_NAME}_2d
    ${PROJECT_NAME}_3d
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    yaml-cpp
    ${OCTOMAP_LIBRARIES}
    ${ndt_map_LIBRARIES}
    )
add_dependencies(${PROJECT_NAME}_node_3d ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_ndt_map_loader_3d
    src/nodes/ndt_map_loader_3d.cpp
)
target_link_libraries(${PROJECT_NAME}_ndt_map_loader_3d
    ${PROJECT_NAME}_3d
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

set(CMAKE_AUTOMOC ON)
include_directories(${rviz_INCLUDE_DIRS})

if(rviz_QT_VERSION VERSION_LESS "5")
    message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
    find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
    ## pull in all required include dirs, define QT_LIBRARIES, etc.
    include(${QT_USE_FILE})
else()
    message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
    find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
    ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
    set(QT_LIBRARIES Qt5::Widgets)
endif()

add_library(${PROJECT_NAME}_rviz
    src/rviz/ndt_visual_3d.cpp
    src/rviz/ndt_display_3d.cpp
    src/rviz/ndt_ellipsoid_3d.cpp
    src/rviz/ndt_mesh_3d.cpp
    )
target_link_libraries(${PROJECT_NAME}_rviz
    ${PROJECT_NAME}_2d
    ${PROJECT_NAME}_3d
    ${QT_LIBRARIES}
    ${catkin_LIBRARIES}
    )
add_dependencies(${PROJECT_NAME}_rviz ${${PROJECT_NAME}_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME}_rviz
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
install(FILES plugin_description.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY ogre_media
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY icons/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
