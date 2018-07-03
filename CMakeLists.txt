cmake_minimum_required(VERSION 2.8.3)
project(cslibs_mapping)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

include(cmake/cslibs_mapping_enable_c++11.cmake)
include(cmake/cslibs_mapping_extras.cmake)
include(cmake/cslibs_mapping_show_headers.cmake)
include(cmake/cslibs_mapping_add_unit_test_gtest.cmake)
include(cmake/cslibs_mapping_eigen_cpu_flags.cmake)

find_package(catkin REQUIRED COMPONENTS
    cslibs_plugins
    cslibs_plugins_data
    cslibs_gridmaps
    cslibs_ndt_2d
    cslibs_ndt_3d
    cslibs_time
    cslibs_utility
    cslibs_math_ros
    message_generation
    std_msgs
    pluginlib
)

find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(PCL REQUIRED COMPONENTS common filters segmentation io QUIET)
find_package(octomap QUIET)
find_package(yaml-cpp REQUIRED)
find_package(ndt_map QUIET)

add_service_files(
    FILES
    SaveMap.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES    ${PROJECT_NAME}
    CATKIN_DEPENDS cslibs_plugins cslibs_plugins_data cslibs_gridmaps cslibs_ndt_2d cslibs_ndt_3d
                   cslibs_time cslibs_utility cslibs_math_ros
    CFG_EXTRAS cslibs_mapping_enable_c++11.cmake
               cslibs_mapping_extras.cmake
               cslibs_mapping_show_headers.cmake
               cslibs_mapping_add_unit_test_gtest.cmake
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)
if(PCL_FOUND)
    find_package(pcl_ros REQUIRED)
    include_directories(
        ${PCL_INCLUDE_DIRS}
        ${pcl_ros_INCLUDE_DIRS}
    )
    if(octomap_FOUND)
        find_package(octomap_msgs REQUIRED)
        include_directories(
            ${OCTOMAP_INCLUDE_DIRS}
        )
    endif()
    if(ndt_map_FOUND)
        include_directories(
            ${ndt_map_INCLUDE_DIRS}
        )
    endif()
endif()

cslibs_mapping_show_headers()

if(PCL_FOUND)
    if (octomap_FOUND)
        if (ndt_map_FOUND)
            #pcl + octomap + ndt_map
            add_library(${PROJECT_NAME}
                src/mapper/ndt_grid_mapper_2d.cpp
                src/mapper/ndt_grid_mapper_3d.cpp
                src/mapper/occupancy_grid_mapper_2d.cpp
                src/mapper/occupancy_grid_mapper_3d.cpp
                src/mapper/occupancy_ndt_grid_mapper_2d.cpp
                src/mapper/occupancy_ndt_grid_mapper_3d.cpp
                src/mapper/oru_ndt_grid_mapper_3d.cpp
                src/publisher/distributions_publisher.cpp
                src/publisher/occupancy_grid_publisher.cpp
                src/publisher/octomap_publisher.cpp
                src/publisher/pointcloud_publisher.cpp
            )
            target_link_libraries(${PROJECT_NAME}
                ${catkin_LIBRARIES}
                ${PCL_LIBRARIES}
                ${OCTOMAP_LIBRARIES}
                yaml-cpp
                ${ndt_map_LIBRARIES}
            )
        else()
            #pcl + octomap
            add_library(${PROJECT_NAME}
                src/mapper/ndt_grid_mapper_2d.cpp
                src/mapper/ndt_grid_mapper_3d.cpp
                src/mapper/occupancy_grid_mapper_2d.cpp
                src/mapper/occupancy_grid_mapper_3d.cpp
                src/mapper/occupancy_ndt_grid_mapper_2d.cpp
                src/mapper/occupancy_ndt_grid_mapper_3d.cpp
                src/publisher/distributions_publisher.cpp
                src/publisher/occupancy_grid_publisher.cpp
                src/publisher/octomap_publisher.cpp
                src/publisher/pointcloud_publisher.cpp
            )
            target_link_libraries(${PROJECT_NAME}
                ${catkin_LIBRARIES}
                ${PCL_LIBRARIES}
                ${OCTOMAP_LIBRARIES}
                yaml-cpp
            )
        endif()
    else()
        if (ndt_map_FOUND)
            #pcl + ndt_map
            add_library(${PROJECT_NAME}
                src/mapper/ndt_grid_mapper_2d.cpp
                src/mapper/ndt_grid_mapper_3d.cpp
                src/mapper/occupancy_grid_mapper_2d.cpp
                src/mapper/occupancy_ndt_grid_mapper_2d.cpp
                src/mapper/occupancy_ndt_grid_mapper_3d.cpp
                src/mapper/oru_ndt_grid_mapper_3d.cpp
                src/publisher/distributions_publisher.cpp
                src/publisher/occupancy_grid_publisher.cpp
                src/publisher/pointcloud_publisher.cpp
                )
            target_link_libraries(${PROJECT_NAME}
                ${catkin_LIBRARIES}
                ${PCL_LIBRARIES}
                yaml-cpp
                ${ndt_map_LIBRARIES}
                )
        else()
            #pcl
            add_library(${PROJECT_NAME}
                src/mapper/ndt_grid_mapper_2d.cpp
                src/mapper/ndt_grid_mapper_3d.cpp
                src/mapper/occupancy_grid_mapper_2d.cpp
                src/mapper/occupancy_ndt_grid_mapper_2d.cpp
                src/mapper/occupancy_ndt_grid_mapper_3d.cpp
                src/publisher/distributions_publisher.cpp
                src/publisher/occupancy_grid_publisher.cpp
                src/publisher/pointcloud_publisher.cpp
                )
            target_link_libraries(${PROJECT_NAME}
                ${catkin_LIBRARIES}
                ${PCL_LIBRARIES}
                yaml-cpp
                )
        endif()
    endif()
    link_directories(${PCL_LIBRARY_DIRS})
else()
    #no pcl and no octomap or ndt_map
    add_library(${PROJECT_NAME}
        src/mapper/ndt_grid_mapper_2d.cpp
        src/mapper/occupancy_grid_mapper_2d.cpp
        src/mapper/occupancy_ndt_grid_mapper_2d.cpp
        src/publisher/occupancy_grid_publisher.cpp
    )
    target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        yaml-cpp
    )
endif()


add_executable(${PROJECT_NAME}_node
    src/nodes/mapping_node.cpp
)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
     yaml-cpp
)


install(FILES plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

