cmake_minimum_required(VERSION 3.0.2)
project(cslibs_mapping CXX)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(cmake/cslibs_mapping_eigen_cpu_flags.cmake)


find_package(catkin REQUIRED COMPONENTS
    cslibs_plugins
    cslibs_plugins_data
    cslibs_gridmaps
    cslibs_ndt_2d
    cslibs_ndt_3d
    cslibs_time
    cslibs_utility
    cslibs_math
    cslibs_math_ros
    cslibs_indexed_storage
    message_generation
    std_msgs
    pluginlib
    roscpp
)

find_package(Boost REQUIRED COMPONENTS filesystem)
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
    INCLUDE_DIRS
        include
    LIBRARIES
        ${PROJECT_NAME}
    CATKIN_DEPENDS
        cslibs_plugins
        cslibs_plugins_data
        cslibs_gridmaps
        cslibs_ndt_2d
        cslibs_ndt_3d
        cslibs_time
        cslibs_utility
        cslibs_math
        cslibs_math_ros
        pluginlib
        roscpp
    CFG_EXTRAS
        cslibs_mapping_eigen_cpu_flags.cmake
)

if(NOT ${CMAKE_BUILD_TYPE} STREQUAL Debug)
    list(APPEND TARGET_COMPILE_OPTIONS
        -Wall -Werror -Ofast -ffast-math
    )
    # linker flag -rdynamic
    # enabling -march=native my result in problems on some machines,
    # my try -mtune=native

    message(STATUS "[${PROJECT_NAME}]: Compiling with optimization!")
endif()

if(octomap_FOUND)
    find_package(octomap_msgs REQUIRED)
endif()

set(TARGET_LINK_LIBRARIES
    ${catkin_LIBRARIES}
    ${OCTOMAP_LIBRARIES}
    ${ndt_map_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
)

set(TARGET_INCLUDE_DIRS
    include/
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
    ${ndt_map_INCLUDE_DIRS}
)


#add_definitions("-DEIGEN_MAX_ALIGN_BYTES=16")

if (octomap_FOUND)
    if (ndt_map_FOUND)
        find_package(PCL REQUIRED COMPONENTS common filters segmentation io)
        list(append TARGET_INCLUDE_DIRS
            ${PCL_INCLUDE_DIRS}
        )
        set(TARGET_SOURCE_FILES
            src/mapper/ndt_grid_mapper_2d.cpp
            src/mapper/ndt_grid_mapper_3d.cpp
            src/mapper/occupancy_grid_mapper_2d.cpp
            src/mapper/occupancy_grid_mapper_3d.cpp
            src/mapper/occupancy_ndt_grid_mapper_2d.cpp
            src/mapper/occupancy_ndt_grid_mapper_3d.cpp
            src/mapper/min_height_mapper_2d.cpp
            src/mapper/distribution_height_mapper_2d.cpp
            src/mapper/oru_ndt_om_grid_mapper_3d.cpp
            src/mapper/oru_ndt_omfg_grid_mapper_3d.cpp
            src/publisher/distributions_publisher.cpp
            src/publisher/occupancy_grid_publisher.cpp
            src/publisher/octomap_publisher.cpp
            src/publisher/pointcloud_publisher.cpp
            src/publisher/oru_publisher.cpp
        )

        #octomap + oru + pcl
    else()
        #octomap
        set(TARGET_SOURCE_FILES
            src/mapper/ndt_grid_mapper_2d.cpp
            src/mapper/ndt_grid_mapper_3d.cpp
            src/mapper/occupancy_grid_mapper_2d.cpp
            src/mapper/occupancy_grid_mapper_3d.cpp
            src/mapper/occupancy_ndt_grid_mapper_2d.cpp
            src/mapper/occupancy_ndt_grid_mapper_3d.cpp
            src/mapper/min_height_mapper_2d.cpp
            src/mapper/distribution_height_mapper_2d.cpp
            src/publisher/distributions_publisher.cpp
            src/publisher/occupancy_grid_publisher.cpp
            src/publisher/octomap_publisher.cpp
            src/publisher/pointcloud_publisher.cpp
        )
    endif()
else()
    if (ndt_map_FOUND)
        find_package(PCL REQUIRED COMPONENTS common filters segmentation io)
        list(append TARGET_INCLUDE_DIRS
            ${PCL_INCLUDE_DIRS}
        )
        #no octomap + oru + pcl
        set(TARGET_SOURCE_FILES
            src/mapper/ndt_grid_mapper_2d.cpp
            src/mapper/ndt_grid_mapper_3d.cpp
            src/mapper/occupancy_grid_mapper_2d.cpp
            src/mapper/occupancy_ndt_grid_mapper_2d.cpp
            src/mapper/occupancy_ndt_grid_mapper_3d.cpp
            src/mapper/min_height_mapper_2d.cpp
            src/mapper/distribution_height_mapper_2d.cpp
            src/mapper/oru_ndt_om_grid_mapper_3d.cpp
            src/mapper/oru_ndt_omfg_grid_mapper_3d.cpp
            src/publisher/distributions_publisher.cpp
            src/publisher/occupancy_grid_publisher.cpp
            src/publisher/pointcloud_publisher.cpp
            src/publisher/oru_publisher.cpp
        )
    else()
        #no octomap
        set(TARGET_SOURCE_FILES
            src/mapper/ndt_grid_mapper_2d.cpp
            src/mapper/ndt_grid_mapper_3d.cpp
            src/mapper/occupancy_grid_mapper_2d.cpp
            src/mapper/occupancy_ndt_grid_mapper_2d.cpp
            src/mapper/occupancy_ndt_grid_mapper_3d.cpp
            src/mapper/min_height_mapper_2d.cpp
            src/mapper/distribution_height_mapper_2d.cpp
            src/publisher/distributions_publisher.cpp
            src/publisher/occupancy_grid_publisher.cpp
            src/publisher/pointcloud_publisher.cpp
        )
    endif()
endif()

add_library(${PROJECT_NAME}
    ${TARGET_SOURCE_FILES}
)

target_include_directories(${PROJECT_NAME}
    PRIVATE
        ${TARGET_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
    PRIVATE
        ${TARGET_LINK_LIBRARIES}
)
target_compile_options(${PROJECT_NAME}
    PRIVATE
        ${TARGET_COMPILE_OPTIONS}
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_node
    src/nodes/mapping_node.cpp
)

target_include_directories(${PROJECT_NAME}_node
    PRIVATE
        ${TARGET_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_node
    PRIVATE
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
)

target_compile_options(${PROJECT_NAME}_node
    PRIVATE
        ${TARGET_COMPILE_OPTIONS}
)

install(FILES plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS ${PROJECT_NAME} #${PROJECT_NAME}_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

