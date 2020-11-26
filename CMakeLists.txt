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
        ${PROJECT_NAME}_mappers
        ${PROJECT_NAME}_publishers
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

# basic libs and dirs
set(TARGET_LINK_LIBRARIES
    ${catkin_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
)
set(TARGET_INCLUDE_DIRS
    include/
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
)

# basic src files
set(TARGET_SOURCE_FILES_MAPPERS
    src/mapper/ndt_grid_mapper_2d.cpp
    src/mapper/ndt_grid_mapper_3d.cpp
    src/mapper/occupancy_grid_mapper_2d.cpp
    src/mapper/occupancy_ndt_grid_mapper_2d.cpp
    src/mapper/occupancy_ndt_grid_mapper_3d.cpp
    src/mapper/min_height_mapper_2d.cpp
    src/mapper/distribution_height_mapper_2d.cpp
)
set(TARGET_SOURCE_FILES_PUBLISHERS
    src/publisher/distributions_publisher.cpp
    src/publisher/occupancy_grid_publisher.cpp
    src/publisher/pointcloud_publisher.cpp
)

# additional libs, dirs and src files with octomap
if (octomap_FOUND)
    # octomap requires octomap_msgs
    find_package(octomap_msgs REQUIRED)
    list(APPEND TARGET_LINK_LIBRARIES
        ${OCTOMAP_LIBRARIES}
    )
    list(APPEND TARGET_INCLUDE_DIRS
        ${OCTOMAP_INCLUDE_DIRS}
    )
    list(APPEND TARGET_SOURCE_FILES_MAPPERS
        src/mapper/occupancy_grid_mapper_3d.cpp
    )
    list(APPEND TARGET_SOURCE_FILES_PUBLISHERS
        src/publisher/octomap_publisher.cpp
    )
endif()

# additional libs, dirs and src files with ndt_map
if (ndt_map_FOUND)
    find_package(PCL REQUIRED COMPONENTS common filters segmentation io)
    list(APPEND TARGET_LINK_LIBRARIES
        ${ndt_map_LIBRARIES}
    )
    list(APPEND TARGET_INCLUDE_DIRS
        ${ndt_map_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
    )
    list(APPEND TARGET_SOURCE_FILES_MAPPERS
        src/mapper/oru_ndt_om_grid_mapper_3d.cpp
        src/mapper/oru_ndt_omfg_grid_mapper_3d.cpp
        src/mapper/oru_ndt_grid_mapper_3d.cpp
        src/mapper/oru_ndt_om_grid_mapper_2d.cpp
        src/mapper/oru_ndt_omfg_grid_mapper_2d.cpp
        src/mapper/oru_ndt_grid_mapper_2d.cpp
    )
    list(APPEND TARGET_SOURCE_FILES_PUBLISHERS
        src/publisher/oru_publisher.cpp
    )
endif()


add_library(${PROJECT_NAME}_mappers
    ${TARGET_SOURCE_FILES_MAPPERS}
)
target_include_directories(${PROJECT_NAME}_mappers
    PRIVATE
        ${TARGET_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_mappers
    PRIVATE
        ${TARGET_LINK_LIBRARIES}
)
target_compile_options(${PROJECT_NAME}_mappers
    PRIVATE
        ${TARGET_COMPILE_OPTIONS}
)
add_dependencies(${PROJECT_NAME}_mappers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(${PROJECT_NAME}_publishers
    ${TARGET_SOURCE_FILES_PUBLISHERS}
)
target_include_directories(${PROJECT_NAME}_publishers
    PRIVATE
        ${TARGET_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_publishers
    PRIVATE
        ${TARGET_LINK_LIBRARIES}
)
target_compile_options(${PROJECT_NAME}_publishers
    PRIVATE
        ${TARGET_COMPILE_OPTIONS}
)

add_dependencies(${PROJECT_NAME}_publishers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_node
    src/nodes/mapping_node.cpp
)

target_include_directories(${PROJECT_NAME}_node
    PRIVATE
        ${TARGET_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_node
    PRIVATE
        ${PROJECT_NAME}_mappers
        ${PROJECT_NAME}_publishers
        ${catkin_LIBRARIES}
)

target_compile_options(${PROJECT_NAME}_node
    PRIVATE
        ${TARGET_COMPILE_OPTIONS}
)

install(FILES plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS ${PROJECT_NAME}_mappers ${PROJECT_NAME}_publishers #${PROJECT_NAME}_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

