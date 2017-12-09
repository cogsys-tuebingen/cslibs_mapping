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
  message_generation
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation io)

add_message_files(
    FILES
    Distribution3d.msg
    Distributions3d.msg
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
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${pcl_ros_INCLUDE_DIRS}
)

cslibs_mapping_show_headers()

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
add_dependencies(${PROJECT_NAME}_3d ${${PROJECT_NAME}_EXPORTED_TARGETS})


add_executable(${PROJECT_NAME}_node_2d
    src/nodes/mapper_node_2d.cpp
)
target_link_libraries(${PROJECT_NAME}_node_2d
    ${PROJECT_NAME}_2d
    ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}_node_2d ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_node_3d
    src/nodes/mapper_node_3d.cpp
)
target_link_libraries(${PROJECT_NAME}_node_3d
    ${PROJECT_NAME}_2d
    ${PROJECT_NAME}_3d
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}_node_3d ${${PROJECT_NAME}_EXPORTED_TARGETS})

find_package(Qt5 COMPONENTS Core Gui Widgets 3DCore 3DRenderer 3DInput REQUIRED)
if(${Qt5_FOUND})
#   install package qt3d5-dev
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
    set(CMAKE_AUTOMOC ON)
    include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_BINARY_DIR}
    )

    qt5_wrap_ui(QT_UI
        src/nodes/qt/ui/cslibs_mapping_viewer_node_3d.ui
        src/nodes/qt/ui/widget.ui
    )

    message("[cslibs_mapping]: Building 3d viewer node.")

    add_executable(${PROJECT_NAME}_viewer_node_3d
        src/nodes/viewer_node_3d.cpp
        src/nodes/qt/scenemodifier.cpp
        ${QT_UI}
    )
    target_link_libraries(${PROJECT_NAME}_viewer_node_3d
        ${catkin_LIBRARIES}
        Qt5::Core
        Qt5::Gui
        Qt5::Widgets
        Qt5::3DCore
        Qt5::3DRenderer
        Qt5::3DInput
    )
else()
    message("[cslibs_mapping]: Building 3d viewer node requires minimum version of Qt5.10.")
endif()

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
