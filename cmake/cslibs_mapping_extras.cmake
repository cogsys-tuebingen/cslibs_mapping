add_definitions(-W
                -Wall
                -Wno-unused-parameter
                -Wno-unused-function
                -fno-strict-aliasing
                -Wno-deprecated-register
                -march=native)

if(NOT ${CMAKE_BUILD_TYPE} STREQUAL Debug)
    add_definitions(-Ofast
                     -rdynamic
                     -ffast-math)
    message("[${PROJECT_NAME}]: Compiling with optimization!")
else()
    add_definitions("-Ofast -g -rdynamic")#" -march=native -ffast-math") sonst tut ndt mapping nicht, pcl tut aber nur mit debug
endif()
