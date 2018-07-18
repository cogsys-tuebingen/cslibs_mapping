add_definitions(-W -Wall -Wno-unused-parameter -fno-strict-aliasing -Wno-unused-function -Wno-deprecated-register)
if(NOT ${CMAKE_BUILD_TYPE} STREQUAL Debug)
    add_definitions("-Ofast -g -rdynamic -march=native -ffast-math")
    message("[${PROJECT_NAME}]: Compiling with optimization!")
else()
    add_definitions("-Ofast -g -rdynamic")#" -march=native -ffast-math") sonst tut ndt mapping nicht, pcl tut aber nur mit debug
endif()
