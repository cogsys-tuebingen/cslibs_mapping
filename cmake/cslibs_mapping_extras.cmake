add_definitions(-W
                -Wall
                -Wno-unused-parameter
                -Wno-unused-function
                -fno-strict-aliasing)

if(${PROJECT_NAME}_USE_NATIVE OR CS_USE_NATIVE)
    message("${PROJECT_NAME} using -march=native!")
    add_definitions(-march=native)
endif()


if(NOT ${CMAKE_BUILD_TYPE} STREQUAL Debug)
    add_definitions(-Ofast
                     -rdynamic
                     -ffast-math)
    message("[${PROJECT_NAME}]: Compiling with optimization!")
endif()
