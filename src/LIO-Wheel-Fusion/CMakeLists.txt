cmake_minimum_required(VERSION 2.8.3)
project(lio_wheel)

SET(CMAKE_BUILD_TYPE "Release")

ADD_COMPILE_OPTIONS(-std=c++14)
ADD_COMPILE_OPTIONS(-std=c++14)
set(CMAKE_CXX_FLAGS "-std=c++14 -O3")

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -std=c++0x -std=c++14 -fexceptions")

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if (CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
    include(ProcessorCount)
    ProcessorCount(N)
    message("Processer number:  ${N}")
    if (N GREATER 4)
        add_definitions(-DMP_EN)
        add_definitions(-DMP_PROC_NUM=3)
        message("core for MP: 3")
    elseif (N GREATER 3)
        add_definitions(-DMP_EN)
        add_definitions(-DMP_PROC_NUM=2)
        message("core for MP: 2")
    else ()
        add_definitions(-DMP_PROC_NUM=1)
    endif ()
else ()
    add_definitions(-DMP_PROC_NUM=1)
endif ()

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

find_package(PythonLibs REQUIRED)
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")

find_package(catkin REQUIRED COMPONENTS
        std_msgs
        geometry_msgs
        nav_msgs
        sensor_msgs
        customized_msgs
        roscpp
        rospy
        tf
        message_generation
        eigen_conversions
        cv_bridge
        )

find_package(PCL 1.8 REQUIRED)
add_subdirectory(Thirdparty/fmt)
find_package(OpenCV)

set(ALL_TARGET_LIBRARIES "")
include(cmake/global_definition.cmake)
include(cmake/glog.cmake)
include(cmake/YAML.cmake)
include(cmake/eigen.cmake)
include(cmake/OpenCV.cmake)
include(cmake/FindEigen.cmake)
include(cmake/ceres_solver.cmake)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${PYTHON_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/include)

message(${PYTHON_INCLUDE_DIRS})

add_message_files(
        FILES
        Pose6D.msg
)

generate_messages(
        DEPENDENCIES
        geometry_msgs
)

catkin_package(
        CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime
        DEPENDS EIGEN3 PCL
        INCLUDE_DIRS
)

file(GLOB_RECURSE ALL_SRCS "src/*")
file(GLOB_RECURSE LASER_MAPPING_MAIN "src/laserMapping.cpp")
list(REMOVE_ITEM ALL_SRCS ${LASER_MAPPING_MAIN})

add_executable(fastlio_mapping src/laserMapping.cpp include/ikd-Tree/ikd_Tree.cpp src/preprocess.cpp  ${ALL_SRCS})
target_link_libraries(fastlio_mapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES} fmt::fmt-header-only  ${ALL_TARGET_LIBRARIES})
target_include_directories(fastlio_mapping PRIVATE ${PYTHON_INCLUDE_DIRS})

add_executable(dr_odo_node nodecpp/pose_source/dr_odo_node.cpp ${ALL_SRCS})
target_link_libraries(dr_odo_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES} ${ALL_TARGET_LIBRARIES} fmt::fmt-header-only)
target_include_directories(dr_odo_node PRIVATE ${PYTHON_INCLUDE_DIRS})

add_executable(distortion_removal_node nodecpp/distortion_removal_node.cpp ${ALL_SRCS})
target_link_libraries(distortion_removal_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES} ${ALL_TARGET_LIBRARIES} fmt::fmt-header-only)
target_include_directories(distortion_removal_node PRIVATE ${PYTHON_INCLUDE_DIRS})