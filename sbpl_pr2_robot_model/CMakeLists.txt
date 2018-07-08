cmake_minimum_required(VERSION 2.8)
project(sbpl_pr2_robot_model)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#list(APPEND CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin
    REQUIRED
    COMPONENTS
        pr2_arm_kinematics
        leatherman
        roscpp
        sbpl_kdl_robot_model
        smpl
        urdf)

find_package(orocos_kdl REQUIRED)

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        sbpl_pr2_robot_model
    CATKIN_DEPENDS
        pr2_arm_kinematics
        roscpp
        sbpl_kdl_robot_model
        urdf
    DEPENDS
        orocos_kdl)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
include_directories(SYSTEM ${orocos_kdl_INCLUDE_DIRS})
include_directories(include)

add_library(
    sbpl_pr2_robot_model
    src/sbpl_math.cpp
    src/orientation_solver.cpp
    src/pr2_kdl_robot_model.cpp
    src/pr2_ikfast_robot_model.cpp
    src/ubr1_kdl_robot_model.cpp)

target_link_libraries(sbpl_pr2_robot_model ${catkin_LIBRARIES})

install(
    DIRECTORY include/sbpl_pr2_robot_model/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(
    TARGETS sbpl_pr2_robot_model
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
