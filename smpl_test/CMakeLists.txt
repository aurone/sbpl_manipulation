cmake_minimum_required(VERSION 2.8)
project(smpl_test)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(Boost REQUIRED COMPONENTS unit_test_framework)

find_package(Eigen3 REQUIRED)

find_package(catkin
    REQUIRED
    COMPONENTS
        geometry_msgs
        leatherman
        moveit_msgs
        moveit_ros_planning
        roscpp
        sbpl_collision_checking
        sbpl_kdl_robot_model
        smpl_ompl_interface
        visualization_msgs)

find_package(orocos_kdl REQUIRED)
find_package(ompl REQUIRED)
find_package(smpl REQUIRED)

catkin_package()

add_definitions(-DSV_PACKAGE_NAME="smpl_test")

include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
include_directories(SYSTEM ${orocos_kdl_INCLUDE_DIRS})

add_executable(callPlanner src/call_planner.cpp src/collision_space_scene.cpp)
target_link_libraries(callPlanner ${catkin_LIBRARIES} smpl::smpl)

#add_executable(call_ompl_planner src/call_ompl_planner.cpp src/collision_space_scene.cpp)
#target_include_directories(call_ompl_planner SYSTEM PRIVATE ${OMPL_INCLUDE_DIRS})
#target_link_libraries(call_ompl_planner ${catkin_LIBRARIES} ${OMPL_LIBRARIES} smpl::smpl)

add_executable(occupancy_grid_test src/occupancy_grid_test.cpp)
target_link_libraries(occupancy_grid_test ${catkin_LIBRARIES} smpl::smpl)

add_executable(visualizer_test src/visualizer_test.cpp)
target_link_libraries(visualizer_test ${catkin_LIBRARIES} smpl::smpl)

add_executable(csv_parser_test src/csv_parser_test.cpp)
target_link_libraries(csv_parser_test ${catkin_LIBRARIES} smpl::smpl)

add_executable(heap_test src/heap_test.cpp)
target_link_libraries(heap_test ${Boost_LIBRARIES} ${catkin_LIBRARIES} smpl::smpl)

add_executable(egraph_test src/egraph_test.cpp)
target_link_libraries(egraph_test ${Boost_LIBRARIES} ${catkin_LIBRARIES} smpl::smpl)

add_executable(octree_test src/octree_tests.cpp)
target_link_libraries(octree_test ${Boost_LIBRARIES} smpl::smpl)

add_executable(sparse_grid_test src/sparse_grid_test.cpp)
target_link_libraries(sparse_grid_test ${Boost_LIBRARIES} smpl::smpl)

add_executable(sparse_binary_grid_test src/sparse_binary_grid_test.cpp)
target_link_libraries(sparse_binary_grid_test ${Boost_LIBRARIES} smpl::smpl)

add_executable(xytheta src/xytheta.cpp)
target_link_libraries(xytheta smpl::smpl)

add_executable(debug_vis_demo src/debug_vis_demo.cpp)
target_link_libraries(debug_vis_demo ${catkin_LIBRARIES} smpl::smpl)

#add_executable(distance_map_test src/distance_map_test.cpp)
#target_link_libraries(distance_map_test ${catkin_LIBRARIES} ${Boost_LIBRARIES} smpl::smpl)

install(
    TARGETS callPlanner
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

