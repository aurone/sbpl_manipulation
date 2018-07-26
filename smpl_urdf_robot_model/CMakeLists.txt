cmake_minimum_required(VERSION 2.8.12)
project(smpl_urdf_robot_model)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp smpl smpl_ros)
find_package(urdfdom REQUIRED)
find_package(urdfdom_headers REQUIRED)

catkin_package(
    INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
    LIBRARIES smpl_urdf_robot_model)

add_library(
    smpl_urdf_robot_model
    src/robot_model.cpp
    src/robot_state.cpp
    src/robot_state_bounds.cpp
    src/robot_state_visualization.cpp
    src/urdf_robot_model.cpp)
target_compile_options(smpl_urdf_robot_model PUBLIC -std=c++11)
target_include_directories(smpl_urdf_robot_model PUBLIC include)
target_include_directories(smpl_urdf_robot_model SYSTEM PRIVATE ${urdfdom_headers_INCLUDE_DIRS})
target_include_directories(smpl_urdf_robot_model SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR})
target_include_directories(smpl_urdf_robot_model SYSTEM PUBLIC ${smpl_INCLUDE_DIRS})
target_link_libraries(smpl_urdf_robot_model PUBLIC ${smpl_LIBRARIES})
target_link_libraries(smpl_urdf_robot_model PRIVATE ${urdfdom_LIBRARIES})

add_executable(robot_model_test src/robot_model_test.cpp)
target_include_directories(robot_model_test PRIVATE ${urdfdom_INCLUDE_DIRS})
target_include_directories(robot_model_test PRIVATE ${smpl_INCLUDE_DIRS})
target_include_directories(robot_model_test PRIVATE ${smpl_ros_INCLUDE_DIRS})
target_include_directories(robot_model_test PRIVATE ${roscpp_INCLUDE_DIRS})
target_link_libraries(robot_model_test smpl_urdf_robot_model)
target_link_libraries(robot_model_test ${urdfdom_LIBRARIES})
target_link_libraries(robot_model_test ${smpl_LIBRARIES})
target_link_libraries(robot_model_test ${smpl_ros_LIBRARIES})
target_link_libraries(robot_model_test ${roscpp_LIBRARIES})

install(
    DIRECTORY include/smpl_urdf_robot_model/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(
    TARGETS smpl_urdf_robot_model
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
